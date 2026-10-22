/*
 * kronethercatmaster.c  --  KronEditor EtherCAT Master runtime (SOEM-backed)
 *
 * Pillar 1: Automatic init — kron_ec_init() called by PLC_Init(), not user code.
 * Pillar 2: Strict task sync — pdo_read before PLC logic, pdo_write after.
 * Pillar 3: User FBs — EC_GetMasterState, EC_GetSlaveState, EC_ResetBus,
 *                       EC_ReadSDO, EC_WriteSDO (async, via SDO queue).
 */

#ifndef KRON_EC_SIM

#include "kronethercatmaster.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>

/* ── SOEM v2 context ─────────────────────────────────────────────────────── */
static ecx_contextt  g_ctx;
static char          g_IOmap[4096];
static KRON_EC_Config *g_cfg_ptr = NULL;

/* ── EtherCAT context mutex ──────────────────────────────────────────────── */
/* How long to exchange process data after ecx_configdc() before arming SYNC0,
 * and how many pumped attempts a state transition gets. Both are deliberately
 * generous: they run once at start-up, and being too impatient here is what
 * leaves a conformant drive parked in SAFE-OP. */
#ifndef KRON_EC_DC_SETTLE_FRAMES
#  define KRON_EC_DC_SETTLE_FRAMES 2000
#endif
#ifndef KRON_EC_OP_ATTEMPTS
#  define KRON_EC_OP_ATTEMPTS 200
#endif

/* SOEM is NOT thread-safe: ecx_SDOread/write and ecx_send/receive_processdata
 * share the same socket. This mutex serializes SDO and PDO access. */
/* ⚠️ RECURSIVE, on purpose. kron_ec_check_state() holds it while calling
 * ecx_reconfig_slave(), and SOEM calls the PO2SO hook from in there; the hook's
 * SDO writes take it again. With a plain mutex the watchdog thread deadlocked
 * on itself the first time a slave had to be reconfigured — no startup SDOs,
 * and every thread that touches the bus hung behind it. */
static pthread_mutex_t g_ctx_mutex;
static pthread_once_t  g_ctx_mutex_once = PTHREAD_ONCE_INIT;

static void ctx_mutex_init(void) {
    pthread_mutexattr_t a;
    pthread_mutexattr_init(&a);
    pthread_mutexattr_settype(&a, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&g_ctx_mutex, &a);
    pthread_mutexattr_destroy(&a);
}
static void ctx_lock(void)   { pthread_once(&g_ctx_mutex_once, ctx_mutex_init); pthread_mutex_lock(&g_ctx_mutex); }
static void ctx_unlock(void) { pthread_mutex_unlock(&g_ctx_mutex); }

/* ── Global SDO queue ─────────────────────────────────────────────────────── */
static KRON_EC_SDO_Request sdo_queue[KRON_EC_MAX_SDO_QUEUE_SIZE];
static pthread_mutex_t sdo_queue_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ── Asynchronous bus-reset request ──────────────────────────────────────── */
/* kron_ec_init() performs blocking network I/O (bus scan plus statecheck
 * timeouts, ~10 s worst case with SOEM defaults). EC_ResetBus runs in the PLC
 * scan, so it must not call it directly — it posts a request here and the
 * background service thread (kron_ec_process_sdo) performs the reinit, the
 * same pattern the SDO blocks already use. */
#define KRON_EC_RESET_IDLE      0
#define KRON_EC_RESET_REQ       1
#define KRON_EC_RESET_RUNNING   2
#define KRON_EC_RESET_DONE_OK  -1
#define KRON_EC_RESET_DONE_ERR -2

static int g_reset_state = KRON_EC_RESET_IDLE;
static int g_reset_rc    = KRON_EC_OK;
static pthread_mutex_t g_reset_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ── helpers ──────────────────────────────────────────────────────────────── */

static uint8_t dtype_bytes(KRON_EC_DataType dt) {
    switch (dt) {
        case KRON_EC_DTYPE_BOOL:
        case KRON_EC_DTYPE_INT8:
        case KRON_EC_DTYPE_UINT8:  return 1;
        case KRON_EC_DTYPE_INT16:
        case KRON_EC_DTYPE_UINT16: return 2;
        case KRON_EC_DTYPE_INT32:
        case KRON_EC_DTYPE_UINT32:
        case KRON_EC_DTYPE_REAL32: return 4;
        case KRON_EC_DTYPE_INT64:
        case KRON_EC_DTYPE_UINT64:
        case KRON_EC_DTYPE_REAL64: return 8;
        default:                   return 1;
    }
}

/* Size of a slave's segment in the IOmap. SOEM reports Ibytes/Obytes as 0 when
 * fewer than 8 bits are mapped, but a whole byte is still allocated, so round
 * the bit count up instead of trusting the zero. */
static uint32_t slave_in_bytes(uint16_t pos) {
    uint32_t b = g_ctx.slavelist[pos].Ibytes;
    if (b == 0u && g_ctx.slavelist[pos].Ibits > 0u) b = 1u;
    return b;
}

static uint32_t slave_out_bytes(uint16_t pos) {
    uint32_t b = g_ctx.slavelist[pos].Obytes;
    if (b == 0u && g_ctx.slavelist[pos].Obits > 0u) b = 1u;
    return b;
}

static int do_sdo_write(uint16_t slave, uint16_t idx, uint8_t sub,
                        uint8_t bsz, uint32_t val) {
    ctx_lock();
    int wkc = ecx_SDOwrite(&g_ctx, slave, idx, sub, FALSE,
                           bsz, &val, EC_TIMEOUTRXM);
    ctx_unlock();
    return (wkc > 0) ? KRON_EC_OK : KRON_EC_ERR_IO;
}

static int do_sdo_read(uint16_t slave, uint16_t idx, uint8_t sub,
                       uint8_t bsz, uint32_t *out) {
    int sz = (int)bsz;
    uint32_t buf = 0;
    ctx_lock();
    int wkc = ecx_SDOread(&g_ctx, slave, idx, sub, FALSE, &sz, &buf, EC_TIMEOUTRXM);
    ctx_unlock();
    if (wkc > 0) { *out = buf; return KRON_EC_OK; }
    return KRON_EC_ERR_IO;
}

/*
 * Called by kron_ec_init(), i.e. also on an EC_ResetBus reinit while the PLC
 * scan is running. Slots owned by an EC_ReadSDO/EC_WriteSDO instance that is
 * currently Busy must NOT be silently reset to IDLE: their owner polls the
 * slot every scan and would wait on it forever. Fail them instead, so the
 * owning block reports ERR_SDO_FAILED and releases the slot itself.
 * At cold boot the array is static-zeroed, so every slot is already IDLE.
 */
static void kron_ec_init_sdo_queue(void) {
    pthread_mutex_lock(&sdo_queue_mutex);
    for (int i = 0; i < KRON_EC_MAX_SDO_QUEUE_SIZE; i++) {
        if (sdo_queue[i].state == KRON_EC_SDO_IDLE) {
            sdo_queue[i].slave_pos = 0;
            sdo_queue[i].index     = 0;
            sdo_queue[i].subindex  = 0;
            sdo_queue[i].byte_size = 0;
            sdo_queue[i].value     = 0;
        } else {
            /* In flight or awaiting pickup across a bus reinit — fail it. */
            sdo_queue[i].state = KRON_EC_SDO_DONE_ERR;
        }
    }
    pthread_mutex_unlock(&sdo_queue_mutex);
}

/* ── PO2SO hook — called by SOEM for each slave during PREOP→SAFEOP ─────── */

/* ⚠️ SOEM waits for PRE-OP before calling this hook, but only EC_TIMEOUTSTATE
 * (2 s) and it ignores the result. A drive that is slow to boot — typically the
 * FIRST start after power-on — is still in INIT then, has no mailbox, and every
 * startup SDO is aborted; the same project worked on the next Stop/Start, when
 * the drive was already up. So: wait up to 3 s and retry each write. What still
 * fails is handled per entry (KRON_EC_SDO.on_error, as in CODESYS' startup
 * parameters): ABORT stops the slave's list and kron_ec_init keeps the bus in
 * PRE-OP; NEXT logs it and carries on. Either way the slave records WHERE it
 * failed and the CoE abort code, which EC_GetSlaveState reports. */
#ifndef KRON_EC_PREOP_WAIT_US
#  define KRON_EC_PREOP_WAIT_US    3000000    /* 3 s */
#endif
#ifndef KRON_EC_SDO_INIT_ATTEMPTS
#  define KRON_EC_SDO_INIT_ATTEMPTS 5
#endif
#ifndef KRON_EC_SDO_INIT_RETRY_US
#  define KRON_EC_SDO_INIT_RETRY_US 200000    /* 200 ms */
#endif

static int g_sdo_init_failed = 0;   /* reset by kron_ec_init before mapping */

/* Drain SOEM's error list. Returns the CoE abort code of the last SDO error
 * in it (the slave's own reason for refusing), 0 if the slave never answered. */
static uint32_t ec_take_abort(ecx_contextt *ctx) {
    uint32_t   code = 0;
    ec_errort  e;
    while (ecx_poperror(ctx, &e)) {
        if (e.Etype == EC_ERR_TYPE_SDO_ERROR) code = (uint32_t)e.AbortCode;
    }
    return code;
}

static const char *ec_abort_text(uint32_t code) {
    return code ? ec_sdoerror2string(code) : "no answer (mailbox timeout, slave not in PRE-OP?)";
}

static int kron_po2so_hook(ecx_contextt *ctx, uint16 slave) {
    if (!g_cfg_ptr) return 1;
    KRON_EC_Slave *sl = NULL;
    for (int si = 0; si < g_cfg_ptr->slave_count; si++) {
        if (g_cfg_ptr->slaves[si].position == slave) { sl = &g_cfg_ptr->slaves[si]; break; }
    }
    if (!sl) return 1;
    sl->sdo_aborted      = false;
    sl->sdo_err_index    = 0;
    sl->sdo_err_subindex = 0;
    sl->sdo_abort_code   = 0;
    if (sl->sdo_count == 0) return 1;

    uint16 st = ecx_statecheck(ctx, slave, EC_STATE_PRE_OP, KRON_EC_PREOP_WAIT_US);
    if ((st & 0x0F) != EC_STATE_PRE_OP)
        fprintf(stderr, "[kronec] Slave %d not in PRE-OP before startup SDOs (state=0x%02X) — "
                        "writing anyway, expect aborts\n", slave, st);

    for (int i = 0; i < sl->sdo_count; i++) {
        KRON_EC_SDO *s = &sl->sdo_inits[i];
        int      attempt, ok = 0;
        uint32_t abort_code = 0;
        (void)ec_take_abort(ctx);   /* start from a clean error list */
        for (attempt = 1; attempt <= KRON_EC_SDO_INIT_ATTEMPTS; attempt++) {
            if (do_sdo_write(slave, s->index, s->subindex, s->byte_size, s->value) == KRON_EC_OK) {
                ok = 1;
                break;
            }
            abort_code = ec_take_abort(ctx);
            fprintf(stderr, "[kronec] Startup SDO slave %d 0x%04X:%02X = 0x%X (%u B) attempt %d/%d failed: "
                            "abort 0x%08X %s\n",
                    slave, s->index, s->subindex, (unsigned)s->value, s->byte_size,
                    attempt, KRON_EC_SDO_INIT_ATTEMPTS, (unsigned)abort_code, ec_abort_text(abort_code));
            usleep(KRON_EC_SDO_INIT_RETRY_US);
        }
        if (!ok) {
            sl->sdo_err_index    = s->index;
            sl->sdo_err_subindex = s->subindex;
            sl->sdo_abort_code   = abort_code;
            if (s->on_error == KRON_EC_SDO_ON_ERROR_NEXT) {
                fprintf(stderr, "[kronec] Startup SDO slave %d entry %d/%d 0x%04X:%02X SKIPPED (on error: next line), "
                                "abort 0x%08X %s\n",
                        slave, i + 1, sl->sdo_count, s->index, s->subindex,
                        (unsigned)abort_code, ec_abort_text(abort_code));
                continue;
            }
            sl->sdo_aborted = true;
            g_sdo_init_failed++;
            fprintf(stderr, "[kronec] Startup SDO slave %d entry %d/%d 0x%04X:%02X ABORTED (on error: abort), "
                            "abort 0x%08X %s — %d remaining entr%s not sent, slave stays in PRE-OP\n",
                    slave, i + 1, sl->sdo_count, s->index, s->subindex,
                    (unsigned)abort_code, ec_abort_text(abort_code),
                    sl->sdo_count - i - 1, (sl->sdo_count - i - 1) == 1 ? "y" : "ies");
            break;
        }
        if (attempt > 1)
            fprintf(stderr, "[kronec] Startup SDO slave %d 0x%04X:%02X succeeded on attempt %d\n",
                    slave, s->index, s->subindex, attempt);

        /* Read back: a drive may ACK a write and keep (or clamp) another value.
         * Only a warning — some objects are write-only or read back differently. */
        uint32_t rb = 0;
        if (do_sdo_read(slave, s->index, s->subindex, s->byte_size, &rb) == KRON_EC_OK) {
            uint32_t mask = (s->byte_size >= 4) ? 0xFFFFFFFFu : ((1u << (8u * s->byte_size)) - 1u);
            if ((rb & mask) != (s->value & mask))
                fprintf(stderr, "[kronec] Startup SDO slave %d 0x%04X:%02X wrote 0x%X, reads back 0x%X\n",
                        slave, s->index, s->subindex, (unsigned)(s->value & mask), (unsigned)(rb & mask));
        } else {
            (void)ec_take_abort(ctx);   /* drain the read abort, not a failure */
        }
    }
    return 1;
}

/* ── kron_ec_pdo_map_check ───────────────────────────────────────────────── */
/*
 * The PDO entry list comes from the project configuration (KronEditor UI) and
 * is not validated against what the slave actually maps. Without this check a
 * too-long or mistyped entry list walks past the slave's IOmap segment and
 * silently reads/writes the neighbouring slave's process data.
 *
 * Called once after ecx_config_map_group(), when Ibytes/Obytes are known.
 * Returns KRON_EC_OK, or KRON_EC_ERR_CONFIG after reporting every offender.
 */
static int kron_ec_pdo_map_check(KRON_EC_Config *cfg) {
    int rc = KRON_EC_OK;

    for (int si = 0; si < cfg->slave_count; si++) {
        KRON_EC_Slave *sl  = &cfg->slaves[si];
        uint16_t       pos = sl->position;

        if (pos < 1 || pos > (uint16_t)g_ctx.slavecount) {
            fprintf(stderr, "[kronec] Config error: slave '%s' at position %u, "
                            "but only %d slave(s) on the bus\n",
                    sl->name, pos, g_ctx.slavecount);
            rc = KRON_EC_ERR_CONFIG;
            continue;
        }

        uint32_t in_lim  = slave_in_bytes(pos);
        uint32_t out_lim = slave_out_bytes(pos);
        uint32_t in_off = 0u, out_off = 0u;

        for (int i = 0; i < sl->pdo_count; i++) {
            KRON_EC_PDO_Entry *e   = &sl->pdo_entries[i];
            uint32_t           sz  = dtype_bytes(e->dtype);
            bool               in  = (e->dir == KRON_EC_DIR_INPUT);
            uint32_t          *off = in ? &in_off : &out_off;
            uint32_t           lim = in ? in_lim  : out_lim;

            if (*off + sz > lim) {
                fprintf(stderr, "[kronec] Config error: slave %u '%s' PDO '%s' "
                                "(0x%04X:%02X) needs %s bytes %u..%u but the "
                                "slave maps only %u\n",
                        pos, sl->name, e->name ? e->name : "?",
                        e->index, e->subindex, in ? "input" : "output",
                        *off, *off + sz - 1u, lim);
                rc = KRON_EC_ERR_CONFIG;
            }
            *off += sz;
        }
    }
    return rc;
}

/* ── SOEM context lifetime ─────────────────────────────────────────────────
 *
 * ⚠️ ecx_close() must run EXACTLY ONCE per successful ecx_init(). kron_ec_init
 * closes the context on each of its own failure paths and returns an error, but
 * kron_ec_close() — called from PLC_Cleanup when the user stops the runtime —
 * closed it again unconditionally. A bus that failed to reach OP therefore ended
 * every run with "free(): double free detected in tcache 2", killing the process
 * at shutdown and making a configuration problem look like memory corruption.
 */
static bool g_ctx_open = false;

static void kron_ec_close_ctx(void) {
    if (!g_ctx_open) return;
    g_ctx_open = false;
    ecx_close(&g_ctx);
}

/* Exchange process data for a while. Used both to let the DC clocks converge
 * and to keep traffic flowing while a state transition is pending — SOEM
 * distributes the DC reference time inside send_processdata, so a bus that is
 * never pumped never synchronises. */
static void kron_ec_pump(int frames, uint32_t cycle_us) {
    struct timespec ts;
    ts.tv_sec  = 0;
    ts.tv_nsec = (long)((cycle_us ? cycle_us : 1000) * 1000UL);
    for (int i = 0; i < frames; i++) {
        ecx_send_processdata(&g_ctx);
        ecx_receive_processdata(&g_ctx, EC_TIMEOUTRET);
        nanosleep(&ts, NULL);
    }
}

/* Arm or disarm DC SYNC0 on every slave. */
static void kron_ec_sync0(KRON_EC_Config *cfg, bool on) {
    uint32 cycle_ns = (uint32)((uint64_t)(cfg->cycle_us ? cfg->cycle_us : 1000) * 1000ULL);
    for (int i = 1; i <= g_ctx.slavecount; i++) {
        ecx_dcsync0(&g_ctx, (uint16)i, on ? TRUE : FALSE, on ? cycle_ns : 0, 0);
    }
}

/* Request OPERATIONAL with process data FLOWING.
 *
 * ⚠️ A single send/receive round followed by one statecheck is not enough: a
 * slave leaves SAFE-OP only while it is receiving valid process data, so the
 * frames have to keep coming during the transition. This is the loop every
 * SOEM example uses; the old code primed the IOmap once and then just waited.
 */
static bool kron_ec_request_op(uint32_t cycle_us) {
    g_ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_send_processdata(&g_ctx);
    ecx_receive_processdata(&g_ctx, EC_TIMEOUTRET);
    ecx_writestate(&g_ctx, 0);
    for (int chk = 0; chk < KRON_EC_OP_ATTEMPTS; chk++) {
        ecx_send_processdata(&g_ctx);
        ecx_receive_processdata(&g_ctx, EC_TIMEOUTRET);
        if (ecx_statecheck(&g_ctx, 0, EC_STATE_OPERATIONAL, 50000) == EC_STATE_OPERATIONAL)
            return true;
        if (cycle_us) {
            struct timespec ts = { 0, (long)(cycle_us * 1000UL) };
            nanosleep(&ts, NULL);
        }
    }
    return false;
}

/* ── kron_ec_init ─────────────────────────────────────────────────────────── */

int kron_ec_init(KRON_EC_Config *cfg) {
    if (!cfg || cfg->ifname[0] == '\0') return KRON_EC_ERR_INIT;

    kron_ec_init_sdo_queue();

    cfg->master_state   = KRON_EC_MASTER_NONE;
    cfg->is_operational = false;
    cfg->found_slaves   = 0;

    memset(&g_ctx, 0, sizeof(g_ctx));

    if (ecx_init(&g_ctx, cfg->ifname) <= 0) {
        fprintf(stderr, "[kronec] ecx_init('%s') failed\n", cfg->ifname);
        cfg->master_state = KRON_EC_MASTER_ERROR;
        return KRON_EC_ERR_INIT;
    }
    g_ctx_open = true;
    cfg->master_state = KRON_EC_MASTER_INIT;

    int found = ecx_config_init(&g_ctx);
    if (found <= 0) {
        fprintf(stderr, "[kronec] No EtherCAT slaves found on %s\n", cfg->ifname);
        kron_ec_close_ctx();
        cfg->master_state = KRON_EC_MASTER_ERROR;
        return KRON_EC_ERR_NO_SLAVES;
    }
    cfg->found_slaves = found;
    fprintf(stderr, "[kronec] Found %d slave(s)\n", found);
    cfg->master_state = KRON_EC_MASTER_PREOP;

    /* Register PO2SO hook on all slaves so SDO inits run during PREOP→SAFEOP
     * transition — exactly when SOEM expects PDO remapping SDOs to be applied. */
    g_cfg_ptr = cfg;
    for (int i = 1; i <= g_ctx.slavecount; i++) {
        g_ctx.slavelist[i].PO2SOconfig = kron_po2so_hook;
    }

    /* ⚠️ manualstatechange: left at 0, ecx_config_map_group() requests SAFE-OP
     * for every slave itself as soon as the mapping is done — i.e. BEFORE the
     * checks below could look at the startup SDOs. The slaves stay in PRE-OP
     * until those checks pass and SAFE-OP is requested explicitly. */
    g_sdo_init_failed = 0;
    g_ctx.manualstatechange = 1;
    ecx_config_map_group(&g_ctx, g_IOmap, 0);
    g_ctx.manualstatechange = 0;

    /* A startup SDO the drive never accepted (after retries, see
     * kron_po2so_hook) means the bus would run with the wrong mode / mapping /
     * parameter while looking healthy. The slaves are left in PRE-OP. */
    if (g_sdo_init_failed > 0) {
        fprintf(stderr, "[kronec] %d slave(s) refused an ABORT startup SDO — bus stays in PRE-OP:\n",
                g_sdo_init_failed);
        for (int si = 0; si < cfg->slave_count; si++) {
            KRON_EC_Slave *sl = &cfg->slaves[si];
            if (sl->position >= 1 && sl->position <= (uint16_t)g_ctx.slavecount)
                sl->current_state = (uint8_t)ecx_statecheck(&g_ctx, sl->position, EC_STATE_PRE_OP, EC_TIMEOUTRET);
            sl->link_up = false;
            if (!sl->sdo_aborted) continue;
            fprintf(stderr, "[kronec]   slave %u '%s': 0x%04X:%02X abort 0x%08X %s\n",
                    sl->position, sl->name, sl->sdo_err_index, sl->sdo_err_subindex,
                    (unsigned)sl->sdo_abort_code, ec_abort_text(sl->sdo_abort_code));
        }
        kron_ec_close_ctx();
        cfg->master_state = KRON_EC_MASTER_ERROR;
        return KRON_EC_ERR_CONFIG;
    }

    /* Refuse to run with a PDO list that does not fit the slaves' IOmap
     * segments — otherwise pdo_read/pdo_write would cross into a neighbour. */
    if (kron_ec_pdo_map_check(cfg) != KRON_EC_OK) {
        kron_ec_close_ctx();
        cfg->master_state = KRON_EC_MASTER_ERROR;
        return KRON_EC_ERR_CONFIG;
    }

    /* Both checks passed: now PRE-OP → SAFE-OP (what map_group used to do). */
    g_ctx.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&g_ctx, 0);

    /* Distributed clocks.
     *
     * ⚠️ ORDER MATTERS AND USED TO BE WRONG. ecx_configdc() only measures
     * propagation delays and picks the reference clock; the reference TIME is
     * distributed by the FRMW datagram that rides inside every
     * ecx_send_processdata(). Arming SYNC0 immediately after configdc — before
     * a single frame has circulated — leaves every slave with an unsynchronised
     * clock, and a conformant drive then refuses OPERATIONAL and parks in
     * SAFE-OP reporting ALstatuscode 0x0000, i.e. with nothing to explain it.
     * Measured on real hardware: DC off → OP, DC on → SAFE-OP, regardless of
     * how OP was requested. So: configure, PUMP until the clocks converge, and
     * only then arm SYNC0. */
    if (cfg->dc_enable) {
        ecx_configdc(&g_ctx);
        kron_ec_pump(KRON_EC_DC_SETTLE_FRAMES, cfg->cycle_us);
        kron_ec_sync0(cfg, true);
        fprintf(stderr, "[kronec] DC: clocks settled over %d frames, SYNC0 armed (cycle: %u us)\n",
                KRON_EC_DC_SETTLE_FRAMES, cfg->cycle_us);
    }

    /* Wait for SAFE-OP */
    ecx_statecheck(&g_ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    cfg->master_state = KRON_EC_MASTER_SAFEOP;

    bool op = kron_ec_request_op(cfg->cycle_us);

    /* ⚠️ Fallback, and a deliberate one: both orders are legal and drives
     * disagree about which they accept. Some refuse to leave SAFE-OP while
     * SYNC0 is already firing (they want to be in OP before they are
     * synchronised), others require SYNC0 first. Trying the standard order and
     * then the other one makes the master work across drives instead of across
     * one drive. */
    if (!op && cfg->dc_enable) {
        fprintf(stderr, "[kronec] OP refused with SYNC0 armed — retrying with SYNC0 armed AFTER OP\n");
        kron_ec_sync0(cfg, false);
        kron_ec_pump(64, cfg->cycle_us);
        op = kron_ec_request_op(cfg->cycle_us);
        if (op) {
            kron_ec_sync0(cfg, true);
            kron_ec_pump(64, cfg->cycle_us);
            /* Arming SYNC0 can knock a slave back out of OP; make sure it stuck. */
            if (ecx_statecheck(&g_ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL) {
                fprintf(stderr, "[kronec] Slave left OP when SYNC0 was armed — continuing without DC SYNC0\n");
                kron_ec_sync0(cfg, false);
                op = kron_ec_request_op(cfg->cycle_us);
            }
        }
    }

    if (!op) {
        fprintf(stderr, "[kronec] Could not reach OP state (bus is 0x%02X)\n",
                g_ctx.slavelist[0].state);
        for (int i = 1; i <= g_ctx.slavecount; i++) {
            fprintf(stderr, "[kronec]   slave %d state=0x%02X ALstatuscode=0x%04X %s\n",
                    i, g_ctx.slavelist[i].state, g_ctx.slavelist[i].ALstatuscode,
                    ec_ALstatuscode2string(g_ctx.slavelist[i].ALstatuscode));
        }
        kron_ec_close_ctx();
        cfg->master_state = KRON_EC_MASTER_ERROR;
        return KRON_EC_ERR_OP;
    }
    cfg->master_state   = KRON_EC_MASTER_OP;
    cfg->is_operational = true;

    /* Update per-slave runtime state */
    for (int si = 0; si < cfg->slave_count; si++) {
        KRON_EC_Slave *sl = &cfg->slaves[si];
        uint16_t pos = sl->position;
        if (pos >= 1 && pos <= (uint16_t)g_ctx.slavecount) {
            sl->current_state = (uint8_t)g_ctx.slavelist[pos].state;
            sl->link_up       = (sl->current_state == EC_STATE_OPERATIONAL);
        }
    }

    fprintf(stderr, "[kronec] Bus running on %s, %d slave(s) in OP\n",
            cfg->ifname, g_ctx.slavecount);
    fprintf(stderr, "[kronec] Cycle time: %u us\n", cfg->cycle_us);
    return KRON_EC_OK;
}

/* ── kron_ec_pdo_read ─────────────────────────────────────────────────────── */
/*
 * Pillar 2 — called at the very START of each RT cycle, before PLC logic.
 * Sequence: send_processdata → receive_processdata → copy inputs to PLC vars.
 */
void kron_ec_pdo_read(KRON_EC_Config *cfg) {
    if (!cfg || !cfg->is_operational) return;

    ctx_lock();
    ecx_send_processdata(&g_ctx);
    ecx_receive_processdata(&g_ctx, EC_TIMEOUTRET);
    ctx_unlock();

    /* Copy TxPDO (inputs) from IOmap → PLC variable pointers */
    for (int si = 0; si < cfg->slave_count; si++) {
        KRON_EC_Slave *sl = &cfg->slaves[si];
        uint16_t pos = sl->position;
        if (pos < 1 || pos > (uint16_t)g_ctx.slavecount) continue;
        uint8_t *inputs = (uint8_t *)g_ctx.slavelist[pos].inputs;
        if (!inputs) continue;

        /* kron_ec_pdo_map_check() rejects an over-long map at init; this is the
         * belt-and-braces guard so a bad offset can never reach the neighbour's
         * IOmap segment on the RT path. */
        uint32_t lim = slave_in_bytes(pos);

        uint32_t byte_off = 0u;
        for (int i = 0; i < sl->pdo_count; i++) {
            KRON_EC_PDO_Entry *e = &sl->pdo_entries[i];
            if (e->dir != KRON_EC_DIR_INPUT) continue;
            uint32_t sz = dtype_bytes(e->dtype);
            if (byte_off + sz > lim) break;
            if (e->var_ptr) memcpy(e->var_ptr, inputs + byte_off, sz);
            byte_off += sz;
        }
    }
}

/* ── kron_ec_pdo_write ────────────────────────────────────────────────────── */
/*
 * Pillar 2 — called at the very END of each RT cycle, after PLC logic.
 * Copies PLC variable values into the IOmap output area.
 * The actual Ethernet frame is sent at the start of the NEXT cycle by pdo_read.
 */
void kron_ec_pdo_write(KRON_EC_Config *cfg) {
    if (!cfg || !cfg->is_operational) return;

    /* Copy PLC variable pointers → RxPDO (outputs) in IOmap */
    for (int si = 0; si < cfg->slave_count; si++) {
        KRON_EC_Slave *sl = &cfg->slaves[si];
        uint16_t pos = sl->position;
        if (pos < 1 || pos > (uint16_t)g_ctx.slavecount) continue;
        uint8_t *outputs = (uint8_t *)g_ctx.slavelist[pos].outputs;
        if (!outputs) continue;

        /* See kron_ec_pdo_read — never write past this slave's segment. */
        uint32_t lim = slave_out_bytes(pos);

        uint32_t byte_off = 0u;
        for (int i = 0; i < sl->pdo_count; i++) {
            KRON_EC_PDO_Entry *e = &sl->pdo_entries[i];
            if (e->dir != KRON_EC_DIR_OUTPUT) continue;
            uint32_t sz = dtype_bytes(e->dtype);
            if (byte_off + sz > lim) break;
            if (e->var_ptr) memcpy(outputs + byte_off, e->var_ptr, sz);
            byte_off += sz;
        }
    }
}

/* ── kron_ec_check_state ──────────────────────────────────────────────────── */
/*
 * Pillar 1 — called periodically (e.g. every 100ms from a watchdog thread)
 * to detect lost slaves and attempt recovery back to OP state.
 */
void kron_ec_check_state(KRON_EC_Config *cfg) {
    if (!cfg) return;

    ctx_lock();
    for (int i = 1; i <= g_ctx.slavecount; i++) {
        uint16_t actual = ecx_statecheck(&g_ctx, i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);

        if (actual != EC_STATE_OPERATIONAL) {
            fprintf(stderr, "[kronec] Slave %d not OP (state=0x%02X), recovering\n",
                    i, actual);

            if (ecx_recover_slave(&g_ctx, i, EC_TIMEOUTSAFE)) {
                g_sdo_init_failed = 0;
                ecx_reconfig_slave(&g_ctx, i, EC_TIMEOUTSAFE);
                g_ctx.slavelist[i].islost = FALSE;
                /* ⚠️ ecx_reconfig_slave() requests SAFE-OP right after the hook
                 * whatever the hook reported — pull the slave back to PRE-OP and
                 * leave it there rather than run it without its startup SDOs. */
                if (g_sdo_init_failed > 0) {
                    fprintf(stderr, "[kronec] Slave %d: %d startup SDO(s) failed on recovery — holding in PRE-OP\n",
                            i, g_sdo_init_failed);
                    g_ctx.slavelist[i].state = EC_STATE_PRE_OP;
                    ecx_writestate(&g_ctx, i);
                    g_ctx.slavelist[i].state = ecx_statecheck(&g_ctx, i, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
                    continue;
                }
                ecx_statecheck(&g_ctx, i, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
            }

            g_ctx.slavelist[i].state = EC_STATE_OPERATIONAL;
            ecx_writestate(&g_ctx, i);
            actual = ecx_statecheck(&g_ctx, i, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
            g_ctx.slavelist[i].state = actual;

            if (actual == EC_STATE_OPERATIONAL)
                fprintf(stderr, "[kronec] Slave %d recovered to OP\n", i);
            else
                fprintf(stderr, "[kronec] Slave %d recovery failed (state=0x%02X)\n", i, actual);
        }
    }
    ctx_unlock();

    /* Count active slaves and update per-slave status */
    int active  = 0;
    bool all_op = true;
    for (int si = 0; si < cfg->slave_count; si++) {
        KRON_EC_Slave *sl  = &cfg->slaves[si];
        uint16_t       pos = sl->position;
        if (pos >= 1 && pos <= (uint16_t)g_ctx.slavecount) {
            sl->current_state = (uint8_t)g_ctx.slavelist[pos].state;
            sl->link_up       = (sl->current_state == EC_STATE_OPERATIONAL);
            if (sl->link_up) active++; else all_op = false;
        } else {
            sl->current_state = 0;
            sl->link_up       = false;
            all_op            = false;
        }
    }
    cfg->found_slaves   = active;
    cfg->is_operational = all_op;
    cfg->master_state   = all_op ? KRON_EC_MASTER_OP : KRON_EC_MASTER_ERROR;
}

/* ── kron_ec_process_sdo ──────────────────────────────────────────────────── */
/*
 * Called from the SDO background thread (NOT the RT cycle thread).
 * Processes one pending request from the async SDO queue.
 * This keeps SDO traffic completely off the real-time PDO path.
 */
void kron_ec_process_sdo(KRON_EC_Config *cfg) {
    /* A pending EC_ResetBus request is serviced here, on this thread, because
     * kron_ec_init() blocks for seconds and must never run in the PLC scan. */
    pthread_mutex_lock(&g_reset_mutex);
    bool do_reset = (g_reset_state == KRON_EC_RESET_REQ);
    if (do_reset) g_reset_state = KRON_EC_RESET_RUNNING;
    pthread_mutex_unlock(&g_reset_mutex);

    if (do_reset) {
        int rc = kron_ec_init(cfg);

        pthread_mutex_lock(&g_reset_mutex);
        g_reset_rc    = rc;
        g_reset_state = (rc == KRON_EC_OK && cfg && cfg->is_operational)
                            ? KRON_EC_RESET_DONE_OK : KRON_EC_RESET_DONE_ERR;
        pthread_mutex_unlock(&g_reset_mutex);
        return;  /* the bus was just reinitialised — SDOs wait for next tick */
    }

    for (int i = 0; i < KRON_EC_MAX_SDO_QUEUE_SIZE; i++) {
        int st = sdo_queue[i].state;

        if (st == KRON_EC_SDO_WRITE_REQ) {
            int r = do_sdo_write(sdo_queue[i].slave_pos,
                                 sdo_queue[i].index,
                                 sdo_queue[i].subindex,
                                 sdo_queue[i].byte_size,
                                 sdo_queue[i].value);
            sdo_queue[i].state = (r == KRON_EC_OK) ? KRON_EC_SDO_DONE_OK
                                                   : KRON_EC_SDO_DONE_ERR;
            break;
        }

        if (st == KRON_EC_SDO_READ_REQ) {
            uint32_t val = 0;
            int r = do_sdo_read(sdo_queue[i].slave_pos,
                                sdo_queue[i].index,
                                sdo_queue[i].subindex,
                                sdo_queue[i].byte_size,
                                &val);
            if (r == KRON_EC_OK) {
                sdo_queue[i].value = val;
                sdo_queue[i].state = KRON_EC_SDO_DONE_OK;
            } else {
                sdo_queue[i].state = KRON_EC_SDO_DONE_ERR;
            }
            break;
        }
    }
}

/* ── kron_ec_close ────────────────────────────────────────────────────────── */

void kron_ec_close(KRON_EC_Config *cfg) {
    if (cfg) { cfg->is_operational = false; cfg->master_state = KRON_EC_MASTER_NONE; }
    /* ⚠️ Nothing to close when init already closed it on a failure path —
     * writing state to a closed port and calling ecx_close() twice is the
     * double free described at kron_ec_close_ctx(). */
    if (!g_ctx_open) return;
    g_ctx.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&g_ctx, 0);
    kron_ec_close_ctx();
    fprintf(stderr, "[kronec] EtherCAT master closed\n");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * USER-FACING FUNCTION BLOCK IMPLEMENTATIONS
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── EC_GetMasterState ────────────────────────────────────────────────────── */
void EC_GetMasterState_Call(EC_GetMasterState *inst, KRON_EC_Config *cfg) {
    inst->Valid       = false;
    inst->Error       = false;
    inst->ErrorID     = 0;
    inst->State       = 0;
    inst->Operational = false;
    inst->SlaveCount  = 0;

    if (inst->Enable && cfg) {
        inst->State       = (uint8_t)cfg->master_state;
        inst->Operational = cfg->is_operational;
        inst->SlaveCount  = (uint16_t)cfg->found_slaves;
        inst->Valid       = true;
    }
    inst->_prevEnable = inst->Enable;
}

/* ── EC_GetSlaveState ─────────────────────────────────────────────────────── */
void EC_GetSlaveState_Call(EC_GetSlaveState *inst, KRON_EC_Config *cfg) {
    inst->Valid   = false;
    inst->Error   = false;
    inst->ErrorID = 0;
    inst->State   = 0;
    inst->LinkUp  = false;
    inst->SdoIndex = 0; inst->SdoSubIndex = 0; inst->SdoAbortCode = 0;

    if (!inst->Enable || !cfg) { inst->_prevEnable = inst->Enable; return; }

    /* Look up matching configured slave */
    for (int si = 0; si < cfg->slave_count; si++) {
        if (cfg->slaves[si].position == inst->SlaveAddress) {
            KRON_EC_Slave *sl = &cfg->slaves[si];
            inst->State        = sl->current_state;
            inst->LinkUp       = sl->link_up;
            inst->SdoIndex     = sl->sdo_err_index;
            inst->SdoSubIndex  = sl->sdo_err_subindex;
            inst->SdoAbortCode = sl->sdo_abort_code;
            if (sl->sdo_aborted) {
                inst->Error   = true;
                inst->ErrorID = 2; /* ERR_STARTUP_SDO: an ABORT startup SDO was refused */
            }
            inst->Valid  = true;
            inst->_prevEnable = inst->Enable;
            return;
        }
    }
    /* Slave not in configured list */
    inst->Error   = true;
    inst->ErrorID = 1; /* ERR_INVALID_SLAVE: address not in configured list */
    inst->_prevEnable = inst->Enable;
}

/* ── EC_ResetBus ──────────────────────────────────────────────────────────── */
/*
 * Full re-initialization via kron_ec_init().
 *
 * Incremental recovery (ecx_recover_slave + ecx_reconfig_slave) is only
 * sufficient when a slave briefly glitches.  After a deliberate power
 * cycle the slave boots from scratch (INIT state, no IOmap), so the
 * master must also rebuild its context from scratch.
 *
 * Non-blocking: kron_ec_init() takes seconds (bus scan + statecheck timeouts),
 * so this block only posts a request on the rising edge of Execute and polls
 * it each scan; the background service thread (kron_ec_process_sdo) performs
 * the reinit. Re-triggering while Busy is rejected with ErrorID 6.
 *
 * kron_ec_init() sets cfg->is_operational = false before touching SOEM,
 * which causes the IO_Bus thread to skip kron_ec_pdo_read/write during
 * the brief reinit window — no mutex needed.
 *
 * ErrorID mapping on failure (see error_codes.xml):
 *   1 — null cfg pointer
 *   2 — ecx_init failed (NIC/driver error)
 *   3 — no slaves found on bus
 *   4 — PDO/IOmap config error (includes a PDO map longer than the slave's
 *       IOmap segment — see kron_ec_pdo_map_check)
 *   5 — could not reach OP state
 *   6 — busy: a reset is already in progress
 */
void EC_ResetBus_Call(EC_ResetBus *inst, KRON_EC_Config *cfg) {
    bool rising = inst->Execute && !inst->_prevExecute;

    if (!inst->Execute && inst->_prevExecute) { inst->Done = false; }

    if (rising && inst->Busy) {
        /* A reset is already running — reject the new trigger and leave the
         * request in flight (same rule as the SDO blocks). */
        inst->Error   = true;
        inst->ErrorID = 6; /* ERR_BUSY */
        rising = false;
    }

    if (rising) {
        inst->Done    = false;
        inst->Busy    = true;
        inst->Error   = false;
        inst->ErrorID = 0;

        if (!cfg) {
            inst->Error   = true;
            inst->ErrorID = 1; /* ERR_NULL_CFG */
            inst->Busy    = false;
        } else {
            /* Post the request; the service thread runs the (blocking) init. */
            pthread_mutex_lock(&g_reset_mutex);
            if (g_reset_state == KRON_EC_RESET_IDLE) {
                g_reset_rc    = KRON_EC_OK;
                g_reset_state = KRON_EC_RESET_REQ;
            } else {
                inst->Error   = true;
                inst->ErrorID = 6; /* ERR_BUSY — another block owns the reset */
                inst->Busy    = false;
            }
            pthread_mutex_unlock(&g_reset_mutex);
        }
    }

    /* Poll the request state — one scan cycle each, never blocking. */
    if (inst->Busy) {
        pthread_mutex_lock(&g_reset_mutex);
        int st = g_reset_state;
        int rc = g_reset_rc;
        if (st == KRON_EC_RESET_DONE_OK || st == KRON_EC_RESET_DONE_ERR)
            g_reset_state = KRON_EC_RESET_IDLE;
        pthread_mutex_unlock(&g_reset_mutex);

        if (st == KRON_EC_RESET_DONE_OK) {
            inst->Done = true;
            inst->Busy = false;
        } else if (st == KRON_EC_RESET_DONE_ERR) {
            inst->Busy  = false;
            inst->Error = true;
            switch (rc) {
                case KRON_EC_ERR_INIT:       inst->ErrorID = 2; break; /* ERR_INIT */
                case KRON_EC_ERR_NO_SLAVES:  inst->ErrorID = 3; break; /* ERR_NO_SLAVES */
                case KRON_EC_ERR_CONFIG:     inst->ErrorID = 4; break; /* ERR_CONFIG */
                case KRON_EC_ERR_OP:         inst->ErrorID = 5; break; /* ERR_OP */
                default:                     inst->ErrorID = 5; break; /* ERR_OP (generic) */
            }
        }
    }

    inst->_prevExecute = inst->Execute;
}

/* ── EC_ReadSDO ───────────────────────────────────────────────────────────── */
/*
 * Non-blocking: posts a request to the global SDO queue on rising edge of
 * Execute, then polls state each cycle. Done/Error/Value set when the
 * SDO background thread completes the transfer.
 *
 * ErrorID: 1 — queue full, 2 — SDO transfer failed, 3 — busy (re-triggered
 * while the previous request was still in flight; the new trigger is ignored).
 */
void EC_ReadSDO_Call(EC_ReadSDO *inst, KRON_EC_Config *cfg) {
    (void)cfg;
    bool rising = inst->Execute && !inst->_prevExecute;

    if (!inst->Execute && inst->_prevExecute) {
        inst->Done  = false;
        inst->Value = 0;
    }

    if (rising && inst->Busy) {
        /* The previous request is still in flight. Overwriting _queue_id here
         * would orphan its queue slot: the service thread parks it in DONE_OK /
         * DONE_ERR and nobody ever returns it to IDLE, so 16 such re-triggers
         * exhaust the queue permanently. Reject the trigger instead. */
        inst->Error   = true;
        inst->ErrorID = 3; /* ERR_BUSY */
        rising = false;
    }

    if (rising) {
        inst->Done    = false;
        inst->Busy    = true;
        inst->Error   = false;
        inst->ErrorID = 0;
        inst->Value   = 0;
        inst->_queue_id = -1;

        pthread_mutex_lock(&sdo_queue_mutex);
        for (int i = 0; i < KRON_EC_MAX_SDO_QUEUE_SIZE; i++) {
            if (sdo_queue[i].state != KRON_EC_SDO_IDLE) continue;

            sdo_queue[i].slave_pos = inst->SlaveAddress;
            sdo_queue[i].index     = inst->Index;
            sdo_queue[i].subindex  = inst->SubIndex;
            sdo_queue[i].byte_size = inst->ByteSize ? inst->ByteSize : 4;
            sdo_queue[i].value     = 0;
            sdo_queue[i].state     = KRON_EC_SDO_READ_REQ;
            inst->_queue_id        = i;
            break;
        }
        pthread_mutex_unlock(&sdo_queue_mutex);

        if (inst->_queue_id < 0) {
            inst->Error   = true;
            inst->ErrorID = 1; /* ERR_QUEUE_FULL */
            inst->Busy    = false;
        }
    }

    /* Poll queue state */
    if (inst->Busy && inst->_queue_id >= 0 &&
        inst->_queue_id < KRON_EC_MAX_SDO_QUEUE_SIZE) {
        KRON_EC_SDO_Request *req = &sdo_queue[inst->_queue_id];
        int st = req->state;

        if (st == KRON_EC_SDO_DONE_OK) {
            inst->Value      = req->value;
            inst->Done       = true;
            inst->Busy       = false;
            req->state       = KRON_EC_SDO_IDLE;
            inst->_queue_id  = -1;
        } else if (st == KRON_EC_SDO_DONE_ERR) {
            inst->Error      = true;
            inst->ErrorID    = 2; /* ERR_SDO_FAILED */
            inst->Busy       = false;
            req->state       = KRON_EC_SDO_IDLE;
            inst->_queue_id  = -1;
        }
    }

    inst->_prevExecute = inst->Execute;
}

/* ── EC_WriteSDO ──────────────────────────────────────────────────────────── */
void EC_WriteSDO_Call(EC_WriteSDO *inst, KRON_EC_Config *cfg) {
    (void)cfg;
    bool rising = inst->Execute && !inst->_prevExecute;

    if (!inst->Execute && inst->_prevExecute) {
        inst->Done = false;
    }

    if (rising && inst->Busy) {
        /* The previous request is still in flight. Overwriting _queue_id here
         * would orphan its queue slot: the service thread parks it in DONE_OK /
         * DONE_ERR and nobody ever returns it to IDLE, so 16 such re-triggers
         * exhaust the queue permanently. Reject the trigger instead. */
        inst->Error   = true;
        inst->ErrorID = 3; /* ERR_BUSY */
        rising = false;
    }

    if (rising) {
        inst->Done    = false;
        inst->Busy    = true;
        inst->Error   = false;
        inst->ErrorID = 0;
        inst->_queue_id = -1;

        pthread_mutex_lock(&sdo_queue_mutex);
        for (int i = 0; i < KRON_EC_MAX_SDO_QUEUE_SIZE; i++) {
            if (sdo_queue[i].state != KRON_EC_SDO_IDLE) continue;

            sdo_queue[i].slave_pos = inst->SlaveAddress;
            sdo_queue[i].index     = inst->Index;
            sdo_queue[i].subindex  = inst->SubIndex;
            sdo_queue[i].byte_size = inst->ByteSize ? inst->ByteSize : 4;
            sdo_queue[i].value     = inst->Value;
            sdo_queue[i].state     = KRON_EC_SDO_WRITE_REQ;
            inst->_queue_id        = i;
            break;
        }
        pthread_mutex_unlock(&sdo_queue_mutex);

        if (inst->_queue_id < 0) {
            inst->Error   = true;
            inst->ErrorID = 1; /* ERR_QUEUE_FULL */
            inst->Busy    = false;
        }
    }

    /* Poll queue state */
    if (inst->Busy && inst->_queue_id >= 0 &&
        inst->_queue_id < KRON_EC_MAX_SDO_QUEUE_SIZE) {
        KRON_EC_SDO_Request *req = &sdo_queue[inst->_queue_id];
        int st = req->state;

        if (st == KRON_EC_SDO_DONE_OK) {
            inst->Done      = true;
            inst->Busy      = false;
            req->state      = KRON_EC_SDO_IDLE;
            inst->_queue_id = -1;
        } else if (st == KRON_EC_SDO_DONE_ERR) {
            inst->Error     = true;
            inst->ErrorID   = 2; /* ERR_SDO_FAILED */
            inst->Busy      = false;
            req->state      = KRON_EC_SDO_IDLE;
            inst->_queue_id = -1;
        }
    }

    inst->_prevExecute = inst->Execute;
}

#endif /* !KRON_EC_SIM */
