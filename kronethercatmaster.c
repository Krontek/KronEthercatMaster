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
/* SOEM is NOT thread-safe: ecx_SDOread/write and ecx_send/receive_processdata
 * share the same socket. This mutex serializes SDO and PDO access. */
static pthread_mutex_t g_ctx_mutex = PTHREAD_MUTEX_INITIALIZER;

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
    pthread_mutex_lock(&g_ctx_mutex);
    int wkc = ecx_SDOwrite(&g_ctx, slave, idx, sub, FALSE,
                           bsz, &val, EC_TIMEOUTRXM);
    pthread_mutex_unlock(&g_ctx_mutex);
    return (wkc > 0) ? KRON_EC_OK : KRON_EC_ERR_IO;
}

static int do_sdo_read(uint16_t slave, uint16_t idx, uint8_t sub,
                       uint8_t bsz, uint32_t *out) {
    int sz = (int)bsz;
    uint32_t buf = 0;
    pthread_mutex_lock(&g_ctx_mutex);
    int wkc = ecx_SDOread(&g_ctx, slave, idx, sub, FALSE, &sz, &buf, EC_TIMEOUTRXM);
    pthread_mutex_unlock(&g_ctx_mutex);
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

static int kron_po2so_hook(ecx_contextt *ctx, uint16 slave) {
    if (!g_cfg_ptr) return 1;
    for (int si = 0; si < g_cfg_ptr->slave_count; si++) {
        KRON_EC_Slave *sl = &g_cfg_ptr->slaves[si];
        if (sl->position != slave) continue;
        for (int i = 0; i < sl->sdo_count; i++) {
            KRON_EC_SDO *s = &sl->sdo_inits[i];
            if (do_sdo_write(slave, s->index, s->subindex, s->byte_size, s->value) != KRON_EC_OK)
                fprintf(stderr, "[kronec] PO2SO SDO failed: slave %d 0x%04X:%02X\n",
                        slave, s->index, s->subindex);
        }
        break;
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
    cfg->master_state = KRON_EC_MASTER_INIT;

    int found = ecx_config_init(&g_ctx);
    if (found <= 0) {
        fprintf(stderr, "[kronec] No EtherCAT slaves found on %s\n", cfg->ifname);
        ecx_close(&g_ctx);
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

    ecx_config_map_group(&g_ctx, g_IOmap, 0);

    /* Refuse to run with a PDO list that does not fit the slaves' IOmap
     * segments — otherwise pdo_read/pdo_write would cross into a neighbour. */
    if (kron_ec_pdo_map_check(cfg) != KRON_EC_OK) {
        ecx_close(&g_ctx);
        cfg->master_state = KRON_EC_MASTER_ERROR;
        return KRON_EC_ERR_CONFIG;
    }

    /* Distributed clocks */
    if (cfg->dc_enable) {
        ecx_configdc(&g_ctx);
        /* Activate DC SYNC0 on each configured slave — must happen AFTER
         * ecx_configdc() so the DC system is initialised.  Required for
         * CSP / CSV / CST drive modes. */
        if (cfg->cycle_us > 0) {
            uint32 cycle_ns = (uint32)((uint64_t)cfg->cycle_us * 1000ULL);
            for (int i = 1; i <= g_ctx.slavecount; i++) {
                ecx_dcsync0(&g_ctx, (uint16)i, TRUE, cycle_ns, 0);
                fprintf(stderr, "[kronec] Slave %d: DC SYNC0 activated (cycle: %u us)\n",
                        i, cfg->cycle_us);
            }
        }
    }

    /* Wait for SAFE-OP */
    ecx_statecheck(&g_ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    cfg->master_state = KRON_EC_MASTER_SAFEOP;

    /* One cycle to prime the IOmap */
    ecx_send_processdata(&g_ctx);
    ecx_receive_processdata(&g_ctx, EC_TIMEOUTRET);

    /* Request OP */
    g_ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&g_ctx, 0);
    ecx_statecheck(&g_ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

    if (g_ctx.slavelist[0].state != EC_STATE_OPERATIONAL) {
        fprintf(stderr, "[kronec] Could not reach OP state\n");
        ecx_close(&g_ctx);
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

    pthread_mutex_lock(&g_ctx_mutex);
    ecx_send_processdata(&g_ctx);
    ecx_receive_processdata(&g_ctx, EC_TIMEOUTRET);
    pthread_mutex_unlock(&g_ctx_mutex);

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

    pthread_mutex_lock(&g_ctx_mutex);
    for (int i = 1; i <= g_ctx.slavecount; i++) {
        uint16_t actual = ecx_statecheck(&g_ctx, i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);

        if (actual != EC_STATE_OPERATIONAL) {
            fprintf(stderr, "[kronec] Slave %d not OP (state=0x%02X), recovering\n",
                    i, actual);

            if (ecx_recover_slave(&g_ctx, i, EC_TIMEOUTSAFE)) {
                ecx_reconfig_slave(&g_ctx, i, EC_TIMEOUTSAFE);
                g_ctx.slavelist[i].islost = FALSE;
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
    pthread_mutex_unlock(&g_ctx_mutex);

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
    g_ctx.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&g_ctx, 0);
    ecx_close(&g_ctx);
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

    if (!inst->Enable || !cfg) { inst->_prevEnable = inst->Enable; return; }

    /* Look up matching configured slave */
    for (int si = 0; si < cfg->slave_count; si++) {
        if (cfg->slaves[si].position == inst->SlaveAddress) {
            inst->State  = cfg->slaves[si].current_state;
            inst->LinkUp = cfg->slaves[si].link_up;
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
