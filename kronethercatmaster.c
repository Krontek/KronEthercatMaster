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

/* ── Global SDO queue (single-slot, lock-free via volatile state) ─────────── */
KRON_EC_SDO_Queue kron_ec_sdo_queue = { KRON_EC_SDO_IDLE, 0, 0, 0, 0, 0 };

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

/* ── PO2SO hook — called by SOEM for each slave during PREOP→SAFEOP ─────── */

static int kron_po2so_hook(uint16 slave) {
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

/* ── kron_ec_init ─────────────────────────────────────────────────────────── */

int kron_ec_init(KRON_EC_Config *cfg) {
    if (!cfg || cfg->ifname[0] == '\0') return KRON_EC_ERR_INIT;

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

    /* Distributed clocks */
    if (cfg->dc_enable) {
        ecx_configdc(&g_ctx);
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

        int byte_off = 0;
        for (int i = 0; i < sl->pdo_count; i++) {
            KRON_EC_PDO_Entry *e = &sl->pdo_entries[i];
            if (e->dir != KRON_EC_DIR_INPUT || !e->var_ptr) {
                if (e->dir == KRON_EC_DIR_INPUT)
                    byte_off += dtype_bytes(e->dtype);
                continue;
            }
            uint8_t sz = dtype_bytes(e->dtype);
            memcpy(e->var_ptr, inputs + byte_off, sz);
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

        int byte_off = 0;
        for (int i = 0; i < sl->pdo_count; i++) {
            KRON_EC_PDO_Entry *e = &sl->pdo_entries[i];
            if (e->dir != KRON_EC_DIR_OUTPUT || !e->var_ptr) {
                if (e->dir == KRON_EC_DIR_OUTPUT)
                    byte_off += dtype_bytes(e->dtype);
                continue;
            }
            uint8_t sz = dtype_bytes(e->dtype);
            memcpy(outputs + byte_off, e->var_ptr, sz);
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
 * Processes one pending request from the single-slot async SDO queue.
 * This keeps SDO traffic completely off the real-time PDO path.
 */
void kron_ec_process_sdo(KRON_EC_Config *cfg) {
    (void)cfg;
    int st = kron_ec_sdo_queue.state;

    if (st == KRON_EC_SDO_WRITE_REQ) {
        int r = do_sdo_write(kron_ec_sdo_queue.slave_pos,
                             kron_ec_sdo_queue.index,
                             kron_ec_sdo_queue.subindex,
                             kron_ec_sdo_queue.byte_size,
                             kron_ec_sdo_queue.value);
        kron_ec_sdo_queue.state = (r == KRON_EC_OK) ? KRON_EC_SDO_DONE_OK
                                                     : KRON_EC_SDO_DONE_ERR;
    } else if (st == KRON_EC_SDO_READ_REQ) {
        uint32_t val = 0;
        int r = do_sdo_read(kron_ec_sdo_queue.slave_pos,
                            kron_ec_sdo_queue.index,
                            kron_ec_sdo_queue.subindex,
                            kron_ec_sdo_queue.byte_size,
                            &val);
        if (r == KRON_EC_OK) {
            kron_ec_sdo_queue.value = val;
            kron_ec_sdo_queue.state = KRON_EC_SDO_DONE_OK;
        } else {
            kron_ec_sdo_queue.state = KRON_EC_SDO_DONE_ERR;
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
    inst->ErrorID = 0x8010; /* Unknown slave address */
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
 * kron_ec_init() sets cfg->is_operational = false before touching SOEM,
 * which causes the IO_Bus thread to skip kron_ec_pdo_read/write during
 * the brief reinit window — no mutex needed.
 *
 * ErrorID mapping on failure:
 *   0x8001 — null cfg pointer
 *   0x8011 — ecx_init failed (NIC/driver error)
 *   0x8012 — no slaves found on bus
 *   0x8013 — PDO/IOmap config error
 *   0x8014 — could not reach OP state
 *   0x8002 — init returned OK but bus still not fully operational
 */
void EC_ResetBus_Call(EC_ResetBus *inst, KRON_EC_Config *cfg) {
    bool rising = inst->Execute && !inst->_prevExecute;

    if (!inst->Execute && inst->_prevExecute) { inst->Done = false; }

    if (rising) {
        inst->Done    = false;
        inst->Busy    = true;
        inst->Error   = false;
        inst->ErrorID = 0;

        if (!cfg) {
            inst->Error   = true;
            inst->ErrorID = 0x8001;
            inst->Busy    = false;
        } else {
            int rc = kron_ec_init(cfg);

            inst->Busy = false;
            if (rc == KRON_EC_OK && cfg->is_operational) {
                inst->Done = true;
            } else {
                inst->Error = true;
                switch (rc) {
                    case KRON_EC_ERR_INIT:      inst->ErrorID = 0x8011; break;
                    case KRON_EC_ERR_NO_SLAVES:  inst->ErrorID = 0x8012; break;
                    case KRON_EC_ERR_CONFIG:     inst->ErrorID = 0x8013; break;
                    case KRON_EC_ERR_OP:         inst->ErrorID = 0x8014; break;
                    default:                     inst->ErrorID = 0x8002; break;
                }
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
 * Only one SDO operation can be in-flight at a time.
 */
void EC_ReadSDO_Call(EC_ReadSDO *inst, KRON_EC_Config *cfg) {
    (void)cfg;
    bool rising = inst->Execute && !inst->_prevExecute;

    if (!inst->Execute && inst->_prevExecute) {
        inst->Done  = false;
        inst->Busy  = false;
        inst->Value = 0;
    }

    if (rising) {
        inst->Done    = false;
        inst->Busy    = true;
        inst->Error   = false;
        inst->ErrorID = 0;
        inst->Value   = 0;

        if (kron_ec_sdo_queue.state != KRON_EC_SDO_IDLE) {
            /* Queue busy — another SDO in-flight */
            inst->Error   = true;
            inst->ErrorID = 0x8020;
            inst->Busy    = false;
        } else {
            kron_ec_sdo_queue.slave_pos = inst->SlaveAddress;
            kron_ec_sdo_queue.index     = inst->Index;
            kron_ec_sdo_queue.subindex  = inst->SubIndex;
            kron_ec_sdo_queue.byte_size = inst->ByteSize ? inst->ByteSize : 4;
            kron_ec_sdo_queue.value     = 0;
            kron_ec_sdo_queue.state     = KRON_EC_SDO_READ_REQ; /* post to queue */
        }
    }

    /* Poll queue state */
    if (inst->Busy) {
        int st = kron_ec_sdo_queue.state;
        if (st == KRON_EC_SDO_DONE_OK) {
            inst->Value             = kron_ec_sdo_queue.value;
            inst->Done              = true;
            inst->Busy              = false;
            kron_ec_sdo_queue.state = KRON_EC_SDO_IDLE;
        } else if (st == KRON_EC_SDO_DONE_ERR) {
            inst->Error             = true;
            inst->ErrorID           = 0x8021;
            inst->Busy              = false;
            kron_ec_sdo_queue.state = KRON_EC_SDO_IDLE;
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
        inst->Busy = false;
    }

    if (rising) {
        inst->Done    = false;
        inst->Busy    = true;
        inst->Error   = false;
        inst->ErrorID = 0;

        if (kron_ec_sdo_queue.state != KRON_EC_SDO_IDLE) {
            inst->Error   = true;
            inst->ErrorID = 0x8020;
            inst->Busy    = false;
        } else {
            kron_ec_sdo_queue.slave_pos = inst->SlaveAddress;
            kron_ec_sdo_queue.index     = inst->Index;
            kron_ec_sdo_queue.subindex  = inst->SubIndex;
            kron_ec_sdo_queue.byte_size = inst->ByteSize ? inst->ByteSize : 4;
            kron_ec_sdo_queue.value     = inst->Value;
            kron_ec_sdo_queue.state     = KRON_EC_SDO_WRITE_REQ;
        }
    }

    /* Poll queue state */
    if (inst->Busy) {
        int st = kron_ec_sdo_queue.state;
        if (st == KRON_EC_SDO_DONE_OK) {
            inst->Done              = true;
            inst->Busy              = false;
            kron_ec_sdo_queue.state = KRON_EC_SDO_IDLE;
        } else if (st == KRON_EC_SDO_DONE_ERR) {
            inst->Error             = true;
            inst->ErrorID           = 0x8022;
            inst->Busy              = false;
            kron_ec_sdo_queue.state = KRON_EC_SDO_IDLE;
        }
    }

    inst->_prevExecute = inst->Execute;
}

#endif /* !KRON_EC_SIM */
