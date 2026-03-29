/*
 * kronethercatmaster.h  --  KronEditor EtherCAT Master runtime header
 *
 * Provides:
 *   - KRON_EC_Config:    Master configuration structure (port, cycle time, …)
 *   - KRON_EC_Slave:     Per-slave PDO/SDO configuration
 *   - KRON_EC_PDO_Entry: Single PDO entry (maps to a PLC variable)
 *   - KRON_EC_SDO:       SDO init command
 *   - kron_ec_init()     Initialise SOEM, configure slaves, enter OP state
 *   - kron_ec_pdo_read() Read inputs (TxPDOs from slaves → PLC vars)
 *   - kron_ec_pdo_write()Write outputs (PLC vars → RxPDOs to slaves)
 *   - kron_ec_close()    Shut down the master
 *   - kron_ec_process_sdo() Process one pending SDO request (call from SDO thread)
 *
 * User-facing diagnostic FBs (Pillar 3):
 *   - EC_GetMasterState  Returns current bus state
 *   - EC_GetSlaveState   Returns state of a specific slave node
 *   - EC_ResetBus        Re-initiates OP state after slave drop/reconnect
 *   - EC_ReadSDO         Asynchronous SDO read (non-blocking, uses SDO queue)
 *   - EC_WriteSDO        Asynchronous SDO write (non-blocking, uses SDO queue)
 *
 * In simulation mode (KRON_EC_SIM defined) all functions are no-ops that
 * return success so that the PLC code compiles and runs on the host.
 */

#ifndef KRONETHERCATMASTER_H
#define KRONETHERCATMASTER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* SOEM header is only needed for the real hardware implementation. */
#ifndef KRON_EC_SIM
#include "soem/soem.h"
#endif

/* ── Master state ─────────────────────────────────────────────────────────── */
typedef enum {
    KRON_EC_MASTER_NONE   = 0x00,  /* Not yet initialised */
    KRON_EC_MASTER_INIT   = 0x01,  /* Bus in INIT state */
    KRON_EC_MASTER_PREOP  = 0x02,  /* Bus in PRE-OP state */
    KRON_EC_MASTER_SAFEOP = 0x04,  /* Bus in SAFE-OP state */
    KRON_EC_MASTER_OP     = 0x08,  /* Bus fully operational */
    KRON_EC_MASTER_ERROR  = 0x10,  /* One or more slaves lost / error */
} KRON_EC_MasterState;

/* ── PDO data types ───────────────────────────────────────────────────────── */
typedef enum {
    KRON_EC_DTYPE_BOOL   = 0,
    KRON_EC_DTYPE_INT8   = 1,
    KRON_EC_DTYPE_UINT8  = 2,
    KRON_EC_DTYPE_INT16  = 3,
    KRON_EC_DTYPE_UINT16 = 4,
    KRON_EC_DTYPE_INT32  = 5,
    KRON_EC_DTYPE_UINT32 = 6,
    KRON_EC_DTYPE_INT64  = 7,
    KRON_EC_DTYPE_UINT64 = 8,
    KRON_EC_DTYPE_REAL32 = 9,
    KRON_EC_DTYPE_REAL64 = 10,
} KRON_EC_DataType;

/* ── PDO direction ────────────────────────────────────────────────────────── */
typedef enum {
    KRON_EC_DIR_INPUT  = 0,   /* TxPDO: slave → master (inputs)  */
    KRON_EC_DIR_OUTPUT = 1,   /* RxPDO: master → slave (outputs) */
} KRON_EC_Dir;

/* ── Single PDO entry ─────────────────────────────────────────────────────── */
typedef struct {
    uint16_t        index;        /* PDO object index (e.g. 0x6000)  */
    uint8_t         subindex;     /* PDO subindex                     */
    KRON_EC_DataType dtype;       /* Variable data type               */
    KRON_EC_Dir     dir;          /* Input or output                  */
    void           *var_ptr;      /* Pointer to PLC variable in SHM   */
    const char     *name;         /* Debug label                      */
} KRON_EC_PDO_Entry;

/* ── SDO init command ─────────────────────────────────────────────────────── */
typedef struct {
    uint16_t  index;
    uint8_t   subindex;
    uint32_t  value;
    uint8_t   byte_size;   /* 1/2/4 bytes */
} KRON_EC_SDO;

/* ── Per-slave configuration ─────────────────────────────────────────────── */
#define KRON_EC_MAX_PDO   256
#define KRON_EC_MAX_SDO   64

typedef struct {
    uint16_t          position;             /* Slave position (1-based) */
    uint32_t          vendor_id;
    uint32_t          product_code;
    char              name[64];
    KRON_EC_PDO_Entry pdo_entries[KRON_EC_MAX_PDO];
    int               pdo_count;
    KRON_EC_SDO       sdo_inits[KRON_EC_MAX_SDO];
    int               sdo_count;
    /* Runtime state (filled by kron_ec_init / kron_ec_check_state) */
    uint8_t           current_state;        /* EC_STATE_* from SOEM, or 0 */
    bool              link_up;              /* TRUE if slave is reachable  */
} KRON_EC_Slave;

/* ── Master configuration ────────────────────────────────────────────────── */
#define KRON_EC_MAX_SLAVES  64

typedef struct {
    char               ifname[64];          /* Network interface, e.g. "eth0"  */
    uint32_t           cycle_us;            /* Cycle time in microseconds       */
    KRON_EC_Slave      slaves[KRON_EC_MAX_SLAVES];
    int                slave_count;
    bool               dc_enable;          /* Enable distributed clocks        */
    /* Runtime state */
    KRON_EC_MasterState master_state;       /* Current overall bus state        */
    bool                is_operational;    /* TRUE when all slaves are in OP   */
    int                 found_slaves;      /* Number of slaves found by SOEM   */
} KRON_EC_Config;

/* ── Error codes ─────────────────────────────────────────────────────────── */
#define KRON_EC_OK              0
#define KRON_EC_ERR_INIT       -1
#define KRON_EC_ERR_NO_SLAVES  -2
#define KRON_EC_ERR_CONFIG     -3
#define KRON_EC_ERR_OP         -4
#define KRON_EC_ERR_IO         -5

/* ── Asynchronous SDO request queue (single-slot, thread-safe via volatile) ─ */
/* State machine: IDLE(0) → REQUEST(1/2) → DONE_OK(-1) / DONE_ERR(-2)        */
#define KRON_EC_SDO_IDLE      0
#define KRON_EC_SDO_WRITE_REQ 1
#define KRON_EC_SDO_READ_REQ  2
#define KRON_EC_SDO_DONE_OK  -1
#define KRON_EC_SDO_DONE_ERR -2

typedef struct {
    volatile int  state;      /* See KRON_EC_SDO_* defines above           */
    uint16_t      slave_pos;  /* 1-based slave position                    */
    uint16_t      index;      /* SDO object index                          */
    uint8_t       subindex;
    uint8_t       byte_size;  /* 1/2/4                                     */
    uint32_t      value;      /* Write: input value; Read: output result   */
} KRON_EC_SDO_Queue;

/* Global singleton SDO queue — defined in kronethercatmaster.c             */
extern KRON_EC_SDO_Queue kron_ec_sdo_queue;

/* ── API ──────────────────────────────────────────────────────────────────── */

#ifdef KRON_EC_SIM
/* ── Simulation stubs — no real hardware accessed ── */
static inline int  kron_ec_init(KRON_EC_Config *cfg) {
    if (!cfg) return KRON_EC_ERR_INIT;
    cfg->master_state   = KRON_EC_MASTER_OP;
    cfg->is_operational = true;
    /* In simulation, pretend all configured slaves are present and in OP */
    cfg->found_slaves   = cfg->slave_count;
    for (int _i = 0; _i < cfg->slave_count; _i++) {
        cfg->slaves[_i].current_state = 0x08; /* EC_STATE_OPERATIONAL */
        cfg->slaves[_i].link_up       = true;
    }
    return KRON_EC_OK;
}
static inline void kron_ec_pdo_read(KRON_EC_Config *cfg)    { (void)cfg; }
static inline void kron_ec_pdo_write(KRON_EC_Config *cfg)   { (void)cfg; }
static inline void kron_ec_close(KRON_EC_Config *cfg)       { if (cfg) { cfg->master_state = KRON_EC_MASTER_NONE; cfg->is_operational = false; cfg->found_slaves = 0; } }
static inline void kron_ec_check_state(KRON_EC_Config *cfg) {
    /* In simulation keep all slaves alive */
    if (!cfg) return;
    cfg->master_state   = KRON_EC_MASTER_OP;
    cfg->is_operational = true;
    cfg->found_slaves   = cfg->slave_count;
    for (int _i = 0; _i < cfg->slave_count; _i++) {
        cfg->slaves[_i].current_state = 0x08;
        cfg->slaves[_i].link_up       = true;
    }
}
static inline void kron_ec_process_sdo(KRON_EC_Config *cfg) {
    (void)cfg;
    /* In sim: immediately complete any pending SDO request */
    if (kron_ec_sdo_queue.state == KRON_EC_SDO_WRITE_REQ)
        kron_ec_sdo_queue.state = KRON_EC_SDO_DONE_OK;
    else if (kron_ec_sdo_queue.state == KRON_EC_SDO_READ_REQ) {
        kron_ec_sdo_queue.value = 0;
        kron_ec_sdo_queue.state = KRON_EC_SDO_DONE_OK;
    }
}

/* SDO queue storage for sim (also defined in .c for real mode) */
#ifndef KRON_EC_SDO_QUEUE_DEFINED
#define KRON_EC_SDO_QUEUE_DEFINED
KRON_EC_SDO_Queue kron_ec_sdo_queue = { KRON_EC_SDO_IDLE, 0, 0, 0, 0, 0 };
#endif

#else
/* ── Real SOEM-backed implementations (defined in kronethercatmaster.c) ── */
int  kron_ec_init(KRON_EC_Config *cfg);
void kron_ec_pdo_read(KRON_EC_Config *cfg);
void kron_ec_pdo_write(KRON_EC_Config *cfg);
void kron_ec_close(KRON_EC_Config *cfg);
void kron_ec_check_state(KRON_EC_Config *cfg);
void kron_ec_process_sdo(KRON_EC_Config *cfg);
#endif /* KRON_EC_SIM */

/* ═══════════════════════════════════════════════════════════════════════════
 * USER-FACING DIAGNOSTIC FUNCTION BLOCKS (Pillar 3)
 * These blocks are always available regardless of KRON_EC_SIM.
 * They access the master state via the global __ec_cfg pointer.
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── EC_GetMasterState ────────────────────────────────────────────────────── */
typedef struct {
    /* VAR_INPUT */
    bool     Enable;
    /* VAR_OUTPUT */
    bool     Valid;
    bool     Error;
    uint16_t ErrorID;
    uint8_t  State;          /* KRON_EC_MasterState value */
    bool     Operational;    /* TRUE when all slaves are in OP */
    uint16_t SlaveCount;     /* Number of slaves found */
    /* Internal */
    bool     _prevEnable;
} EC_GetMasterState;

/* ── EC_GetSlaveState ─────────────────────────────────────────────────────── */
typedef struct {
    /* VAR_INPUT */
    bool     Enable;
    uint16_t SlaveAddress;  /* 1-based slave position */
    /* VAR_OUTPUT */
    bool     Valid;
    bool     Error;
    uint16_t ErrorID;
    uint8_t  State;         /* EC_STATE_* of the specific slave */
    bool     LinkUp;        /* Slave is reachable */
    /* Internal */
    bool     _prevEnable;
} EC_GetSlaveState;

/* ── EC_ResetBus ──────────────────────────────────────────────────────────── */
typedef struct {
    /* VAR_INPUT */
    bool     Execute;
    /* VAR_OUTPUT */
    bool     Done;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    /* Internal */
    bool     _prevExecute;
} EC_ResetBus;

/* ── EC_ReadSDO ───────────────────────────────────────────────────────────── */
typedef struct {
    /* VAR_INPUT */
    bool     Execute;
    uint16_t SlaveAddress; /* 1-based slave position */
    uint16_t Index;        /* SDO object index       */
    uint8_t  SubIndex;
    uint8_t  ByteSize;     /* 1 / 2 / 4              */
    /* VAR_OUTPUT */
    bool     Done;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    uint32_t Value;        /* Read value (valid when Done=TRUE) */
    /* Internal */
    bool     _prevExecute;
} EC_ReadSDO;

/* ── EC_WriteSDO ──────────────────────────────────────────────────────────── */
typedef struct {
    /* VAR_INPUT */
    bool     Execute;
    uint16_t SlaveAddress;
    uint16_t Index;
    uint8_t  SubIndex;
    uint8_t  ByteSize;
    uint32_t Value;
    /* VAR_OUTPUT */
    bool     Done;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    /* Internal */
    bool     _prevExecute;
} EC_WriteSDO;

/* ── EC FB Call function declarations / inline stubs ─────────────────────── */
/* Real mode: implemented in kronethercatmaster.c and linked via .a           */
/* Sim mode:  inline stubs — library is not linked, no external symbol needed */
#ifndef KRON_EC_SIM

void EC_GetMasterState_Call(EC_GetMasterState *inst, KRON_EC_Config *cfg);
void EC_GetSlaveState_Call(EC_GetSlaveState *inst, KRON_EC_Config *cfg);
void EC_ResetBus_Call(EC_ResetBus *inst, KRON_EC_Config *cfg);
void EC_ReadSDO_Call(EC_ReadSDO *inst, KRON_EC_Config *cfg);
void EC_WriteSDO_Call(EC_WriteSDO *inst, KRON_EC_Config *cfg);

#else /* KRON_EC_SIM */

static inline void EC_GetMasterState_Call(EC_GetMasterState *inst, KRON_EC_Config *cfg) {
    if (!inst->Enable) { inst->Valid = false; return; }
    inst->Valid       = true;
    inst->Error       = false;
    inst->ErrorID     = 0;
    inst->State       = cfg ? (uint8_t)cfg->master_state : (uint8_t)KRON_EC_MASTER_OP;
    inst->Operational = cfg ? cfg->is_operational : true;
    inst->SlaveCount  = cfg ? (uint16_t)cfg->found_slaves : 0;
}

static inline void EC_GetSlaveState_Call(EC_GetSlaveState *inst, KRON_EC_Config *cfg) {
    if (!inst->Enable) { inst->Valid = false; return; }
    inst->Valid   = true;
    inst->Error   = false;
    inst->ErrorID = 0;
    inst->State   = 0x08; /* EC_STATE_OPERATIONAL */
    inst->LinkUp  = true;
    (void)cfg;
}

static inline void EC_ResetBus_Call(EC_ResetBus *inst, KRON_EC_Config *cfg) {
    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;
    if (rising) { inst->Done = true; inst->Busy = false; inst->Error = false; inst->ErrorID = 0; }
    (void)cfg;
}

static inline void EC_ReadSDO_Call(EC_ReadSDO *inst, KRON_EC_Config *cfg) {
    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;
    if (rising) { inst->Done = true; inst->Busy = false; inst->Error = false; inst->ErrorID = 0; inst->Value = 0; }
    (void)cfg;
}

static inline void EC_WriteSDO_Call(EC_WriteSDO *inst, KRON_EC_Config *cfg) {
    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;
    if (rising) { inst->Done = true; inst->Busy = false; inst->Error = false; inst->ErrorID = 0; }
    (void)cfg;
}

#endif /* KRON_EC_SIM */

#endif /* KRONETHERCATMASTER_H */
