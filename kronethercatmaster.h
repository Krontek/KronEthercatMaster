/*
 * kronec.h  --  KronEditor EtherCAT Master runtime header
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
 *
 * In simulation mode (KRON_EC_SIM defined) all functions are no-ops that
 * return success so that the PLC code compiles and runs on the host.
 */

#ifndef KRONEC_H
#define KRONEC_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

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
} KRON_EC_Slave;

/* ── Master configuration ────────────────────────────────────────────────── */
#define KRON_EC_MAX_SLAVES  64

typedef struct {
    char           ifname[64];              /* Network interface, e.g. "eth0"  */
    uint32_t       cycle_us;               /* Cycle time in microseconds       */
    KRON_EC_Slave  slaves[KRON_EC_MAX_SLAVES];
    int            slave_count;
    bool           dc_enable;              /* Enable distributed clocks        */
} KRON_EC_Config;

/* ── Error codes ─────────────────────────────────────────────────────────── */
#define KRON_EC_OK              0
#define KRON_EC_ERR_INIT       -1
#define KRON_EC_ERR_NO_SLAVES  -2
#define KRON_EC_ERR_CONFIG     -3
#define KRON_EC_ERR_OP         -4
#define KRON_EC_ERR_IO         -5

/* ── API ──────────────────────────────────────────────────────────────────── */

#ifdef KRON_EC_SIM
/* ── Simulation stubs — no real hardware accessed ── */
static inline int  kron_ec_init(KRON_EC_Config *cfg)       { (void)cfg; return KRON_EC_OK; }
static inline void kron_ec_pdo_read(KRON_EC_Config *cfg)   { (void)cfg; }
static inline void kron_ec_pdo_write(KRON_EC_Config *cfg)  { (void)cfg; }
static inline void kron_ec_close(KRON_EC_Config *cfg)      { (void)cfg; }
static inline void kron_ec_check_state(KRON_EC_Config *cfg){ (void)cfg; }
#else
/* ── Real SOEM-backed implementations (defined in kronec.c) ── */
int  kron_ec_init(KRON_EC_Config *cfg);
void kron_ec_pdo_read(KRON_EC_Config *cfg);
void kron_ec_pdo_write(KRON_EC_Config *cfg);
void kron_ec_close(KRON_EC_Config *cfg);
void kron_ec_check_state(KRON_EC_Config *cfg);  /* Call periodically to recover lost slaves */
#endif /* KRON_EC_SIM */

#endif /* KRONEC_H */
