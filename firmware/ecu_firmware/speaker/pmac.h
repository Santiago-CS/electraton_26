/*
 * pmac.h - CANopen decoder for the PMAC (Sevcon-family) motor controller.
 *
 * The TPDO map in pmac.c is "CAN Open RD TPDO Configuration.xlsx", row for
 * row, checked against candump captures of the real controller (node 1,
 * 500 kbit/s, SYNC every 40 ms).
 */
#ifndef PMAC_H
#define PMAC_H

#include <stdint.h>
#include <stdio.h>

typedef enum { T_U8, T_I8, T_U16, T_I16, T_I32 } pmac_ftype;

/* One value per mapped object. Index into pmac_state.v[]. */
enum {
    /* TPDO1 0x391 */ SIG_ID_TARGET, SIG_IQ_TARGET, SIG_ID, SIG_IQ,
    /* TPDO2 0x330 */ SIG_UD, SIG_UQ, SIG_MODULATION, SIG_INDUCTANCE,
    /* TPDO3 0x411 */ SIG_VLIM_CIRCLE, SIG_MAX_FLUX_I, SIG_MAX_IQ, SIG_MOTOR_TEMP,
    /* TPDO4 0x271 */ SIG_CAP_V, SIG_HEATSINK_TEMP, SIG_BATT_I, SIG_TORQUE,
    /* TPDO5 0x454 */ SIG_DIN, SIG_SPEED,
    SIG_COUNT
};

typedef struct {
    int         sig;     /* slot in pmac_state.v[]                          */
    const char *od;      /* index / sub-index as the spreadsheet writes it  */
    pmac_ftype  type;
    double      scale;   /* engineering = raw * scale (sheet "Scaling")     */
    const char *unit;
    const char *label;   /* short label for text output                     */
    const char *col;     /* CSV column name                                 */
    int         dec;     /* decimals in text output                         */
} pmac_field;

typedef struct {
    uint16_t          cob_id;
    const char       *name;
    const pmac_field *f;
    int               nf;
} pmac_tpdo_def;

#define PMAC_NTPDO 5
extern const pmac_tpdo_def pmac_tpdo[PMAC_NTPDO];

/* Payload length a TPDO's mapping implies: the sum of its field widths. */
int pmac_tpdo_len(const pmac_tpdo_def *p);

typedef enum {
    FR_OTHER, FR_SYNC, FR_TPDO, FR_EMCY, FR_HEARTBEAT, FR_NMT,
    FR_CANERR,     /* controller error frame; id carries the CAN_ERR_* flags */
    FR_TICK        /* no frame for a while (live mode keep-alive)            */
} frame_kind;

typedef struct {
    int      node;                     /* controller node id               */

    double   v[SIG_COUNT];             /* last value, engineering units    */
    int32_t  raw[SIG_COUNT];           /* last raw value                   */
    int      have[SIG_COUNT];          /* value received at least once     */
    double   t_sig[SIG_COUNT];         /* when each value last arrived     */

    unsigned long n_tpdo[PMAC_NTPDO];  /* frames received per TPDO         */
    unsigned long n_len_err[PMAC_NTPDO]; /* frames whose length != the map */
    double   t_first[PMAC_NTPDO], t_last[PMAC_NTPDO];

    unsigned long n_sync;
    double   t_sync_first, t_sync_last;

    int      nmt_state;                /* last heartbeat state, -1 = none  */
    double   t_hb;

    unsigned long n_emcy;
    uint16_t emcy_code;                /* CANopen emergency error code     */
    uint8_t  emcy_reg;                 /* error register (1001h)           */
    uint16_t fault_id;                 /* Sevcon fault identifier          */
    double   t_emcy;

    unsigned long n_canerr;            /* controller error frames          */
} pmac_state;

void       pmac_init(pmac_state *s, int node);

/* Feed one standard-id frame. For a TPDO, *which is its index in pmac_tpdo[].
 * Fields that do not fit in a short frame keep their previous value. */
frame_kind pmac_feed(pmac_state *s, double t, uint32_t id, int len,
                     const uint8_t *d, int *which);

const char *pmac_nmt_name(int st);
const char *pmac_emcy_class(uint16_t code);
const char *pmac_fault_text(uint16_t fault_id, const char **cause);

/* ---- output ---- */

typedef struct {
    int    verbose;     /* also SYNC, every heartbeat, unknown frames       */
    double rated_nm;    /* 2916h:01 rated torque, 0 = unknown               */
} pmac_print_opts;

/* One human-readable line for a decoded frame (nothing for frames that
 * are only shown with verbose). trel = seconds since the first frame. */
void pmac_print(FILE *o, const pmac_state *s, frame_kind k, int which,
                double trel, uint32_t id, int len, const uint8_t *d,
                const pmac_print_opts *opt);

/* CSV pieces: the header columns and one row of values, each starting with
 * a comma, so the caller adds its own leading and trailing columns. */
void pmac_csv_header(FILE *o, double rated_nm);
void pmac_csv_values(FILE *o, const pmac_state *s, double rated_nm);

/* Counts, periods, length checks, node state, fault, last values. */
void pmac_summary(FILE *o, const pmac_state *s, double t0,
                  unsigned long nframes, unsigned long nbad);

/* One candump line -> frame. Accepts candump -L/-l (391#0011..), -ta, -tA
 * and no-timestamp output. Returns 1 for a standard data frame, 0 for a line
 * to skip silently (blank, extended id, RTR, FD), -1 for a malformed or
 * truncated line. *t is untouched when the line has no timestamp. */
int pmac_parse_candump(const char *line, double *t, uint32_t *id, int *len,
                       uint8_t *d);

#endif
