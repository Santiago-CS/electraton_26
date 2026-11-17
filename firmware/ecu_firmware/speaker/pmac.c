/*
 * pmac.c - CANopen decoder for the PMAC (Sevcon-family) motor controller.
 *
 * The map below is "CAN Open RD TPDO Configuration.xlsx", row for row.
 * CANopen packs the mapped objects into each PDO in mapping order,
 * byte-aligned and little-endian, so byte offsets are not written down: they
 * follow from the order and width of the rows. If the controller is ever
 * remapped, edit the rows of the affected TPDO and everything else follows.
 *
 * Checked against candump captures of the real controller: every TPDO
 * arrives with exactly the payload length this map implies (8, 8, 8, 7, 5).
 */

#include "pmac.h"

#include <stdlib.h>
#include <string.h>
#include <linux/can/error.h>

/* ======================================================================
 *  TPDO map
 * ====================================================================== */

static const int type_width[] = { 1, 1, 2, 2, 4 };

#define S_16TH  (1.0 / 16.0)          /* sheet: 0.0625                 */
#define S_MOD   (100.0 / 255.0)       /* sheet: 0.39215686274509803    */
#define S_2P24  (1.0 / 16777216.0)    /* sheet: 5.960464477539063e-08  */

static const pmac_field tpdo1[] = {   /* 0x391  SYNC, 1 sync per transmit */
    { SIG_ID_TARGET, "4600h/5", T_I16, S_16TH, "A", "Id*", "id_target_A", 2 },
    { SIG_IQ_TARGET, "4600h/6", T_I16, S_16TH, "A", "Iq*", "iq_target_A", 2 },
    { SIG_ID,        "4600h/7", T_I16, S_16TH, "A", "Id",  "id_A",        2 },
    { SIG_IQ,        "4600h/8", T_I16, S_16TH, "A", "Iq",  "iq_A",        2 },
};

static const pmac_field tpdo2[] = {   /* 0x330 */
    { SIG_UD,         "4600h/9",  T_I16, S_16TH, "V", "Ud",  "ud_V",           2 },
    { SIG_UQ,         "4600h/10", T_I16, S_16TH, "V", "Uq",  "uq_V",           2 },
    { SIG_MODULATION, "4600h/11", T_I16, S_MOD,  "%", "mod", "modulation_pct", 1 },
    /* Sheet: Integer16 with limits 0..65535. The limits are an unsigned range
     * and an inductance cannot be negative, so it is read as unsigned.
     * Sheet scale is 2^-24 H per bit; kept in microhenry here. */
    { SIG_INDUCTANCE, "4602h/29", T_U16, S_2P24 * 1e6, "uH", "L", "inductance_uH", 2 },
};

static const pmac_field tpdo3[] = {   /* 0x411 */
    /* Same Integer16-with-0..65535-limits case as the inductance: unsigned.
     * At standstill there is no back-EMF, so the voltage limit circle sits
     * at a large clamp value; it only means something while spinning. */
    { SIG_VLIM_CIRCLE, "4602h/31", T_U16, 1.0, "A",    "Vlim",   "vlim_circle_A", 0 },
    { SIG_MAX_FLUX_I,  "4602h/32", T_U16, 1.0, "A",    "maxIf",  "max_flux_A",    0 },
    { SIG_MAX_IQ,      "4602h/33", T_U16, 1.0, "A",    "maxIq",  "max_iq_A",      0 },
    /* The sheet gives 4600h/3 here AND for Capacitor Voltage in TPDO4; one of
     * the two index cells is a typo. The byte position is not affected. No
     * scale in the sheet; the description says 1 degC per bit. */
    { SIG_MOTOR_TEMP,  "4600h/3?", T_I16, 1.0, "degC", "Tmotor", "motor_temp_C",  0 },
};

static const pmac_field tpdo4[] = {   /* 0x271 */
    { SIG_CAP_V,         "4600h/3", T_U16, S_16TH, "V",    "Vcap",   "cap_V",      2 },
    { SIG_HEATSINK_TEMP, "4600h/4", T_I8,  1.0,    "degC", "Thsink", "heatsink_C", 0 },
    { SIG_BATT_I,        "4600h/2", T_I16, S_16TH, "A",    "Ibat",   "batt_A",     2 },
    /* Thousandths of the rated torque in 2916h/1 (not in the sheet). */
    { SIG_TORQUE,        "2020h/6", T_I16, 0.001,  "pu",   "torque", "torque_pu",  3 },
};

static const pmac_field tpdo5[] = {   /* 0x454 */
    { SIG_DIN,   "6800h/1", T_U8,  1.0, "",    "DIN",   "din",       0 },  /* bit n = input n+1 */
    { SIG_SPEED, "606Ch/0", T_I32, 1.0, "rpm", "speed", "speed_rpm", 0 },
};

#define NF(a) ((int)(sizeof (a) / sizeof (a)[0]))
const pmac_tpdo_def pmac_tpdo[PMAC_NTPDO] = {
    { 0x391, "TPDO1 currents",        tpdo1, NF(tpdo1) },
    { 0x330, "TPDO2 voltages",        tpdo2, NF(tpdo2) },
    { 0x411, "TPDO3 field weakening", tpdo3, NF(tpdo3) },
    { 0x271, "TPDO4 DC link/torque",  tpdo4, NF(tpdo4) },
    { 0x454, "TPDO5 inputs/speed",    tpdo5, NF(tpdo5) },
};

int pmac_tpdo_len(const pmac_tpdo_def *p)
{
    int i, n = 0;
    for (i = 0; i < p->nf; i++)
        n += type_width[p->f[i].type];
    return n;
}

/* ======================================================================
 *  Decoder
 * ====================================================================== */

#define COB_NMT        0x000u
#define COB_SYNC       0x080u
#define COB_EMCY(n)    (0x080u + (n))
#define COB_HB(n)      (0x700u + (n))

void pmac_init(pmac_state *s, int node)
{
    memset(s, 0, sizeof *s);
    s->node = node;
    s->nmt_state = -1;
}

static int32_t get_raw(const uint8_t *p, pmac_ftype t)
{
    /* Assembled byte by byte: CANopen is little-endian on the wire, and the
     * fields are not aligned (TPDO5's Integer32 starts at byte 1). */
    switch (t) {
    case T_U8:  return p[0];
    case T_I8:  return (int8_t)p[0];
    case T_U16: return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8));
    case T_I16: return (int16_t)(uint16_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8));
    case T_I32: return (int32_t)((uint32_t)p[0]         | ((uint32_t)p[1] << 8) |
                                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
    }
    return 0;
}

frame_kind pmac_feed(pmac_state *s, double t, uint32_t id, int len,
                     const uint8_t *d, int *which)
{
    int i, k;

    if (id == COB_SYNC && len == 0) {
        if (s->n_sync++ == 0)
            s->t_sync_first = t;
        s->t_sync_last = t;
        return FR_SYNC;
    }

    for (k = 0; k < PMAC_NTPDO; k++) {
        const pmac_tpdo_def *p = &pmac_tpdo[k];
        int off = 0;
        if (id != p->cob_id)
            continue;
        if (len != pmac_tpdo_len(p))
            s->n_len_err[k]++;
        for (i = 0; i < p->nf; i++) {
            const pmac_field *f = &p->f[i];
            int w = type_width[f->type];
            if (off + w <= len) {
                int32_t r = get_raw(d + off, f->type);
                s->raw[f->sig]   = r;
                s->v[f->sig]     = (double)r * f->scale;
                s->have[f->sig]  = 1;
                s->t_sig[f->sig] = t;
            }
            off += w;
        }
        if (s->n_tpdo[k]++ == 0)
            s->t_first[k] = t;
        s->t_last[k] = t;
        if (which)
            *which = k;
        return FR_TPDO;
    }

    if (id == COB_EMCY((uint32_t)s->node) && len >= 3) {
        /* CANopen EMCY: bytes 0-1 error code, byte 2 error register,
         * bytes 3-7 manufacturer specific. Sevcon puts its own fault
         * identifier in bytes 3-4. Code 0000 = fault cleared. */
        s->n_emcy++;
        s->emcy_code = (uint16_t)(d[0] | (d[1] << 8));
        s->emcy_reg  = d[2];
        s->fault_id  = len >= 5 ? (uint16_t)(d[3] | (d[4] << 8)) : 0;
        s->t_emcy    = t;
        return FR_EMCY;
    }

    if (id == COB_HB((uint32_t)s->node) && len >= 1) {
        s->nmt_state = d[0] & 0x7F;
        s->t_hb = t;
        return FR_HEARTBEAT;
    }

    if (id == COB_NMT && len == 2)
        return FR_NMT;

    return FR_OTHER;
}

const char *pmac_nmt_name(int st)
{
    switch (st) {
    case -1:   return "no heartbeat";
    case 0x00: return "boot-up";
    case 0x04: return "Stopped";
    case 0x05: return "Operational";
    case 0x7F: return "Pre-operational";
    default:   return "unknown state";
    }
}

const char *pmac_emcy_class(uint16_t c)
{
    if (c == 0x0000)          return "error reset / no error";
    switch (c >> 12) {
    case 0x1: return "generic error";
    case 0x2: return "current";
    case 0x3: return "voltage";
    case 0x4: return "temperature";
    case 0x5: return "device hardware";
    case 0x6: return "device software";
    case 0x7: return "additional modules";
    case 0x8: return "monitoring / communication";
    case 0x9: return "external error";
    case 0xF: return "additional functions / device specific";
    default:  return "unclassified";
    }
}

/* Sevcon Gen4 fault identifiers, drive-inhibit group. Text from the OVMS
 * Renault Twizy (Sevcon Gen4) fault table. Other ids print as hex: look them
 * up in DVT or the Gen4 manual. */
static const struct { uint16_t id; const char *name, *cause; } sevcon_fault[] = {
    { 0x4881, "Seat Fault",          "direction selected with operator not seated" },
    { 0x4882, "Two Direction Fault", "forward and reverse active together > 200 ms" },
    { 0x4883, "SRO Fault",           "FS1 active without a direction selected" },
    { 0x4884, "Sequence Fault",      "a drive switch was active at power up - release all drive switches" },
    { 0x4885, "FS1 Recycle Fault",   "FS1 active after a direction change" },
    { 0x4886, "Inch Fault",          "inch switch active together with another drive switch" },
    { 0x4887, "Overload Fault",      "vehicle overloaded" },
    { 0x488A, "Traction Inhibit Fault", "traction inhibit switch active (2137h)" },
};

const char *pmac_fault_text(uint16_t id, const char **cause)
{
    size_t i;
    for (i = 0; i < sizeof sevcon_fault / sizeof sevcon_fault[0]; i++)
        if (sevcon_fault[i].id == id) {
            if (cause) *cause = sevcon_fault[i].cause;
            return sevcon_fault[i].name;
        }
    if (cause) *cause = NULL;
    return NULL;
}

/* ======================================================================
 *  Output
 * ====================================================================== */

static void print_field(FILE *o, const pmac_field *f, const pmac_state *s,
                        const pmac_print_opts *opt)
{
    if (f->sig == SIG_DIN) {
        int b, any = 0;
        fprintf(o, "  %s=0x%02X [", f->label, (unsigned)s->raw[f->sig]);
        for (b = 0; b < 8; b++)
            if (s->raw[f->sig] & (1 << b)) {
                fprintf(o, "%sin%d", any ? " " : "", b + 1);
                any = 1;
            }
        fprintf(o, "]");
        return;
    }
    fprintf(o, "  %s=%.*f", f->label, f->dec, s->v[f->sig]);
    if (f->unit[0])
        fprintf(o, " %s", f->unit);
    if (f->sig == SIG_TORQUE && opt->rated_nm > 0)
        fprintf(o, " (%.1f Nm)", s->v[SIG_TORQUE] * opt->rated_nm);
}

void pmac_print(FILE *o, const pmac_state *s, frame_kind k, int which,
                double trel, uint32_t id, int len, const uint8_t *d,
                const pmac_print_opts *opt)
{
    static int last_nmt = -2;
    int i;

    switch (k) {
    case FR_TPDO: {
        const pmac_tpdo_def *p = &pmac_tpdo[which];
        int off = 0;
        fprintf(o, "%9.4f  %03X %-22s", trel, (unsigned)id, p->name);
        for (i = 0; i < p->nf; i++) {
            off += type_width[p->f[i].type];
            if (off <= len)
                print_field(o, &p->f[i], s, opt);
            else
                fprintf(o, "  %s=--", p->f[i].label);   /* not in this short frame */
        }
        if (len != pmac_tpdo_len(p))
            fprintf(o, "  !! %d bytes, map expects %d", len, pmac_tpdo_len(p));
        fprintf(o, "\n");
        break;
    }
    case FR_SYNC:
        if (opt->verbose)
            fprintf(o, "%9.4f  080 SYNC\n", trel);
        break;
    case FR_EMCY: {
        const char *cause, *name = pmac_fault_text(s->fault_id, &cause);
        fprintf(o, "%9.4f  %03X EMCY node %d  code 0x%04X (%s)  reg 0x%02X",
                trel, (unsigned)id, s->node, s->emcy_code,
                pmac_emcy_class(s->emcy_code), s->emcy_reg);
        if (s->emcy_code == 0x0000)
            fprintf(o, "  -> fault cleared");
        else if (name)
            fprintf(o, "  fault 0x%04X %s: %s", s->fault_id, name, cause);
        else
            fprintf(o, "  fault 0x%04X", s->fault_id);
        fprintf(o, "\n");
        break;
    }
    case FR_HEARTBEAT:
        if (opt->verbose || s->nmt_state != last_nmt)
            fprintf(o, "%9.4f  %03X HEARTBEAT node %d %s\n", trel, (unsigned)id,
                    s->node, pmac_nmt_name(s->nmt_state));
        last_nmt = s->nmt_state;
        break;
    case FR_NMT:
        fprintf(o, "%9.4f  000 NMT command 0x%02X to node %d\n", trel, d[0], d[1]);
        break;
    case FR_CANERR:
        fprintf(o, "%9.4f  CAN error frame:%s%s%s%s%s\n", trel,
                id & CAN_ERR_ACK       ? " no-ACK"     : "",
                id & CAN_ERR_BUSOFF    ? " bus-off"    : "",
                id & CAN_ERR_CRTL      ? " controller" : "",
                id & CAN_ERR_PROT      ? " protocol"   : "",
                id & CAN_ERR_RESTARTED ? " restarted"  : "");
        break;
    case FR_OTHER:
        if (opt->verbose) {
            fprintf(o, "%9.4f  %03X [%d]", trel, (unsigned)id, len);
            for (i = 0; i < len; i++)
                fprintf(o, " %02X", d[i]);
            fprintf(o, "\n");
        }
        break;
    case FR_TICK:
        break;
    }
}

void pmac_csv_header(FILE *o, double rated_nm)
{
    int k, i;
    for (k = 0; k < PMAC_NTPDO; k++)
        for (i = 0; i < pmac_tpdo[k].nf; i++)
            fprintf(o, ",%s", pmac_tpdo[k].f[i].col);
    if (rated_nm > 0)
        fprintf(o, ",torque_Nm");
    fprintf(o, ",nmt_state,emcy_code,fault_id");
}

void pmac_csv_values(FILE *o, const pmac_state *s, double rated_nm)
{
    int k, i;
    for (k = 0; k < PMAC_NTPDO; k++)
        for (i = 0; i < pmac_tpdo[k].nf; i++) {
            const pmac_field *f = &pmac_tpdo[k].f[i];
            if (s->have[f->sig]) fprintf(o, ",%.9g", s->v[f->sig]);
            else                 fprintf(o, ",");
        }
    if (rated_nm > 0) {
        if (s->have[SIG_TORQUE]) fprintf(o, ",%.3f", s->v[SIG_TORQUE] * rated_nm);
        else                     fprintf(o, ",");
    }
    if (s->nmt_state >= 0) fprintf(o, ",0x%02X", s->nmt_state); else fprintf(o, ",");
    if (s->n_emcy)         fprintf(o, ",0x%04X,0x%04X", s->emcy_code, s->fault_id);
    else                   fprintf(o, ",,");
}

void pmac_summary(FILE *o, const pmac_state *s, double t0,
                  unsigned long nframes, unsigned long nbad)
{
    int k, i;

    fprintf(o, "\n--- PMAC summary ------------------------------------------------\n");
    fprintf(o, "frames %lu", nframes);
    if (nbad)        fprintf(o, ", %lu unparseable/truncated lines skipped", nbad);
    if (s->n_canerr) fprintf(o, ", %lu CAN error frames", s->n_canerr);
    fprintf(o, "\n");

    fprintf(o, "SYNC 080                     %6lu", s->n_sync);
    if (s->n_sync > 1)
        fprintf(o, "   every %.1f ms",
                1e3 * (s->t_sync_last - s->t_sync_first) / (double)(s->n_sync - 1));
    fprintf(o, "\n");

    for (k = 0; k < PMAC_NTPDO; k++) {
        const pmac_tpdo_def *p = &pmac_tpdo[k];
        fprintf(o, "%-22s %03X  %6lu", p->name, p->cob_id, s->n_tpdo[k]);
        if (s->n_tpdo[k] > 1)
            fprintf(o, "   every %.1f ms",
                    1e3 * (s->t_last[k] - s->t_first[k]) / (double)(s->n_tpdo[k] - 1));
        if (!s->n_tpdo[k])
            fprintf(o, "   NOT SEEN");
        else if (s->n_len_err[k])
            fprintf(o, "   %lu frames with length != %d (mapping changed?)",
                    s->n_len_err[k], pmac_tpdo_len(p));
        else
            fprintf(o, "   length %d ok", pmac_tpdo_len(p));
        fprintf(o, "\n");
    }

    fprintf(o, "node %d: %s\n", s->node, pmac_nmt_name(s->nmt_state));

    if (s->n_emcy) {
        const char *name = pmac_fault_text(s->fault_id, NULL);
        fprintf(o, "EMCY x%lu, last at %.3f s: code 0x%04X", s->n_emcy,
                s->t_emcy - t0, s->emcy_code);
        if (s->emcy_code == 0)
            fprintf(o, " (cleared)\n");
        else
            fprintf(o, ", fault 0x%04X %s - no clearing EMCY (code 0000) seen\n",
                    s->fault_id, name ? name : "");
    }

    fprintf(o, "last values:\n");
    for (k = 0; k < PMAC_NTPDO; k++)
        for (i = 0; i < pmac_tpdo[k].nf; i++) {
            const pmac_field *f = &pmac_tpdo[k].f[i];
            if (!s->have[f->sig])
                continue;
            if (f->sig == SIG_DIN)
                fprintf(o, "  %-22s %-9s         0x%02X   (bit n = input n+1)\n",
                        f->col, f->od, (unsigned)s->raw[f->sig]);
            else
                fprintf(o, "  %-22s %-9s %12.4f %s\n", f->col, f->od,
                        s->v[f->sig], f->unit);
        }
}

/* ======================================================================
 *  candump log parsing
 * ====================================================================== */

static int hexval(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

int pmac_parse_candump(const char *ln, double *t, uint32_t *id, int *len, uint8_t *d)
{
    const char *p = ln;
    char tok[64];
    int n, i;

    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0' || *p == '\n' || *p == '\r' || *p == '#')
        return 0;

    if (*p == '(') {                               /* timestamp */
        int Y, M, D, h, m;
        double sec;
        const char *e = strchr(p, ')');
        if (!e) return -1;
        if (sscanf(p + 1, "%d-%d-%d %d:%d:%lf", &Y, &M, &D, &h, &m, &sec) == 6)
            *t = h * 3600.0 + m * 60.0 + sec;      /* candump -tA */
        else if (sscanf(p + 1, "%lf", t) != 1)
            return -1;
        p = e + 1;
    }

    if (sscanf(p, "%63s%n", tok, &n) != 1) return -1;   /* interface */
    p += n;
    if (sscanf(p, "%63s%n", tok, &n) != 1) return -1;   /* id or id#data */
    p += n;

    {
        char *hash = strchr(tok, '#');
        size_t idlen = hash ? (size_t)(hash - tok) : strlen(tok);
        if (idlen == 0 || idlen > 8) return -1;
        for (i = 0; i < (int)idlen; i++)
            if (hexval(tok[i]) < 0) return -1;     /* not a candump line */
        if (idlen > 3) return 0;                   /* extended id */
        *id = (uint32_t)strtoul(tok, NULL, 16);

        if (hash) {                                /* candump -L: 391#0011.. */
            const char *h = hash + 1;
            if (*h == '#' || *h == 'R' || *h == 'r') return 0;  /* FD / RTR */
            for (i = 0; h[0] && h[1] && i < 8; h += 2, i++) {
                int a = hexval(h[0]), b = hexval(h[1]);
                if (a < 0 || b < 0) return -1;
                d[i] = (uint8_t)(a << 4 | b);
            }
            if (*h && *h != '\n' && *h != '\r' && *h != ' ') return -1;
            *len = i;
            return 1;
        }
    }

    /* candump default: 391   [8]  00 11 22 ... */
    if (sscanf(p, " [%d]%n", len, &n) != 1) return -1;
    p += n;
    if (*len < 0 || *len > 8) return 0;
    for (i = 0; i < *len; i++) {
        unsigned b;
        if (sscanf(p, " %2x%n", &b, &n) != 1) return -1;   /* truncated */
        d[i] = (uint8_t)b;
        p += n;
    }
    if (strstr(p, "remote"))                       /* "[0]  remote request" */
        return 0;
    return 1;
}
