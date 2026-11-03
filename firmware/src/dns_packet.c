#include "dns_packet.h"

#include <string.h>

/* Skips the QNAME labels; returns the offset after it or -1. */
static int skip_name(const uint8_t *buf, int len, int off)
{
    while (off < len) {
        uint8_t l = buf[off];
        if (l == 0) {
            return off + 1;
        }
        if ((l & 0xC0) == 0xC0) {
            return off + 2 <= len ? off + 2 : -1;
        }
        if (l & 0xC0) {
            return -1; /* reserved label types */
        }
        off += l + 1;
    }
    return -1;
}

int dns_build_reply(uint8_t *buf, int len, int cap, uint32_t ip)
{
    if (len < 12 || len > cap) {
        return -1;
    }
    uint16_t qdcount = (uint16_t)(buf[4] << 8 | buf[5]);
    if ((buf[2] & 0x80) || (buf[2] & 0x78) || qdcount != 1) {
        return -1; /* a response, not a standard query, or several questions */
    }

    int qend = skip_name(buf, len, 12);
    if (qend < 0 || qend + 4 > len) {
        return -1;
    }
    uint16_t qtype = (uint16_t)(buf[qend] << 8 | buf[qend + 1]);
    int out = qend + 4; /* drop anything after the question (EDNS etc.) */

    buf[2] = 0x84 | (buf[2] & 0x01); /* response, authoritative, keep RD */
    buf[3] = 0x80;                    /* RA, no error */
    buf[6] = buf[7] = 0;              /* ANCOUNT */
    buf[8] = buf[9] = 0;              /* NSCOUNT */
    buf[10] = buf[11] = 0;            /* ARCOUNT */

    if (qtype == 1 /* A */ && out + 16 <= cap) {
        const uint8_t answer[] = {
            0xC0, 0x0C,             /* name: pointer to the question */
            0x00, 0x01, 0x00, 0x01, /* type A, class IN */
            0x00, 0x00, 0x00, 0x3C, /* TTL 60 s */
            0x00, 0x04,             /* RDLENGTH */
        };
        memcpy(buf + out, answer, sizeof(answer));
        memcpy(buf + out + sizeof(answer), &ip, 4);
        out += (int)sizeof(answer) + 4;
        buf[7] = 1;
    }
    return out;
}
