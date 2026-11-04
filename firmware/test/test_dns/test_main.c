#include <string.h>

#include <unity.h>

#include "dns_packet.h"

void setUp(void) {}
void tearDown(void) {}

static const uint32_t IP = 0x0104A8C0; /* 192.168.4.1 in network order */

/* Query for connectivitycheck.gstatic.com, type qtype, plus an EDNS record. */
static int make_query(uint8_t *b, uint16_t qtype)
{
    const uint8_t hdr[] = { 0x12, 0x34, 0x01, 0x00, 0, 1, 0, 0, 0, 0, 0, 1 };
    memcpy(b, hdr, sizeof(hdr));
    int o = sizeof(hdr);
    const char *labels[] = { "connectivitycheck", "gstatic", "com" };
    for (int i = 0; i < 3; i++) {
        b[o++] = (uint8_t)strlen(labels[i]);
        memcpy(b + o, labels[i], strlen(labels[i]));
        o += (int)strlen(labels[i]);
    }
    b[o++] = 0;
    b[o++] = (uint8_t)(qtype >> 8);
    b[o++] = (uint8_t)qtype;
    b[o++] = 0;
    b[o++] = 1;
    const uint8_t opt[] = { 0, 0, 41, 0x10, 0, 0, 0, 0, 0, 0, 0 };
    memcpy(b + o, opt, sizeof(opt));
    return o + (int)sizeof(opt);
}

static void test_a_query_gets_ap_address(void)
{
    uint8_t b[512];
    int len = make_query(b, 1);
    int qlen = len - 11; /* question ends before the EDNS record */
    int out = dns_build_reply(b, len, sizeof(b), IP);
    TEST_ASSERT_EQUAL(qlen + 16, out);
    TEST_ASSERT_EQUAL_HEX8(0x12, b[0]);   /* id kept */
    TEST_ASSERT_EQUAL_HEX8(0x85, b[2]);   /* response, AA, RD kept */
    TEST_ASSERT_EQUAL(1, b[7]);           /* one answer */
    TEST_ASSERT_EQUAL(0, b[11]);          /* EDNS dropped */
    TEST_ASSERT_EQUAL_HEX8(0xC0, b[qlen]); /* name pointer */
    TEST_ASSERT_EQUAL_UINT8_ARRAY(((const uint8_t[]){ 192, 168, 4, 1 }),
                                  b + out - 4, 4);
}

static void test_aaaa_gets_empty_answer(void)
{
    uint8_t b[512];
    int len = make_query(b, 28);
    int out = dns_build_reply(b, len, sizeof(b), IP);
    TEST_ASSERT_EQUAL(len - 11, out);
    TEST_ASSERT_EQUAL(0, b[7]);
    TEST_ASSERT_EQUAL_HEX8(0x80, b[3]); /* no error, so clients do not retry */
}

static void test_garbage_ignored(void)
{
    uint8_t b[512];
    TEST_ASSERT_EQUAL(-1, dns_build_reply(b, 5, sizeof(b), IP));

    int len = make_query(b, 1);
    b[2] |= 0x80; /* already a response */
    TEST_ASSERT_EQUAL(-1, dns_build_reply(b, len, sizeof(b), IP));

    len = make_query(b, 1);
    b[12] = 60; /* label runs past the end */
    TEST_ASSERT_EQUAL(-1, dns_build_reply(b, 20, sizeof(b), IP));

    len = make_query(b, 1);
    b[5] = 2; /* two questions */
    TEST_ASSERT_EQUAL(-1, dns_build_reply(b, len, sizeof(b), IP));

    len = make_query(b, 1);
    b[12] = 0x80; /* reserved label type */
    TEST_ASSERT_EQUAL(-1, dns_build_reply(b, len, sizeof(b), IP));
}

static void test_no_room_for_answer(void)
{
    uint8_t b[512];
    int len = make_query(b, 1);
    int qlen = len - 11;
    /* buffer exactly as long as the query: reply without answer */
    TEST_ASSERT_EQUAL(qlen, dns_build_reply(b, len, len, IP));
    TEST_ASSERT_EQUAL(0, b[7]);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_a_query_gets_ap_address);
    RUN_TEST(test_aaaa_gets_empty_answer);
    RUN_TEST(test_garbage_ignored);
    RUN_TEST(test_no_room_for_answer);
    return UNITY_END();
}
