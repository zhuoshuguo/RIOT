/*
 * Copyright (C) 2016, 2016 Shuguo Zhuo <shuguo.zhuo@inria.fr>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 *
 * @file
 */
#include <string.h>

#include "embUnit.h"

#include "net/gnrc/pkt.h"
#include "net/gnrc/mac/internal.h"

#include "unittests-constants.h"
#include "tests-gnrc_mac_internal.h"

static void set_up(void)
{
    gnrc_pktbuf_init();
}

static void test_gnrc_mac_pktbuf_find(void)
{
    gnrc_pktsnip_t *pkt = gnrc_pktbuf_add(NULL, TEST_STRING4, sizeof(TEST_STRING4),
                                          GNRC_NETTYPE_UNDEF);
    pkt = gnrc_pktbuf_add(pkt, TEST_STRING8, sizeof(TEST_STRING8),
                          GNRC_NETTYPE_NETIF);
    pkt = gnrc_pktbuf_add(pkt, TEST_STRING16, sizeof(TEST_STRING16),
                          GNRC_NETTYPE_IOVEC);

    void* data;
    data = gnrc_mac_pktbuf_find(pkt, GNRC_NETTYPE_IOVEC);
    TEST_ASSERT_EQUAL_STRING(TEST_STRING16, data);

    data = gnrc_mac_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
    TEST_ASSERT_EQUAL_STRING(TEST_STRING8, data);

    data = gnrc_mac_pktbuf_find(pkt, GNRC_NETTYPE_UNDEF);
    TEST_ASSERT_EQUAL_STRING(TEST_STRING4, data);
}

static void test_gnrc_mac_get_dstaddr(void)
{
    gnrc_pktsnip_t *hdr;
    uint8_t dst_addr[2];
    uint8_t* add;
    int addr_len = 0;

    dst_addr[0] = 0x76;
    dst_addr[1] = 0xb6;

    hdr = gnrc_netif_hdr_build(NULL, 0, dst_addr, 2);
    gnrc_pktsnip_t *pkt = gnrc_pktbuf_add(NULL, TEST_STRING4, sizeof(TEST_STRING4),
                                          GNRC_NETTYPE_UNDEF);
    LL_APPEND(hdr, pkt);
    pkt = hdr;

    addr_len = gnrc_mac_get_dstaddr(pkt,&add);

    TEST_ASSERT(addr_len == 2);
    TEST_ASSERT_EQUAL_STRING(TEST_STRING4, pkt->next->data);
    TEST_ASSERT(0 == memcmp(dst_addr,add,addr_len));
}

static void test_gnrc_mac_chk_pkt_bcast(void)
{
    gnrc_pktsnip_t *hdr;
    gnrc_netif_hdr_t* netif_hdr;

    hdr = gnrc_netif_hdr_build(NULL, 0, NULL, 0);
    gnrc_pktsnip_t *pkt = gnrc_pktbuf_add(NULL, TEST_STRING4, sizeof(TEST_STRING4),
                                          GNRC_NETTYPE_UNDEF);
    LL_APPEND(hdr, pkt);
    pkt = hdr;

    netif_hdr = gnrc_mac_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
    netif_hdr->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

    TEST_ASSERT(netif_hdr == pkt->data);
    TEST_ASSERT_EQUAL_STRING(TEST_STRING4, pkt->next->data);
    TEST_ASSERT(gnrc_mac_chk_pkt_bcast(pkt));

    netif_hdr->flags &= ~GNRC_NETIF_HDR_FLAGS_BROADCAST;
    TEST_ASSERT(false == gnrc_mac_chk_pkt_bcast(pkt));
}

static void test_gnrc_mac_addr_match(void)
{
    uint8_t addr1[8];
    uint8_t addr2[8];

    uint64_t add = 0x0123456789abcdef;

    memcpy(addr1, &add, 8);
    memcpy(addr2, &add, 8);

    TEST_ASSERT(gnrc_mac_addr_match(addr1,addr2,2));
    TEST_ASSERT(gnrc_mac_addr_match(addr1,addr2,5));
    TEST_ASSERT(gnrc_mac_addr_match(addr1,addr2,8));

    add = 0xfedcba9876543210;
    memcpy(addr2, &add, 8);

    TEST_ASSERT(false == gnrc_mac_addr_match(addr1,addr2,2));
    TEST_ASSERT(false == gnrc_mac_addr_match(addr1,addr2,5));
    TEST_ASSERT(false == gnrc_mac_addr_match(addr1,addr2,8));
}

Test *tests_gnrc_mac_internal_tests(void)
{
    EMB_UNIT_TESTFIXTURES(fixtures) {
        new_TestFixture(test_gnrc_mac_pktbuf_find),
        new_TestFixture(test_gnrc_mac_get_dstaddr),
        new_TestFixture(test_gnrc_mac_chk_pkt_bcast),
        new_TestFixture(test_gnrc_mac_addr_match),
    };

    EMB_UNIT_TESTCALLER(gnrc_mac_internal_tests, set_up, NULL, fixtures);

    return (Test *)&gnrc_mac_internal_tests;
}

void tests_gnrc_mac_internal(void)
{
    TESTS_RUN(tests_gnrc_mac_internal_tests());
}
/** @} */
