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

#define PKT_INIT_ELEM(len, data, next, type) \
    { 1, (next), (data), (len), type }
#define PKT_INIT_ELEM_STATIC_DATA(data, next, type) PKT_INIT_ELEM(sizeof(data), data, next, type)
#define PKTQUEUE_INIT_ELEM(pkt) { NULL, pkt }

static void set_up(void)
{
    gnrc_pktbuf_init();
}

static void test_gnrc_pktbuf_find(void)
{
    gnrc_pktsnip_t *pkt = gnrc_pktbuf_add(NULL, TEST_STRING4, sizeof(TEST_STRING4),
                                          GNRC_NETTYPE_UNDEF);
    pkt = gnrc_pktbuf_add(pkt, TEST_STRING8, sizeof(TEST_STRING8),
                          GNRC_NETTYPE_NETIF);
    pkt = gnrc_pktbuf_add(pkt, TEST_STRING16, sizeof(TEST_STRING16),
                          GNRC_NETTYPE_IOVEC);

    void* data;
    data = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_IOVEC);
    TEST_ASSERT_EQUAL_STRING(TEST_STRING16, data);

    data = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
    TEST_ASSERT_EQUAL_STRING(TEST_STRING8, data);

    data = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_UNDEF);
    TEST_ASSERT_EQUAL_STRING(TEST_STRING4, data);
}

static void test_get_dest_address(void)
{
    gnrc_netif_hdr_t* netif_hdr;
    uint8_t dst_addr[2];
    uint8_t* add;
    int addr_len2 = 0;
    gnrc_pktsnip_t *pkt = gnrc_pktbuf_add(NULL, TEST_STRING4, sizeof(TEST_STRING4),
                                          GNRC_NETTYPE_UNDEF);
    pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t) + 2,
                          GNRC_NETTYPE_NETIF);
    dst_addr[0] = 0x76;
    dst_addr[1] = 0xb6;

    netif_hdr = (gnrc_netif_hdr_t*) _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
    gnrc_netif_hdr_init(netif_hdr, 0, 2);
    gnrc_netif_hdr_set_dst_addr(netif_hdr, dst_addr, 2);

    addr_len2 = _get_dest_address(pkt,&add);

    TEST_ASSERT(netif_hdr == pkt->data);
    TEST_ASSERT(addr_len2 == 2);
    TEST_ASSERT_EQUAL_STRING(TEST_STRING4, pkt->next->data);
    TEST_ASSERT(add[0] == 0x76);
    TEST_ASSERT(add[1] == 0xb6);
}


Test *tests_gnrc_mac_internal_tests(void)
{
    EMB_UNIT_TESTFIXTURES(fixtures) {
        new_TestFixture(test_gnrc_pktbuf_find),
        new_TestFixture(test_get_dest_address),
    };

    EMB_UNIT_TESTCALLER(gnrc_mac_internal_tests, set_up, NULL, fixtures);

    return (Test *)&gnrc_mac_internal_tests;
}

void tests_gnrc_mac_internal(void)
{
    TESTS_RUN(tests_gnrc_mac_internal_tests());
}
/** @} */
