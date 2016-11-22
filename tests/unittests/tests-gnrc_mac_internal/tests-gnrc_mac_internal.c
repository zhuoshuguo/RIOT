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

    netif_hdr = (gnrc_netif_hdr_t*) gnrc_mac_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
    gnrc_netif_hdr_init(netif_hdr, 0, 2);
    gnrc_netif_hdr_set_dst_addr(netif_hdr, dst_addr, 2);

    addr_len2 = gnrc_mac_get_dstaddr(pkt,&add);

    TEST_ASSERT(netif_hdr == pkt->data);
    TEST_ASSERT(addr_len2 == 2);
    TEST_ASSERT_EQUAL_STRING(TEST_STRING4, pkt->next->data);
    TEST_ASSERT(add[0] == 0x76);
    TEST_ASSERT(add[1] == 0xb6);
}

static void test_gnrc_mac_chk_pkt_bcast(void)
{
    gnrc_netif_hdr_t* netif_hdr;
    uint8_t src_addr[2];
    gnrc_pktsnip_t *pkt = gnrc_pktbuf_add(NULL, TEST_STRING4, sizeof(TEST_STRING4),
                                          GNRC_NETTYPE_UNDEF);
    pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t) + 2,
                          GNRC_NETTYPE_NETIF);
    src_addr[0] = 0xf3;
    src_addr[1] = 0xb6;

    netif_hdr = (gnrc_netif_hdr_t*) gnrc_mac_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
    gnrc_netif_hdr_init(netif_hdr, 2, 0);
    gnrc_netif_hdr_set_src_addr(netif_hdr, src_addr, 2);

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

    TEST_ASSERT(false ==gnrc_mac_addr_match(addr1,addr2,2));
    TEST_ASSERT(false ==gnrc_mac_addr_match(addr1,addr2,5));
    TEST_ASSERT(false ==gnrc_mac_addr_match(addr1,addr2,8));
}

static void test_gnrc_mac_queue_tx_packet(void)
{
	gnrc_mac_tx_t tx;
    gnrc_netif_hdr_t* netif_hdr;
    uint8_t dst_addr[2];
    bool push_success;
    ////////////////////
    gnrc_pktsnip_t *pkt = gnrc_pktbuf_add(NULL, TEST_STRING4, sizeof(TEST_STRING4),
                                          GNRC_NETTYPE_UNDEF);
    pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t) + 2,
                          GNRC_NETTYPE_NETIF);
    dst_addr[0] = 0x76;
    dst_addr[1] = 0xb6;
    netif_hdr = (gnrc_netif_hdr_t*) gnrc_mac_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
    gnrc_netif_hdr_init(netif_hdr, 0, 2);
    gnrc_netif_hdr_set_dst_addr(netif_hdr, dst_addr, 2);

    //////////////////
    gnrc_pktsnip_t *pkt2 = gnrc_pktbuf_add(NULL, TEST_STRING8, sizeof(TEST_STRING8),
                                          GNRC_NETTYPE_UNDEF);
    pkt2 = gnrc_pktbuf_add(pkt2, NULL, sizeof(gnrc_netif_hdr_t) + 2,
                           GNRC_NETTYPE_NETIF);

    netif_hdr = (gnrc_netif_hdr_t*) gnrc_mac_pktbuf_find(pkt2, GNRC_NETTYPE_NETIF);
    gnrc_netif_hdr_init(netif_hdr, 0, 2);
    gnrc_netif_hdr_set_dst_addr(netif_hdr, dst_addr, 2);

    /////////////////
    dst_addr[0] = 0x44;
    dst_addr[1] = 0x7e;
    gnrc_pktsnip_t *pkt3 = gnrc_pktbuf_add(NULL, TEST_STRING16, sizeof(TEST_STRING16),
                                           GNRC_NETTYPE_UNDEF);
    pkt3 = gnrc_pktbuf_add(pkt3, NULL, sizeof(gnrc_netif_hdr_t) + 2,
                          GNRC_NETTYPE_NETIF);

    netif_hdr = (gnrc_netif_hdr_t*) gnrc_mac_pktbuf_find(pkt3, GNRC_NETTYPE_NETIF);
    gnrc_netif_hdr_init(netif_hdr, 0, 2);
    gnrc_netif_hdr_set_dst_addr(netif_hdr, dst_addr, 2);


    for (size_t i = 0; i < GNRC_MAC_TX_QUEUE_SIZE; i++) {
        tx._queue_nodes[i].pkt = NULL;
        tx._queue_nodes[i].next = NULL;
    }

    for(int i = 0; i <= (signed)GNRC_MAC_NEIGHBOR_COUNT; i++) {
    	tx.neighbors[i].l2_addr_len = 0;
        gnrc_priority_pktqueue_init(&(tx.neighbors[i].queue));
    }

    push_success = gnrc_mac_queue_tx_packet(&tx,1,pkt);

    gnrc_pktsnip_t *pkt_head;
    pkt_head = gnrc_priority_pktqueue_head(&tx.neighbors[0].queue);

    TEST_ASSERT(push_success);
    TEST_ASSERT(pkt_head == pkt);
    TEST_ASSERT(1 == gnrc_priority_pktqueue_length(&tx.neighbors[0].queue));
    TEST_ASSERT_EQUAL_STRING(TEST_STRING4, pkt_head->next->data);

    push_success = gnrc_mac_queue_tx_packet(&tx,0,pkt2);
    pkt_head = gnrc_priority_pktqueue_head(&tx.neighbors[0].queue);

    TEST_ASSERT(push_success);
    TEST_ASSERT(pkt_head == pkt2);
    TEST_ASSERT(2 == gnrc_priority_pktqueue_length(&tx.neighbors[0].queue));
    TEST_ASSERT_EQUAL_STRING(TEST_STRING8, pkt_head->next->data);

    pkt_head = gnrc_priority_pktqueue_pop(&tx.neighbors[0].queue);
    TEST_ASSERT(pkt_head == pkt2);
    TEST_ASSERT(1 == gnrc_priority_pktqueue_length(&tx.neighbors[0].queue));
    TEST_ASSERT_EQUAL_STRING(TEST_STRING8, pkt_head->next->data);

    pkt_head = gnrc_priority_pktqueue_head(&tx.neighbors[0].queue);
    TEST_ASSERT(pkt_head == pkt);
    TEST_ASSERT_EQUAL_STRING(TEST_STRING4, pkt_head->next->data);

    push_success = gnrc_mac_queue_tx_packet(&tx,0,pkt3);
    pkt_head = gnrc_priority_pktqueue_head(&tx.neighbors[1].queue);
    TEST_ASSERT(push_success);
    TEST_ASSERT(pkt_head == pkt3);
    TEST_ASSERT(1 == gnrc_priority_pktqueue_length(&tx.neighbors[1].queue));
    TEST_ASSERT_EQUAL_STRING(TEST_STRING16, pkt_head->next->data);
}

Test *tests_gnrc_mac_internal_tests(void)
{
    EMB_UNIT_TESTFIXTURES(fixtures) {
        new_TestFixture(test_gnrc_mac_pktbuf_find),
        new_TestFixture(test_gnrc_mac_get_dstaddr),
        new_TestFixture(test_gnrc_mac_chk_pkt_bcast),
        new_TestFixture(test_gnrc_mac_addr_match),
        new_TestFixture(test_gnrc_mac_queue_tx_packet),
    };

    EMB_UNIT_TESTCALLER(gnrc_mac_internal_tests, set_up, NULL, fixtures);

    return (Test *)&gnrc_mac_internal_tests;
}

void tests_gnrc_mac_internal(void)
{
    TESTS_RUN(tests_gnrc_mac_internal_tests());
}
/** @} */
