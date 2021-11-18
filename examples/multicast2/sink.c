/*
 * Copyright (c) 2010, Loughborough University - Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *         This node is part of the RPL multicast example. It is a node that
 *         joins a multicast group and listens for messages. It also knows how
 *         to forward messages down the tree.
 *         For the example to work, we need one or more of those nodes.
 *
 * \author
 *         George Oikonomou - <oikonomou@users.sourceforge.net>
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ipv6/multicast/uip-mcast6.h"
#include "net/ipv6/simple-udp.h"
#include "node-id.h"

// 11 Nov. 2021
#include "dev/leds.h"
#include "ledcmd.h"

#include <string.h>

#define DEBUG DEBUG_PRINT
#include "net/ipv6/uip-debug.h"

#include "sys/log.h"

#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

#define MCAST_SINK_UDP_PORT 3001 /* Host byte order */
//#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

static struct simple_udp_connection udp_conn;

//static struct uip_udp_conn *sink_conn;
static uint16_t count;

#if !NETSTACK_CONF_WITH_IPV6 || !UIP_CONF_ROUTER || !UIP_IPV6_MULTICAST || !UIP_CONF_IPV6_RPL
#error "This example can not work with the current contiki configuration"
#error "Check the values of: NETSTACK_CONF_WITH_IPV6, UIP_CONF_ROUTER, UIP_CONF_IPV6_RPL"
#endif
/*---------------------------------------------------------------------------*/
PROCESS(mcast_sink_process, "Multicast Sink");
AUTOSTART_PROCESSES(&mcast_sink_process);
/*---------------------------------------------------------------------------*/
/*
static void
tcpip_handler(void) // データを受信したときに動くところ
{
  if(uip_newdata()) {
    count++;
    PRINTF("In: [0x%08lx], TTL %u, total %u\n",
        (unsigned long)uip_ntohl((unsigned long) *((uint32_t *)(uip_appdata))),
        UIP_IP_BUF->ttl, count);
    // 返信処理を入れる

  }
  return;
}
*/
/*---------------------------------------------------------------------------*/
#if UIP_MCAST6_CONF_ENGINE != UIP_MCAST6_ENGINE_MPL
static uip_ds6_maddr_t *
join_mcast_group(void)
{
  uip_ipaddr_t addr;
  uip_ds6_maddr_t *rv;
  const uip_ipaddr_t *default_prefix = uip_ds6_default_prefix();

  /* First, set our v6 global */
  uip_ip6addr_copy(&addr, default_prefix);
  uip_ds6_set_addr_iid(&addr, &uip_lladdr);
  uip_ds6_addr_add(&addr, 0, ADDR_AUTOCONF);

  /*
   * IPHC will use stateless multicast compression for this destination
   * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD)
   */
  if (node_id % 2 == 0)
    uip_ip6addr(&addr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);
  else
    uip_ip6addr(&addr, 0xFF1E,0,0,0,0,0,0x89,0xABCE);
  rv = uip_ds6_maddr_add(&addr);

  if(rv) {
    PRINTF("Joined multicast group ");
    PRINT6ADDR(&uip_ds6_maddr_lookup(&addr)->ipaddr);
    PRINTF("\n");
  }
  return rv;
}
#endif
/*---------------------------------------------------------------------------*/
static void
udp_rx_callback(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  int cmd = uip_ntohl((unsigned long) *((uint32_t *)(data)));
  switch (cmd) {
    case LED_R_ON: leds_single_on(2); break;
    case LED_R_OFF: leds_single_off(2); break;
    case LED_B_ON: leds_single_on(1); break;
    case LED_B_OFF: leds_single_off(1); break;
  }
  LOG_INFO("led %02x \n", leds_get());

  //LOG_INFO("Received request '%.*s' from ", //datalen, (char *) data);
  LOG_INFO("Received request '%08lx' from ", uip_ntohl((unsigned long) *((uint32_t *)(data))));
  LOG_INFO_6ADDR(sender_addr);
  LOG_INFO(" port '%d'\n", uip_ntohs(sender_port));
  /* send back the same string to the client as an echo reply */
  LOG_INFO("Sending response to ");
  LOG_INFO_6ADDR(sender_addr);
  LOG_INFO("%d\n", uip_ntohs(sender_port));
  simple_udp_sendto_port(&udp_conn, data, datalen, sender_addr, uip_ntohs(sender_port));
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mcast_sink_process, ev, data)
{
  PROCESS_BEGIN();

  PRINTF("Multicast Engine: '%s'\n", UIP_MCAST6.name);

  /*
   * MPL nodes are automatically configured to subscribe to the ALL_MPL_FORWARDERS
   *  well-known address, so this isn't needed.
   */
#if UIP_MCAST6_CONF_ENGINE != UIP_MCAST6_ENGINE_MPL
  if(join_mcast_group() == NULL) {
    PRINTF("Failed to join multicast group\n");
    PROCESS_EXIT();
  }
#endif

  leds_init();

  count = 0;

  /* Initialize UDP connection */
  simple_udp_register(&udp_conn, UDP_SERVER_PORT, NULL,
                    UIP_HTONS(0), udp_rx_callback);

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
