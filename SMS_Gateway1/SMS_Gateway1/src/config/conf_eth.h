 /**
 * \file
 *
 * \brief GMAC (Ethernet MAC) driver configuration.
 *
 * Copyright (c) 2013-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef CONF_EMAC_H_INCLUDED
#define CONF_EMAC_H_INCLUDED

#include "gmac.h"

#define SMS_UDP_SEND_PORT1                          11999
#define SMS_UDP_SEND_PORT2                          12000
#define SMS_UDP_SEND_PORT3                          12001
#define SMS_UDP_SEND_PORT4                          12002
#define SMS_UDP_RCV_PORT1                           12999
#define SMS_UDP_RCV_PORT2                           13000
#define SMS_UDP_RCV_PORT3                           13001
#define SMS_UDP_RCV_PORT4                           13002

/**
 * LWIP_NETIF_TX_SINGLE_PBUF: if this is set to 1, lwIP tries to put all data
 * to be sent into one single pbuf. This is for compatibility with DMA-enabled
 * MACs that do not support scatter-gather.
 */
#define LWIP_NETIF_TX_SINGLE_PBUF                     1

/** Number of buffer for RX */
#define GMAC_RX_BUFFERS                               3

/** Number of buffer for TX */
#define GMAC_TX_BUFFERS                               3

/** MAC PHY operation max retry count */
#define MAC_PHY_RETRY_MAX                             1000000

/** MAC address definition.  The MAC address must be unique on the network. */
#define ETHERNET_CONF_ETHADDR0                        0x00
#define ETHERNET_CONF_ETHADDR1                        0x04
#define ETHERNET_CONF_ETHADDR2                        0x25
#define ETHERNET_CONF_ETHADDR3                        0x1C
#define ETHERNET_CONF_ETHADDR4                        0xA0
#define ETHERNET_CONF_ETHADDR5                        0x02

/** The IP address being used. */
#define ETHERNET_CONF_IPADDR0                         169
#define ETHERNET_CONF_IPADDR1                         254
#define ETHERNET_CONF_IPADDR2                         0
#define ETHERNET_CONF_IPADDR3                         100

/** The gateway address being used. */
#define ETHERNET_CONF_GATEWAY_ADDR0                   169
#define ETHERNET_CONF_GATEWAY_ADDR1                   254
#define ETHERNET_CONF_GATEWAY_ADDR2                   0
#define ETHERNET_CONF_GATEWAY_ADDR3                   1

/** The network mask being used. */
#define ETHERNET_CONF_NET_MASK0                       255
#define ETHERNET_CONF_NET_MASK1                       255
#define ETHERNET_CONF_NET_MASK2                       0
#define ETHERNET_CONF_NET_MASK3                       0

/** Ethernet MII/RMII mode */
#define ETH_PHY_MODE                                  GMAC_PHY_MII

#endif /* CONF_EMAC_H_INCLUDED */