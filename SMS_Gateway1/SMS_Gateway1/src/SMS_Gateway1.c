/**
 * \file
 *
 * \brief lwIP Raw HTTP basic example.
 *
 * Copyright (c) 2012-2015 Atmel Corporation. All rights reserved.
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

/**
 *  \mainpage lwIP Raw HTTP basic example
 *
 *  \section Purpose
 *  This documents data structures, functions, variables, defines, enums, and
 *  typedefs in the software for the lwIP Raw HTTP basic example.
 *
 *  The given example is a lwIP example using the current lwIP stack and MAC driver.
 *
 *  \section Requirements
 *
 *  This package can be used with SAM3X-EK,SAM4E-EK,SAMV71 and SAME70.
 *
 *  \section Description
 *
 *  This example features a simple lwIP web server.
 *  - Plug the Ethernet cable directly into the evaluation kit to connect to the PC.
 *  - Configuring the PC network port to local mode to setup a 'point to point' network.
 *  - Start the example.
 *  - Launch your favorite web browser.
 *  - Type the WEB server example IP address in your browser's address bar.
 *
 *  \section Usage
 *
 *  -# Build the program and download it into the evaluation board. Please
 *     refer to the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/6421B.pdf">
 *     SAM-BA User Guide</a>, the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *     GNU-Based Software Development</a>
 *     application note or the
 *     <a href="http://www.iar.com/website1/1.0.1.0/78/1/">
 *     IAR EWARM User and reference guides</a>,
 *     depending on the solutions that users choose.
 *  -# On the computer, open and configure a terminal application
 *     (e.g., HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# In the terminal window, the
 *     following text should appear (if DHCP mode is not enabled):
 *     \code
 *      Network up IP==xxx.xxx.xxx.xxx
 *      Static IP Address Assigned
 *     \endcode
 *
 */

#include <string.h>
#include "sysclk.h"
#include "ioport.h"
#include "stdio_serial.h"
#include "ethernet.h"
//#include "httpd.h"
#include "udp.h"

#define OSC_HEADER      ("/sabre/pressure\00,ii")
#define OSC_HEADER_LEN  sizeof(OSC_HEADER)
#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- Raw HTTP Basic Example --"STRING_EOL \
		"-- "BOARD_NAME" --"STRING_EOL \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/**
 *  \brief Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#endif
}

static void udp_sender(struct udp_pcb *pcb)
{
    struct ip_addr to_ip;
    char msg[100] = OSC_HEADER;
    struct pbuf *pb;
    err_t err;
    
    IP4_ADDR(&to_ip, 169, 254, 14, 49);
    
    for(uint8_t i = 0; i < 40; i++) {
        msg[OSC_HEADER_LEN + i] = i;
    }
    msg[OSC_HEADER_LEN + 40] = "\Q";
    pb = pbuf_alloc(PBUF_TRANSPORT, 100, PBUF_RAM);
    memcpy(pb->payload, msg, 100);
    
    printf("Sending to %d.%d.%d.%d on port 11999: ", (to_ip.addr & 0xff), ((to_ip.addr >> 8) & 0xff), ((to_ip.addr >> 16) & 0xff), ((to_ip.addr >> 24) & 0xff));
    for(uint8_t i = 0; i < sizeof(msg); i++) {
        printf("%c", msg[i]);
    }
    printf("\n\r");
    
    err = udp_sendto(pcb, pb, &to_ip, 11999);
    if(err == ERR_MEM) {
        printf("ERROR: out of memory\n\r");
    }
    else if(err == ERR_RTE) {
        printf("ERROR: route not found\n\r");
    }
    else if(err != ERR_OK) {
        printf("ERROR: %d\n\r", err);
    }
    else {
        printf("Packet sent!\n\r");
    }
    
    pbuf_free(pb);
}

static void udp_receiver(void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip_addr *addr, u16_t port) {
    LWIP_UNUSED_ARG(arg);
    if(p == NULL) {
        return;
    }
    printf("Sending back to %d.%d.%d.%d on port 11999\n\r", ((addr->addr >> 0) & 0xff), ((addr->addr >> 8) & 0xff), ((addr->addr >> 16) & 0xff), ((addr->addr >> 24) & 0xff));
    udp_sendto(pcb, p, addr, 11999);
    pbuf_free(p);
}

/**
 * \brief Main program function. Configure the hardware, initialize lwIP
 * TCP/IP stack, and start HTTP service.
 */
int main(void)
{
	/* Initialize the SAM system. */
	sysclk_init();
	board_init();

	/* Configure debug UART */
	configure_console();

	/* Print example information. */
	puts(STRING_HEADER);

	/* Bring up the ethernet interface & initialize timer0, channel0. */
	init_ethernet();

    struct ip_addr remote_ip;
    struct udp_pcb *pcb;
    err_t err;
    char msg[100] = OSC_HEADER;
    struct pbuf *pb;
    
    IP4_ADDR(&remote_ip, 169, 254, 14, 49);
    

    pcb = udp_new();
    //if((err = udp_bind(pcb, IP_ADDR_ANY, 11998)) != ERR_OK) {
        //printf("Failed to bind to port 11998\n\r");
    //}
    //else {
        //printf("UDP bound to port 11998\n\r");
    //}
    if((err = udp_connect(pcb, &remote_ip, 11999)) != ERR_OK) {
        printf("Failed to connect to port 11999\n\r");
    }
    else {
        printf("UDP connected to port 11999\n\r");
    }
    //if((err = udp_bind(pcb, IP_ADDR_ANY, 6667)) != ERR_OK) {
        //printf("Failed to bind to port 6667\n\r");
    //}
    //else {
        //printf("UDP bound to port 6667\n\r");
    //}
    udp_recv(pcb, udp_receiver, pcb);

	///* Bring up the web server. */
	//httpd_init();

	/* Program main loop. */
	while (1) {
    	/* Check for input packet and process it. */
    	ethernet_task();
    	
        ////////////////////////////////////////////////
        static uint32_t i1 = 0x11111000;
        static uint32_t i2 = 0x88888000;
        for(uint8_t i = 0; i < 4; i++) {
            msg[OSC_HEADER_LEN + i] = (uint8_t)((i1 >> ((3-i)*8)) & 0xff);
        }
        for(uint8_t i = 0; i < 4; i++) {
            msg[OSC_HEADER_LEN + 4 + i] = (uint8_t)((i2 >> ((3-i)*8)) & 0xff);
        }
        msg[OSC_HEADER_LEN + 8] = 'Q';
        i1++;
        i2++;
        
    	pb = pbuf_alloc(PBUF_TRANSPORT, 100, PBUF_POOL);
    	memcpy(pb->payload, msg, 100);
    	
    	printf("Sending to %d.%d.%d.%d on port 11999: ", (remote_ip.addr & 0xff), ((remote_ip.addr >> 8) & 0xff), ((remote_ip.addr >> 16) & 0xff), ((remote_ip.addr >> 24) & 0xff));
    	for(uint8_t i = 0; i < sizeof(msg); i++) {
        	printf("%c", msg[i]);
    	}
    	printf("\n\r");
    	
    	err = udp_send(pcb, pb);
        
    	if(err == ERR_MEM) {
        	printf("ERROR: out of memory\n\r");
    	}
    	else if(err == ERR_RTE) {
        	printf("ERROR: route not found\n\r");
    	}
    	else if(err != ERR_OK) {
        	printf("ERROR: %d\n\r", err);
    	}
    	else {
        	printf("Packet sent!\n\r");
    	}
    	
    	pbuf_free(pb);
        
    	uint32_t i = 10000000;
    	while(i > 0) {
        	i--;
    	};
	}
}
