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
#include <stdlib.h>
#include "sysclk.h"
#include "ioport.h"
#include "stdio_serial.h"
#include "ethernet.h"
#include "netif.h"
#include "netif/etharp.h"
//#include "httpd.h"
#include "udp.h"
#include "ping.h"
#include "conf_OSC.h"
#include "OSCMessage.h"
#include "spi.h"
#include "conf_clock.h"
#include "conf_board.h"
#include "conf_spi_example.h"

#define OSC_HEADER      ("/sabre/pressure\00,ii")
#define OSC_HEADER_LEN  sizeof(OSC_HEADER)
#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- Raw HTTP Basic Example --"STRING_EOL \
"-- "BOARD_NAME" --"STRING_EOL \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/* Chip select. */
#define SPI_CHIP_SEL 0
#define SPI_CHIP_PCS spi_get_pcs(SPI_CHIP_SEL)
/* Clock polarity. */
#define SPI_CLK_POLARITY 0
/* Clock phase. */
#define SPI_CLK_PHASE 1
/* Delay before SPCK. */
#define SPI_DLYBS 0x40
/* Delay between consecutive transfers. */
#define SPI_DLYBCT 0x10
/* SPI slave states for this example. */
#define SLAVE_STATE_IDLE           0
#define SLAVE_STATE_TEST           1
#define SLAVE_STATE_DATA           2
#define SLAVE_STATE_STATUS_ENTRY   3
#define SLAVE_STATE_STATUS         4
#define SLAVE_STATE_END            5
/* SPI example commands for this example. */
/* slave test state, begin to return RC_RDY. */
#define CMD_TEST     0x10101010
/* Slave data state, begin to return last data block. */
#define CMD_DATA     0x29380000
/* Slave status state, begin to return RC_RDY + RC_STATUS. */
#define CMD_STATUS   0x68390384
/* Slave idle state, begin to return RC_SYN. */
#define CMD_END      0x68390484
/* General return value. */
#define RC_SYN       0x55AA55AA
/* Ready status. */
#define RC_RDY       0x12345678
/* Slave data mask. */
#define CMD_DATA_MSK 0xFFFF0000
/* Slave data block mask. */
#define DATA_BLOCK_MSK 0x0000FFFF
/* SPI Communicate buffer size. */
#define COMM_BUFFER_SIZE   64
/* Number of commands logged in status. */
#define NB_STATUS_CMD   20

OSCPacketStream osc_stream;
struct eth_addr remote_eth;
struct ip_addr remote_ip, broadcast_ip;
struct udp_pcb *pcb;
char msg[SMS_OSC_MSG_MAX_LEN];
/* Status block. */
struct status_block_t {
	/** Number of data blocks. */
	uint32_t ul_total_block_number;
	/** Number of SPI commands (including data blocks). */
	uint32_t ul_total_command_number;
	/** Command list. */
	uint32_t ul_cmd_list[NB_STATUS_CMD];
};

/* SPI clock setting (Hz). */
static uint32_t gs_ul_spi_clock = 500000;
/* Current SPI return code. */
static uint32_t gs_ul_spi_cmd = 0x11AABBCC; //RC_SYN;
/* Current SPI state. */
static uint32_t gs_ul_spi_state = 0;
/* 64 bytes data buffer for SPI transfer and receive. */
static uint8_t gs_uc_spi_buffer[COMM_BUFFER_SIZE];
/* Pointer to transfer buffer. */
static uint8_t *gs_puc_transfer_buffer;
/* Transfer buffer index. */
static uint32_t gs_ul_transfer_index;
/* Transfer buffer length. */
static uint32_t gs_ul_transfer_length;
/* SPI Status. */
static struct status_block_t gs_spi_status;
static uint32_t gs_ul_test_block_number;

bool udp_forward = false;
//static uint8_t my_spi_buffer[COMM_BUFFER_SIZE];

extern struct netif gs_net_if;

void osc_write(uint8_t *buf, uint32_t size) {
	err_t err;
	struct pbuf *pb = pbuf_alloc(PBUF_TRANSPORT, size, PBUF_RAM);
	memcpy(pb->payload, buf, size);
	
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
}

void osc_read(uint8_t *buf, uint32_t size) {
	printf("LALALA\n\r");
}

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

static void init_udp(void) {
	//TODO LATER: ping the 169.254.x.x range to find the communication partner
	IP4_ADDR(&remote_ip, 169, 254, 19, 40);

	pcb = udp_new();

	err_t err = ERR_CONN;
	uint32_t snd_port = SMS_UDP_SEND_PORT1;
	while(err != ERR_OK) {
		if((err = udp_connect(pcb, &remote_ip, snd_port)) != ERR_OK) {
			printf("Failed to connect to port %ld\n\r", snd_port);
			snd_port++;
		}
		else {
			printf("UDP connected to port %ld\n\r", snd_port);
		}
	}

}

static void init_osc(void) {
	osc_stream.writePacket = &osc_write;
	osc_stream.readPacket = &osc_read;
}


/**
 * \brief Set SPI slave transfer.
 *
 * \param p_buf Pointer to buffer to transfer.
 * \param size Size of the buffer.
 */
static void spi_slave_transfer(void *p_buf, uint32_t size)
{
	gs_puc_transfer_buffer = p_buf;
	gs_ul_transfer_length = size;
	gs_ul_transfer_index = 0;
	spi_write(SPI_SLAVE_BASE, gs_puc_transfer_buffer[gs_ul_transfer_index], 0, 0);
}

/**
 * \brief  SPI command block process.
 */
static void spi_slave_command_process(void)
{
	if (gs_ul_spi_cmd == CMD_END) {
		gs_ul_spi_state = SLAVE_STATE_IDLE;
		gs_spi_status.ul_total_block_number = 0;
		gs_spi_status.ul_total_command_number = 0;
	} else {
		switch (gs_ul_spi_state) {
		case SLAVE_STATE_IDLE:
			/* Only CMD_TEST accepted. */
			if (gs_ul_spi_cmd == CMD_TEST) {
				gs_ul_spi_state = SLAVE_STATE_TEST;
			}
			break;

		case SLAVE_STATE_TEST:
			/* Only CMD_DATA accepted. */
			if ((gs_ul_spi_cmd & CMD_DATA_MSK) == CMD_DATA) {
				gs_ul_spi_state = SLAVE_STATE_DATA;
			}
			gs_ul_test_block_number = gs_ul_spi_cmd & DATA_BLOCK_MSK;
			break;

		case SLAVE_STATE_DATA:
			gs_spi_status.ul_total_block_number++;

			if (gs_spi_status.ul_total_block_number == 
					gs_ul_test_block_number) {
				gs_ul_spi_state = SLAVE_STATE_STATUS_ENTRY;
			}
			break;

		case SLAVE_STATE_STATUS_ENTRY:
			gs_ul_spi_state = SLAVE_STATE_STATUS;
			break;

		case SLAVE_STATE_END:
			break;
		}
	}
}

/**
 * \brief  Start waiting new command.
 */
static void spi_slave_new_command(void)
{
	switch (gs_ul_spi_state) {
	case SLAVE_STATE_IDLE:
	case SLAVE_STATE_END:
		gs_ul_spi_cmd = RC_SYN;
		spi_slave_transfer(&gs_ul_spi_cmd, sizeof(gs_ul_spi_cmd));
		break;

	case SLAVE_STATE_TEST:
		gs_ul_spi_cmd = RC_RDY;
		spi_slave_transfer(&gs_ul_spi_cmd, sizeof(gs_ul_spi_cmd));
		break;

	case SLAVE_STATE_DATA:
		if (gs_spi_status.ul_total_block_number < gs_ul_test_block_number) {
			spi_slave_transfer(gs_uc_spi_buffer, COMM_BUFFER_SIZE);
		}
		break;

	case SLAVE_STATE_STATUS_ENTRY:
		gs_ul_spi_cmd = RC_RDY;
		spi_slave_transfer(&gs_ul_spi_cmd, sizeof(gs_ul_spi_cmd));
		gs_ul_spi_state = SLAVE_STATE_STATUS;
		break;

	case SLAVE_STATE_STATUS:
		gs_ul_spi_cmd = RC_SYN;
		spi_slave_transfer(&gs_spi_status, sizeof(struct status_block_t));
		gs_ul_spi_state = SLAVE_STATE_END;
		break;
	}
}

/**
 * \brief Interrupt handler for the SPI slave.
 */
void SPI_Handler(void)
{
	uint32_t new_cmd = 0;
	static uint16_t data;
	uint8_t uc_pcs;
	bool data_ready = false;

	if(spi_read_status(SPI_SLAVE_BASE) & SPI_SR_RDRF) {
		spi_read(SPI_SLAVE_BASE, &data, &uc_pcs);
		gs_puc_transfer_buffer[gs_ul_transfer_index] = data;
		gs_ul_transfer_length--;
		if(gs_ul_transfer_length) {
			spi_write(SPI_SLAVE_BASE, gs_puc_transfer_buffer[gs_ul_transfer_index], 0, 0);
		}
		else {
			udp_forward = true;
			data_ready = true;
			for(uint8_t i = 0; i < COMM_BUFFER_SIZE; i++) {
				printf("%02x ", gs_uc_spi_buffer[i]);
			}
		}
		gs_ul_transfer_index++;
		
		if(data_ready) {
			printf("READING COMPLETE\n\r");
			spi_slave_transfer(gs_uc_spi_buffer, COMM_BUFFER_SIZE);
			data_ready = false;
		}
	}
}

/**
 * \brief Initialize SPI as slave.
 */
static void spi_slave_initialize(void)
{
	uint32_t i;

	/* Reset status */
	gs_spi_status.ul_total_block_number = 0;
	gs_spi_status.ul_total_command_number = 0;
	for (i = 0; i < NB_STATUS_CMD; i++) {
		gs_spi_status.ul_cmd_list[i] = 0;
	}
	gs_ul_spi_state = SLAVE_STATE_IDLE;
	gs_ul_spi_cmd = RC_SYN;

	puts("-I- Initialize SPI as slave \r");
	/* Configure an SPI peripheral. */
	spi_enable_clock(SPI_SLAVE_BASE);
	spi_disable(SPI_SLAVE_BASE);
	spi_reset(SPI_SLAVE_BASE);
	spi_set_slave_mode(SPI_SLAVE_BASE);
	spi_disable_mode_fault_detect(SPI_SLAVE_BASE);
	spi_set_peripheral_chip_select_value(SPI_SLAVE_BASE, SPI_CHIP_PCS);
	spi_set_clock_polarity(SPI_SLAVE_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_SLAVE_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_SLAVE_BASE, SPI_CHIP_SEL, SPI_CSR_BITS_8_BIT);
	spi_enable_interrupt(SPI_SLAVE_BASE, SPI_IER_RDRF);
	spi_enable(SPI_SLAVE_BASE);

	/* Start waiting command. */
	//spi_slave_transfer(&gs_ul_spi_cmd, sizeof(gs_ul_spi_cmd));
	spi_slave_transfer(gs_uc_spi_buffer, COMM_BUFFER_SIZE);
}


static int send_osc_msg(uint8_t node, enum sensor_types sens, uint32_t *values) {
	OSCMessage *msg = OSCMessage_new();	
	switch (sens) {
		case SMS_SENS_ACCEL:
		OSCMessage_setAddress(msg, "sabre/1/accel");
		OSCMessage_addArgument_int32(msg, values[0]);
		OSCMessage_addArgument_int32(msg, values[1]);
		OSCMessage_addArgument_int32(msg, values[2]);
		OSCMessage_addArgument_int32(msg, values[3]);
		break;
		
		case SMS_SENS_GYRO:
		OSCMessage_setAddress(msg, "sabre/1/gyro");
		OSCMessage_addArgument_int32(msg, values[0]);
		OSCMessage_addArgument_int32(msg, values[1]);
		OSCMessage_addArgument_int32(msg, values[2]);
		OSCMessage_addArgument_int32(msg, values[3]);
		break;
		
		case SMS_SENS_PRESS:
		break;
		
		case SMS_SENS_BUTTON:
		break;
		
		case SMS_SENS_QUAT:
		break;
		
		default:
		break;
	}
	
	OSCMessage_sendMessage(msg, &osc_stream);
	OSCMessage_delete(msg);
}


/**
* \brief Main program function. Configure the hardware, initialize lwIP
* TCP/IP stack, and start HTTP service.
*/
int main(void)
{
	uint8_t uc_key;
	
	/* Initialize the SAM system. */
	sysclk_init();
	board_init();

	/* Configure debug UART */
	configure_console();

	/* Print example information. */
	puts(STRING_HEADER);

	/* Bring up the ethernet interface & initialize timer0, channel0. */
	init_ethernet();

	/* Search for a communication partner and setup UDP connection */
	init_udp();
	
	init_osc();
	
	/* Configure SPI interrupts for slave only. */
	NVIC_DisableIRQ(SPI_IRQn);
	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_SetPriority(SPI_IRQn, 0);
	NVIC_EnableIRQ(SPI_IRQn);

	spi_slave_initialize();
	
	while((ip4_addr1_16(&remote_ip) != 169) && (ip4_addr2_16(&remote_ip) != 254)) {
		ethernet_task();
	}
	printf("Remote device found! eth %02x:%02x:%02x:%02x:%02x:%02x, ip %d.%d.%d.%d", remote_eth.addr[0], remote_eth.addr[1], remote_eth.addr[2], remote_eth.addr[3], remote_eth.addr[4], remote_eth.addr[5], ip4_addr1_16(&remote_ip), ip4_addr2_16(&remote_ip), ip4_addr3_16(&remote_ip), ip4_addr4_16(&remote_ip));

	
	/* Program main loop. */
	while (1) {
		//scanf("%c", (char*)&uc_key);
		/* Check for input packet and process it. */
		ethernet_task();
		
		if(udp_forward) {
			OSCMessage *osc_msg = OSCMessage_new();
			OSCMessage_setAddress(osc_msg, "sabre/1");
			//OSCMessage_addArgument_int32(osc_msg, 0x01234567);
			//OSCMessage_addArgument_int32(osc_msg, 0xFEDCBA98);
			uint32_t udp_data[16];
			for(uint8_t i = 0; i < 16; i++) {
				udp_data[i] = ((uint32_t)gs_uc_spi_buffer[4*i] << 24) & 0xff000000;
				udp_data[i] |= ((uint32_t)gs_uc_spi_buffer[(4*i)+1] << 16) & 0xff0000;
				udp_data[i] |= ((uint32_t)gs_uc_spi_buffer[(4*i)+2] << 8) & 0xff00;
				udp_data[i] |= (uint32_t)gs_uc_spi_buffer[(4*i)+3] & 0xff;
				//udp_data[i] = 0x12345670 + i;
				OSCMessage_addArgument_int32(osc_msg, udp_data[i]);
			}
			
			OSCMessage_sendMessage(osc_msg, &osc_stream);
			OSCMessage_delete(osc_msg);
			
			udp_forward = false;
		}
		//////////////////////////////////////////////
		//struct pbuf *pb;
		//static uint32_t ax = 0;
		//static uint32_t ay = 1000;
		//static uint32_t az = 2000;
		//static uint32_t asum = 3000;
		//OSCMessage *osc_msg = OSCMessage_new();
		//OSCMessage_setAddress(osc_msg, "sabre/1");
		//OSCMessage_addArgument_int32(osc_msg, ax);
		//OSCMessage_addArgument_int32(osc_msg, ay);
		//OSCMessage_addArgument_int32(osc_msg, az);
		//OSCMessage_addArgument_int32(osc_msg, asum);
		//OSCMessage_sendMessage(osc_msg, &osc_stream);
		//OSCMessage_delete(osc_msg);
//
		//ax++;
		//ay++;
		//az++;
		//asum++;
//
		//uint32_t i = 5000000;
		//while(i > 0) {
			//i--;
		//};
	}
}
