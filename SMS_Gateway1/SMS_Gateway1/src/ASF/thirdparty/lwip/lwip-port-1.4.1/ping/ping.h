/*
 * ping.h
 *
 * Created: 02.12.2016 11:22:55
 *  Author: sschies1
 */ 


#ifndef PING_H_
#define PING_H_

/**
 * PING_USE_SOCKETS: Set to 1 to use sockets, otherwise the raw api is used
 */
#ifndef PING_USE_SOCKETS
#define PING_USE_SOCKETS    LWIP_SOCKET
#endif


void ping_init(void);

#if !PING_USE_SOCKETS
void ping_send_now(void);
#endif /* !PING_USE_SOCKETS */

#endif /* PING_H_ */