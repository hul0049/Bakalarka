#ifndef __ENET_H
#define __ENET_H

#include "lwip/udp.h"

#define DATA_BUF_NUM			4
#define UDP_DATA_LIMIT			1472

//#define configPHY_ADDRESS 1

/* Pit IRQ configuration. */
//#define BOARD_PIT_IRQ_HANDLER PIT0_IRQHandler
//#define BOARD_PIT_IRQ_ID PIT0_IRQn
//#define BOARD_PIT_CHANNEL kPIT_Chnl_0

class Enet
{
public:
	uint32_t m_initialized = 0;
    struct netif m_fsl_netif;
    ip4_addr_t m_if_ipaddr, m_if_netmask, m_if_gw;
    ip4_addr_t m_pc_ipaddr;
    uint16_t m_pc_port;
    udp_pcb *m_pc_pcb;
    pbuf *m_data_pbuf[ DATA_BUF_NUM ];
    uint8_t m_data[ DATA_BUF_NUM ][ UDP_DATA_LIMIT ];
    uint32_t m_data_inx;
    uint32_t m_data_len_limit;

	int init( uint32_t data_len_limit, uint16_t port );
	int send( const void *ptr_data, uint32_t len );
	void check();
};

#endif // __ENET_H
