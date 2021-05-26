#include <stdio.h>
#include "lwip/opt.h"

extern "C" {
#include "udpecho_raw.h"
#include "fsl_device_registers.h"
}
#include "lwip/timeouts.h"
#include "lwip/init.h"
#include "lwip/udp.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"
#include "fsl_phyksz8081.h"
#include "fsl_enet_mdio.h"

#include "pin_mux.h"
#include "fsl_port.h"
#include "board.h"
#include "enet.h"

// **************************************************************************
// Initialization of pins

/*
- pin_list:
  - {pin_num: E10, peripheral: UART0, signal: RX, pin_signal: PTB16/SPI1_SOUT/UART0_RX/FTM_CLKIN0/FB_AD17/EWM_IN}
  - {pin_num: E9, peripheral: UART0, signal: TX, pin_signal: PTB17/SPI1_SIN/UART0_TX/FTM_CLKIN1/FB_AD16/EWM_OUT_b}
  - {pin_num: A6, peripheral: ENET, signal: 'TMR_1588, 0', pin_signal: PTC16/UART3_RX/ENET0_1588_TMR0/FB_CS5_b/FB_TSIZ1/FB_BE23_16_BLS15_8_b}
  - {pin_num: D5, peripheral: ENET, signal: 'TMR_1588, 1', pin_signal: PTC17/UART3_TX/ENET0_1588_TMR1/FB_CS4_b/FB_TSIZ0/FB_BE31_24_BLS7_0_b}
  - {pin_num: C5, peripheral: ENET, signal: 'TMR_1588, 2', pin_signal: PTC18/UART3_RTS_b/ENET0_1588_TMR2/FB_TBST_b/FB_CS2_b/FB_BE15_8_BLS23_16_b}
  - {pin_num: B5, peripheral: ENET, signal: 'TMR_1588, 3', pin_signal: PTC19/UART3_CTS_b/ENET0_1588_TMR3/FB_CS3_b/FB_BE7_0_BLS31_24_b/FB_TA_b}
  - {pin_num: H9, peripheral: ENET, signal: RMII_MDC, pin_signal: ADC0_SE9/ADC1_SE9/PTB1/I2C0_SDA/FTM1_CH1/RMII0_MDC/MII0_MDC/FTM1_QD_PHB}
  - {pin_num: H10, peripheral: ENET, signal: RMII_MDIO, pin_signal: ADC0_SE8/ADC1_SE8/PTB0/LLWU_P5/I2C0_SCL/FTM1_CH0/RMII0_MDIO/MII0_MDIO/FTM1_QD_PHA, open_drain: enable,
    pull_select: up, pull_enable: enable}
  - {pin_num: K9, peripheral: ENET, signal: RMII_RXD1, pin_signal: CMP2_IN0/PTA12/CAN0_TX/FTM1_CH0/RMII0_RXD1/MII0_RXD1/I2C2_SCL/I2S0_TXD0/FTM1_QD_PHA}
  - {pin_num: J9, peripheral: ENET, signal: RMII_RXD0, pin_signal: CMP2_IN1/PTA13/LLWU_P4/CAN0_RX/FTM1_CH1/RMII0_RXD0/MII0_RXD0/I2C2_SDA/I2S0_TX_FS/FTM1_QD_PHB}
  - {pin_num: L10, peripheral: ENET, signal: RMII_CRS_DV, pin_signal: PTA14/SPI0_PCS0/UART0_TX/RMII0_CRS_DV/MII0_RXDV/I2C2_SCL/I2S0_RX_BCLK/I2S0_TXD1}
  - {pin_num: L11, peripheral: ENET, signal: RMII_TXEN, pin_signal: PTA15/SPI0_SCK/UART0_RX/RMII0_TXEN/MII0_TXEN/I2S0_RXD0}
  - {pin_num: K11, peripheral: ENET, signal: RMII_TXD1, pin_signal: ADC1_SE17/PTA17/SPI0_SIN/UART0_RTS_b/RMII0_TXD1/MII0_TXD1/I2S0_MCLK}
  - {pin_num: K10, peripheral: ENET, signal: RMII_TXD0, pin_signal: PTA16/SPI0_SOUT/UART0_CTS_b/UART0_COL_b/RMII0_TXD0/MII0_TXD0/I2S0_RX_FS/I2S0_RXD1}
  - {pin_num: M8, peripheral: ENET, signal: RMII_RXER, pin_signal: PTA5/USB_CLKIN/FTM0_CH2/RMII0_RXER/MII0_RXER/CMP2_OUT/I2S0_TX_BCLK/JTAG_TRST_b}
  - {pin_num: H12, peripheral: ENET, signal: MII_TXER, pin_signal: PTA28/MII0_TXER/FB_A25}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE PINS TOOL ***
 */

void Enet_InitPins(void) {
//  CLOCK_EnableClock(kCLOCK_PortA);                           /* Port A Clock Gate Control: Clock enabled */
//  CLOCK_EnableClock(kCLOCK_PortB);                           /* Port B Clock Gate Control: Clock enabled */
//  CLOCK_EnableClock(kCLOCK_PortC);                           /* Port C Clock Gate Control: Clock enabled */
//
//  PORT_SetPinMux(PORTA, PIN12_IDX, kPORT_MuxAlt4);           /* PORTA12 (pin K9) is configured as RMII0_RXD1 */
//  PORT_SetPinMux(PORTA, PIN13_IDX, kPORT_MuxAlt4);           /* PORTA13 (pin J9) is configured as RMII0_RXD0 */
//  PORT_SetPinMux(PORTA, PIN14_IDX, kPORT_MuxAlt4);           /* PORTA14 (pin L10) is configured as RMII0_CRS_DV */
//  PORT_SetPinMux(PORTA, PIN15_IDX, kPORT_MuxAlt4);           /* PORTA15 (pin L11) is configured as RMII0_TXEN */
//  PORT_SetPinMux(PORTA, PIN16_IDX, kPORT_MuxAlt4);           /* PORTA16 (pin K10) is configured as RMII0_TXD0 */
//  PORT_SetPinMux(PORTA, PIN17_IDX, kPORT_MuxAlt4);           /* PORTA17 (pin K11) is configured as RMII0_TXD1 */
//  PORT_SetPinMux(PORTA, PIN28_IDX, kPORT_MuxAlt4);           /* PORTA28 (pin H12) is configured as MII0_TXER */
//  PORT_SetPinMux(PORTA, PIN5_IDX, kPORT_MuxAlt4);            /* PORTA5 (pin M8) is configured as RMII0_RXER */
//  PORT_SetPinMux(PORTB, PIN0_IDX, kPORT_MuxAlt4);            /* PORTB0 (pin H10) is configured as RMII0_MDIO */
//  PORTB->PCR[0] = ((PORTB->PCR[0] &
//    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ODE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
//      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
//      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
//      | PORT_PCR_ODE(PCR_ODE_ENABLED)                        /* Open Drain Enable: Open drain output is enabled on the corresponding pin, if the pin is configured as a digital output. */
//    );
//  PORT_SetPinMux(PORTB, PIN1_IDX, kPORT_MuxAlt4);            /* PORTB1 (pin H9) is configured as RMII0_MDC */
//  PORT_SetPinMux(PORTC, PIN16_IDX, kPORT_MuxAlt4);           /* PORTC16 (pin A6) is configured as ENET0_1588_TMR0 */
//  PORT_SetPinMux(PORTC, PIN17_IDX, kPORT_MuxAlt4);           /* PORTC17 (pin D5) is configured as ENET0_1588_TMR1 */
//  PORT_SetPinMux(PORTC, PIN18_IDX, kPORT_MuxAlt4);           /* PORTC18 (pin C5) is configured as ENET0_1588_TMR2 */
//  PORT_SetPinMux(PORTC, PIN19_IDX, kPORT_MuxAlt4);           /* PORTC19 (pin B5) is configured as ENET0_1588_TMR3 */
}


//***************************************************************************
// Timer for LWIP

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Interrupt service for SysTick timer.
 */
void SysTick_Handler(void)
{
    time_isr();
}

#ifdef __cplusplus
}
#endif


// **************************************************************************
// Class Enet for outgoing UDP packets

// IP Addresses

#define PC 0

#if PC
/*
 * If connected directly to Windows PC
 * */
uint8_t configIP[ 4 ] = { 169, 254, 229, 243 };
uint8_t configNET[ 4 ] = { 255, 255, 0, 0 };
uint8_t configGW[ 4 ] = { 169, 254, 229, 242 };
//uint8_t configPC[ 4 ] = { 169, 254, 94, 242 }; // PC
uint8_t configPC[ 4 ] = { 169, 254, 255, 255 }; // broadcast

#else
/*
 * If connected to mini AP
 * */

uint8_t configIP[ 4 ] = { 192, 168, 99, 101 };
uint8_t configNET[ 4 ] = { 255, 255, 255, 0 };
uint8_t configGW[ 4 ] = { 192, 168, 99, 1 };
uint8_t configPC[ 4 ] = { 192, 168, 99, 255 }; // broadcast
#endif


#define IP4_ADDR_U8x4( addr, u8x4 ) IP4_ADDR( addr, u8x4[0], u8x4[1], u8x4[2], u8x4[3] )

/* MAC address configuration. */
#define configMAC_ADDR                     \
    {                                      \
        0x02, 0x12, 0x13, 0x10, 0x15, 0x11 \
    }

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS

/* ENET clock frequency. */
#define EXAMPLE_CLOCK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)

/* MDIO operations. */
#define EXAMPLE_MDIO_OPS enet_ops

/* PHY operations. */
#define EXAMPLE_PHY_OPS phyksz8081_ops

#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */



static mdio_handle_t mdioHandle = {.ops = &EXAMPLE_MDIO_OPS};
static phy_handle_t phyHandle   = {.phyAddr = EXAMPLE_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &EXAMPLE_PHY_OPS};

struct netif netif;
ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;


// Initialization: pins, addresses, timer, lwip layer, UDP connection
int Enet::init( uint32_t data_len_limit, uint16_t port )
{
	//Enet_InitPins();

	IP4_ADDR_U8x4( &m_if_ipaddr, configIP );
	IP4_ADDR_U8x4( &m_if_gw, configGW );
	IP4_ADDR_U8x4( &m_if_netmask, configNET );
	IP4_ADDR_U8x4( &m_pc_ipaddr, configPC );
	m_pc_port = port;
	m_data_inx = 0;
	m_data_len_limit = MIN( UDP_DATA_LIMIT, data_len_limit );

    ethernetif_config_t enet_config = {
        .phyHandle  = &phyHandle,
        .macAddress = configMAC_ADDR,
    };

    SYSMPU_Type *base = SYSMPU;

    /* Disable SYSMPU. */
    base->CESR &= ~SYSMPU_CESR_VLD_MASK;
    /* Set RMII clock src. */
    SIM->SOPT2 |= SIM_SOPT2_RMIISRC_MASK;

    mdioHandle.resource.csrClock_Hz = EXAMPLE_CLOCK_FREQ;

    lwip_init();

    netif_add( &m_fsl_netif, &m_if_ipaddr, &m_if_netmask, &m_if_gw, &enet_config, ethernetif0_init, ethernet_input);
    netif_set_default( &m_fsl_netif );
    netif_set_up( &m_fsl_netif );

    time_init();

//    udpecho_raw_init();

    PRINTF("\r\n************************************************\r\n");
    PRINTF(" IP Configuration\r\n");
    PRINTF("************************************************\r\n");
    uint8_t *adr = ( uint8_t * ) &m_if_ipaddr;
    PRINTF(" IPv4 Address     : %u.%u.%u.%u\r\n", adr[0], adr[1], adr[2], adr[3] );
    adr = ( uint8_t * ) &m_if_netmask;
    PRINTF(" IPv4 Subnet mask : %u.%u.%u.%u\r\n", adr[0], adr[1], adr[2], adr[3] );
    adr = ( uint8_t * ) &m_if_gw;
    PRINTF(" IPv4 Gateway     : %u.%u.%u.%u\r\n", adr[0], adr[1], adr[2], adr[3] );
    PRINTF("************************************************\r\n");

    PRINTF("\r\n************************************************\r\n");
    PRINTF(" UDP Echo example\r\n");
    PRINTF("************************************************\r\n");
    PRINTF(" IPv4 Address     : %u.%u.%u.%u\r\n", ((u8_t *)&netif_ipaddr)[0], ((u8_t *)&netif_ipaddr)[1],
           ((u8_t *)&netif_ipaddr)[2], ((u8_t *)&netif_ipaddr)[3]);
    PRINTF(" IPv4 Subnet mask : %u.%u.%u.%u\r\n", ((u8_t *)&netif_netmask)[0], ((u8_t *)&netif_netmask)[1],
           ((u8_t *)&netif_netmask)[2], ((u8_t *)&netif_netmask)[3]);
    PRINTF(" IPv4 Gateway     : %u.%u.%u.%u\r\n", ((u8_t *)&netif_gw)[0], ((u8_t *)&netif_gw)[1],
           ((u8_t *)&netif_gw)[2], ((u8_t *)&netif_gw)[3]);
    PRINTF("************************************************\r\n");

    for ( int i = 0; i < DATA_BUF_NUM; i++ )
    {
    	m_data_pbuf[ i ] = pbuf_alloc( PBUF_TRANSPORT, m_data_len_limit, PBUF_REF );
    	m_data_pbuf[ i ]->payload = m_data[ i ];
    }

    m_pc_pcb = udp_new();
    udp_bind( m_pc_pcb, &m_if_ipaddr, m_pc_port );
    udp_connect( m_pc_pcb, &m_pc_ipaddr, m_pc_port );

    m_initialized = 1;

    return 1;
}

int Enet::send( const void *ptr_data, uint32_t len )
{
	if ( !m_initialized ){
		return 0;
	}

	len = MIN( UDP_DATA_LIMIT, len );
	memcpy( m_data[ m_data_inx ], ptr_data, len );
	udp_send( m_pc_pcb, m_data_pbuf[ m_data_inx ] );
	m_data_inx = ( m_data_inx + 1 ) % DATA_BUF_NUM;
	return 1;
}

void Enet::check()
{
	if ( !m_initialized ){
		return;
	}
	ethernetif_input( &m_fsl_netif );  // Maybe some unexpected data
	sys_check_timeouts(); /* Handle all system timeouts for all core protocols */
}


