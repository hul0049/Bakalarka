/*
*/
#include <tfc_k6xf.h>
#include "board.h"
#include "pin_mux.h"
#include "fsl_device_registers.h"
#include "fsl_clock.h"
#include "fsl_port.h"
#include "fsl_pit.h"
#include "fsl_adc16.h"
#include "fsl_ftm.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

#include "tfc_config.h"
#include "tfc.h"

#ifndef HW_TFC_PCB_VERSION
#error Unknown TFC PCB version
#endif

#if !defined( K64F12_SERIES ) && !defined( K66F18_SERIES )
#error Unknown MCU type
#endif

#define HW_TFC_PCB_VERSION_MAJOR	( ( HW_TFC_PCB_VERSION ) / 10 )
#define HW_TFC_PCB_VERSION_MINOR	( ( HW_TFC_PCB_VERSION ) % 10 )

#ifdef SDK_OS_FREE_RTOS
SemaphoreHandle_t RC_semaphore;
SemaphoreHandle_t Camera_semaphore;
#endif
/** *************************************************************************
 * Motors
*/

#ifdef K64F12_SERIES

#if ( HW_TFC_PCB_VERSION_MAJOR == 1 )
#define HW_TFC_MOTOR_PORT_SIG		PORTD			// Port with control signals
#define HW_TFC_MOTOR_CLOCK_SIG		kCLOCK_PortD	// Clock gate for PORT_SIG
#define HW_TFC_MOTOR_GPIO_SIG		GPIOD			// GPIO for control signals
#define HW_TFC_MOTOR_PIN_FAULT		3				// Fault status from H-Bridges (2x Open Collector)
#define HW_TFC_MOTOR_PIN_ENABLE		2				// Enable both H-Bridges
#else
#define HW_TFC_MOTOR_PORT_SIG		PORTC			// Port with control signals
#define HW_TFC_MOTOR_CLOCK_SIG		kCLOCK_PortC	// Clock gate for PORT_SIG
#define HW_TFC_MOTOR_GPIO_SIG		GPIOC			// GPIO for control signals
#define HW_TFC_MOTOR_PIN_FAULT		0				// Fault status from H-Bridges (2x Open Collector)
#define HW_TFC_MOTOR_PIN_ENABLE		12				// Enable both H-Bridges
#endif

#endif // K64F12_SERIES

#ifdef K66F18_SERIES

#define HW_TFC_MOTOR_PORT_SIG		PORTE			// Port with control signals
#define HW_TFC_MOTOR_CLOCK_SIG		kCLOCK_PortE	// Clock gate for PORT_SIG
#define HW_TFC_MOTOR_GPIO_SIG		GPIOE			// GPIO for control signals
#define HW_TFC_MOTOR_PIN_ENABLE		7				// Enable both H-Bridges
#define HW_TFC_MOTOR_PIN_FAULT		8				// Fault status from H-Bridges (2x Open Collector)

#endif // K66F18_SERIES

#define HW_TFC_MOTOR_FTM			FTM0			// selected FTM for motor PWM
#define HW_TFC_MOTOR_FREQ			4000			// Hz
#define HW_TFC_MOTOR_PORT_HB		PORTC			// Port with four FTM_CHx
#define HW_TFC_MOTOR_CLOCK_HB		kCLOCK_PortC	// Port with four FTM_CHx
#define HW_TFC_MOTOR_PIN_HB_A0		1				// Pin number for HB_A0
#define HW_TFC_MOTOR_CHNL_HB_A0		kFTM_Chnl_0		// FTM_CHx for HB_A0
#define HW_TFC_MOTOR_PIN_HB_A1		2				// Pin number for HB_A1
#define HW_TFC_MOTOR_CHNL_HB_A1		kFTM_Chnl_1		// FTM_CHx for HB_A1
#define HW_TFC_MOTOR_PIN_HB_B0		3				// Pin number for HB_B0
#define HW_TFC_MOTOR_CHNL_HB_B0		kFTM_Chnl_2		// FTM_CHx for HB_B0
#define HW_TFC_MOTOR_PIN_HB_B1		4				// Pin number for HB_B1
#define HW_TFC_MOTOR_CHNL_HB_B1		kFTM_Chnl_3		// FTM_CHx for HB_B1

void HW_TFC_MOTOR_Init()
{
    CLOCK_EnableClock( HW_TFC_MOTOR_CLOCK_HB );
    CLOCK_EnableClock( HW_TFC_MOTOR_CLOCK_SIG );

    PORT_SetPinMux( HW_TFC_MOTOR_PORT_HB, HW_TFC_MOTOR_PIN_HB_A0, kPORT_MuxAlt4 );
    PORT_SetPinMux( HW_TFC_MOTOR_PORT_HB, HW_TFC_MOTOR_PIN_HB_A1, kPORT_MuxAlt4 );
    PORT_SetPinMux( HW_TFC_MOTOR_PORT_HB, HW_TFC_MOTOR_PIN_HB_B0, kPORT_MuxAlt4 );
    PORT_SetPinMux( HW_TFC_MOTOR_PORT_HB, HW_TFC_MOTOR_PIN_HB_B1, kPORT_MuxAlt4 );

    PORT_SetPinMux( HW_TFC_MOTOR_PORT_SIG, HW_TFC_MOTOR_PIN_ENABLE, kPORT_MuxAsGpio );
    PORT_SetPinMux( HW_TFC_MOTOR_PORT_SIG, HW_TFC_MOTOR_PIN_FAULT, kPORT_MuxAsGpio );

    gpio_pin_config_t pin_output = { kGPIO_DigitalOutput, 0 };
    GPIO_PinInit( HW_TFC_MOTOR_GPIO_SIG, HW_TFC_MOTOR_PIN_ENABLE, &pin_output );

    ftm_config_t ftmconf;
    ftm_chnl_pwm_signal_param_t ftmpar;

	FTM_GetDefaultConfig( &ftmconf );

	ftmconf.reloadPoints = kFTM_CntMax;
	ftmconf.prescale = kFTM_Prescale_Divide_32;
	ftmconf.bdmMode = kFTM_BdmMode_1;

	FTM_Init( HW_TFC_MOTOR_FTM, &ftmconf);

	//HW_TFC_MOTOR_FTM->MODE &= ~FTM_MODE_FTMEN_MASK; // Legacy mode
	HW_TFC_MOTOR_FTM->SYNCONF &= ~FTM_SYNCONF_SWRSTCNT_MASK;

	ftmpar.dutyCyclePercent = 0;
	ftmpar.firstEdgeDelayPercent = 0;
	ftmpar.level = kFTM_HighTrue;

	ftmpar.chnlNumber = HW_TFC_MOTOR_CHNL_HB_A0;
	FTM_SetupPwm( HW_TFC_MOTOR_FTM, &ftmpar, 1, kFTM_EdgeAlignedPwm, HW_TFC_MOTOR_FREQ, CLOCK_GetBusClkFreq() );

	ftmpar.chnlNumber = HW_TFC_MOTOR_CHNL_HB_B0;
	FTM_SetupPwm( HW_TFC_MOTOR_FTM, &ftmpar, 1, kFTM_EdgeAlignedPwm, HW_TFC_MOTOR_FREQ, CLOCK_GetBusClkFreq() );

	ftmpar.chnlNumber = HW_TFC_MOTOR_CHNL_HB_A1;
	FTM_SetupPwm( HW_TFC_MOTOR_FTM, &ftmpar, 1, kFTM_EdgeAlignedPwm, HW_TFC_MOTOR_FREQ, CLOCK_GetBusClkFreq() );

	ftmpar.chnlNumber = HW_TFC_MOTOR_CHNL_HB_B1;
	FTM_SetupPwm( HW_TFC_MOTOR_FTM, &ftmpar, 1, kFTM_EdgeAlignedPwm, HW_TFC_MOTOR_FREQ, CLOCK_GetBusClkFreq() );
}

void HW_TFC_MOTOR_OnOff( int onoff )
{
	static int _s_is_on = 0;
	if ( !_s_is_on == !onoff ) return;
	_s_is_on = onoff;
	if ( onoff )
	{
		HW_TFC_MOTOR_SetPWM( 0, 0 );
		FTM_StartTimer( HW_TFC_MOTOR_FTM, kFTM_SystemClock );
		GPIO_PortSet( HW_TFC_MOTOR_GPIO_SIG, 1 << HW_TFC_MOTOR_PIN_ENABLE );
	}
	else
	{
		HW_TFC_MOTOR_SetPWM( 0, 0 );
		GPIO_PortClear( HW_TFC_MOTOR_GPIO_SIG, 1 << HW_TFC_MOTOR_PIN_ENABLE );
		FTM_StopTimer( HW_TFC_MOTOR_FTM );
	}
}

void HW_TFC_MOTOR_SetPWM( int motor_a, int motor_b )
{
	uint32_t duty_a = 0;
	uint32_t duty_b = 0;
	int32_t sign_a = motor_a < 0;
	int32_t sign_b = motor_b < 0;

	if ( motor_a || motor_b )
	{
		motor_a = MIN( motor_a, HW_TFC_PWM_MAX );
		motor_a = MAX( motor_a, -HW_TFC_PWM_MAX );
		motor_b = MIN( motor_b, HW_TFC_PWM_MAX );
		motor_b = MAX( motor_b, -HW_TFC_PWM_MAX );

		if ( sign_a ) motor_a = -motor_a;
		if ( sign_b ) motor_b = -motor_b;

		duty_a = HW_TFC_MOTOR_FTM->MOD * motor_a / ( TFC_PWM_MINMAX );
		duty_b = HW_TFC_MOTOR_FTM->MOD * motor_b / ( TFC_PWM_MINMAX );
	}

	HW_TFC_MOTOR_FTM->CONTROLS[ HW_TFC_MOTOR_CHNL_HB_A0 + sign_a ].CnV = duty_a;
	HW_TFC_MOTOR_FTM->CONTROLS[ HW_TFC_MOTOR_CHNL_HB_A1 - sign_a ].CnV = 0;
	HW_TFC_MOTOR_FTM->CONTROLS[ HW_TFC_MOTOR_CHNL_HB_B0 + sign_b ].CnV = duty_b;
	HW_TFC_MOTOR_FTM->CONTROLS[ HW_TFC_MOTOR_CHNL_HB_B1 - sign_b ].CnV = 0;

	FTM_SetSoftwareTrigger( HW_TFC_MOTOR_FTM, true );
}

/** *************************************************************************
 * Servos
 */

#if defined( K64F12_SERIES ) && ( HW_TFC_PCB_VERSION_MAJOR == 1 )

#define HW_TFC_SERVO_FTM			FTM3			// selected FTM for both servos
#define HW_TFC_SERVO_IRQHandler		FTM3_IRQHandler // IRQ handler
#define HW_TFC_SERVO_IRQn			FTM3_IRQn		// IRQn
#define HW_TFC_SERVO_PORT			PORTD			// Port with both FTM_CHx
#define HW_TFC_SERVO_CLOCK			kCLOCK_PortD	// Clock gate for PORTD
#define HW_TFC_SERVO_PIN_ALTx		kPORT_MuxAlt4
#define HW_TFC_SERVO0_PIN			0				// Pin number for servo 0
#define HW_TFC_SERVO1_PIN			1				// Pin number for servo 1

#else

#define HW_TFC_SERVO_FTM			FTM2			// selected FTM for both servos
#define HW_TFC_SERVO_IRQHandler		FTM2_IRQHandler // IRQ handler
#define HW_TFC_SERVO_IRQn			FTM2_IRQn		// IRQn
#define HW_TFC_SERVO_PORT			PORTB			// Port with both FTM_CHx
#define HW_TFC_SERVO_CLOCK			kCLOCK_PortB	// Clock gate for SERVO_PORT
#define HW_TFC_SERVO_PIN_ALTx		kPORT_MuxAlt3
#define HW_TFC_SERVO0_PIN			18				// Pin number for servo 0
#define HW_TFC_SERVO1_PIN			19				// Pin number for servo 1
#endif

#define HW_TFC_SERVO_FREQ			100			    // Hz, Futaba accepts freq. from 46 to 100 Hz
#define HW_TFC_SERVO0_CHNL			kFTM_Chnl_0		// FTM_CHx for servo 0
#define HW_TFC_SERVO1_CHNL			kFTM_Chnl_1		// FTM_CHx for servo 1

// Set proper timing for used SERVO!
// e.g. Futaba S3010, center 1500 us, move +/- 500 us (up to 600 us)
#define SERVO0_CENTER				1500			// time in us, center position
#define SERVO0_MAX_LR				500				// time in us, max. move left & right

#define SERVO1_CENTER				1500   			// time in us, center position
#define SERVO1_MAX_LR				500				// time in us, max. move left & right


extern "C" { void HW_TFC_SERVO_IRQHandler(); }

static volatile int _s_hw_tfc_servo_is_on = 0;
static volatile int _s_hw_tfc_start_servo = 0;

void HW_TFC_SERVO_IRQHandler()
{
	FTM_DisableInterrupts( HW_TFC_SERVO_FTM, kFTM_TimeOverflowInterruptEnable );
	FTM_ClearStatusFlags( HW_TFC_SERVO_FTM, kFTM_TimeOverflowFlag );
	FTM_StopTimer( HW_TFC_SERVO_FTM );
	_s_hw_tfc_servo_is_on = 0;
}


void HW_TFC_SERVO_Init()
{
    CLOCK_EnableClock( HW_TFC_SERVO_CLOCK );

    PORT_SetPinMux( HW_TFC_SERVO_PORT, HW_TFC_SERVO0_PIN, HW_TFC_SERVO_PIN_ALTx );
    PORT_SetPinMux( HW_TFC_SERVO_PORT, HW_TFC_SERVO1_PIN, HW_TFC_SERVO_PIN_ALTx );

    ftm_config_t ftmconf;
    ftm_chnl_pwm_signal_param_t ftmpar;

	FTM_GetDefaultConfig( &ftmconf );
	ftmconf.reloadPoints = kFTM_CntMax;
	ftmconf.prescale = kFTM_Prescale_Divide_32;
	ftmconf.bdmMode = kFTM_BdmMode_1;

	FTM_Init( HW_TFC_SERVO_FTM, &ftmconf );

	//HW_TFC_SERVO_FTM->MODE &= ~FTM_MODE_FTMEN_MASK; // Legacy mode
	HW_TFC_SERVO_FTM->SYNCONF &= ~FTM_SYNCONF_SWRSTCNT_MASK;

	ftmpar.chnlNumber = HW_TFC_SERVO0_CHNL;
	ftmpar.level = kFTM_HighTrue;
	ftmpar.dutyCyclePercent = 0;

	FTM_SetupPwm(HW_TFC_SERVO_FTM, &ftmpar, 1, kFTM_EdgeAlignedPwm, HW_TFC_SERVO_FREQ, CLOCK_GetBusClkFreq());

	ftmpar.chnlNumber = HW_TFC_SERVO1_CHNL;

	FTM_SetupPwm(HW_TFC_SERVO_FTM, &ftmpar, 1, kFTM_EdgeAlignedPwm, HW_TFC_SERVO_FREQ, CLOCK_GetBusClkFreq());

	EnableIRQ( HW_TFC_SERVO_IRQn );

	_s_hw_tfc_servo_is_on = 0;
	_s_hw_tfc_start_servo = 0;
}

void HW_TFC_SERVO_OnOff( int onoff )
{
	if ( !onoff == !_s_hw_tfc_servo_is_on ) return;

	HW_TFC_SERVO_Set( 0, 0 );
	HW_TFC_SERVO_Set( 1, 0 );

	if ( onoff )
	{
		_s_hw_tfc_start_servo = 1;	// postpone starting
		HW_TFC_SERVO_FTM->CNT = 0;
	}
	else
	{
		FTM_ClearStatusFlags( HW_TFC_SERVO_FTM, kFTM_TimeOverflowFlag );
		FTM_EnableInterrupts( HW_TFC_SERVO_FTM, kFTM_TimeOverflowInterruptEnable );
	}
}

void HW_TFC_SERVO_Set( int servo, int pos_us )
{
	if ( !_s_hw_tfc_servo_is_on ) return;

	pos_us = MIN( pos_us, HW_TFC_SERVO_MAX );
	pos_us = MAX( pos_us, HW_TFC_SERVO_MIN );

	servo = servo != 0;
	HW_TFC_SERVO_FTM->CONTROLS[ servo ].CnV = HW_TFC_SERVO_FTM->MOD * pos_us / ( 1000000/*us*/ / HW_TFC_SERVO_FREQ );
	FTM_SetSoftwareTrigger( HW_TFC_SERVO_FTM, true );
}

/** *************************************************************************
 * AD converter
 */
#ifdef K64F12_SERIES

#define HW_TFC_ANDATA_ADC			ADC1			// AD controler
#define HW_TFC_ANDATA_IRQHandler	ADC1_IRQHandler // IRQ handler
#define HW_TFC_ANDATA_IRQn			ADC1_IRQn		// IRQn
#define HW_TFC_ANDATA_PORT			PORTB			// Port with AD channels
#define HW_TFC_ANDATA_CLOCK1		kCLOCK_Adc1		// Clock

#if ( HW_TFC_PCB_VERSION_MAJOR == 1 )

#define HW_TFC_ANDATA_CLOCK2		kCLOCK_PortB	// Clock
#define HW_TFC_ANDATA_PIN_POT1		11				// Pin of POT1
#define HW_TFC_ANDATA_PIN_POT2		10				// Pin of POT2
#define HW_TFC_ANDATA_CHN_POT1		15				// AD channel of POT1
#define HW_TFC_ANDATA_CHN_POT2		14				// AD channel of POT2
#define HW_TFC_ANDATA_CHN_FB_A		20				// AD channel of FB_A
#define HW_TFC_ANDATA_CHN_FB_B		1				// AD channel of FB_B
#define HW_TFC_ANDATA_CHN_BAT		18				// AD channel of Battery

#else

#define HW_TFC_ANDATA_CHN_POT1		0				// AD channel of POT1
#define HW_TFC_ANDATA_CHN_POT2		19				// AD channel of POT2
#define HW_TFC_ANDATA_CHN_FB_A		1				// AD channel of FB_A
#define HW_TFC_ANDATA_CHN_FB_B		20				// AD channel of FB_B
#define HW_TFC_ANDATA_CHN_BAT		18				// AD channel of Battery

#endif

#endif // K64F12_SERIES

#ifdef K66F18_SERIES

#define HW_TFC_ANDATA_ADC			ADC0			// AD controller
#define HW_TFC_ANDATA_IRQHandler	ADC0_IRQHandler // IRQ handler
#define HW_TFC_ANDATA_IRQn			ADC0_IRQn		// IRQn
#define HW_TFC_ANDATA_CLOCK1		kCLOCK_Adc0		// Clock
#define HW_TFC_ANDATA_CHN_POT1		17				// AD channel of POT1
#define HW_TFC_ANDATA_CHN_POT2		18				// AD channel of POT2
#define HW_TFC_ANDATA_CHN_FB_A		12				// AD channel of FB_A
#define HW_TFC_ANDATA_CHN_FB_B		13				// AD channel of FB_B
#define HW_TFC_ANDATA_CHN_BAT		14				// AD channel of Battery

#endif // K66F18_SERIES


// Pair of channel and measured value
typedef struct {
	const uint32_t channel;
	uint32_t value;
} hw_tfc_anadata_chnl_t;

// Table for measured values
static volatile hw_tfc_anadata_chnl_t _s_andata_chnls[ anLast ] = {
		{ HW_TFC_ANDATA_CHN_POT1, 0 },	// anPOT_1
		{ HW_TFC_ANDATA_CHN_POT2, 0 }, 	// anPOT_2
		{ HW_TFC_ANDATA_CHN_FB_A, 0 }, 	// anFB_A
		{ HW_TFC_ANDATA_CHN_FB_B, 0 }, 	// anFB_B
		{ HW_TFC_ANDATA_CHN_BAT, 0 }	// anBAT
};

// Internal global variables for AD converter
static volatile uint32_t _s_andata_chnl_idx = -1;
static adc16_channel_config_t _s_andata_chnl_conf = { 0, true, false };

/**
 * Interrupt handler for AD converter
 */
extern "C" { void HW_TFC_ANDATA_IRQHandler(); }

void HW_TFC_ANDATA_IRQHandler()
{
	_s_andata_chnls[ _s_andata_chnl_idx ].value = ADC16_GetChannelConversionValue( HW_TFC_ANDATA_ADC, 0 );

	_s_andata_chnl_idx++;

	if ( _s_andata_chnl_idx == anLast )
	{
		_s_andata_chnl_idx = -1;
		HW_TFC_ANDATA_OnOff( 0 );
		//DisableIRQ( ADC1_IRQn );
	}
	else
	{
		_s_andata_chnl_conf.channelNumber = _s_andata_chnls[ _s_andata_chnl_idx ].channel;
		ADC16_SetChannelConfig( HW_TFC_ANDATA_ADC, 0, &_s_andata_chnl_conf );
	}
}

void HW_TFC_ANDATA_Init()
{
	CLOCK_EnableClock( HW_TFC_ANDATA_CLOCK1 );
#if defined( K64F12_SERIES ) && ( HW_TFC_PCB_VERSION_MAJOR == 1 )
	CLOCK_EnableClock( HW_TFC_ANDATA_CLOCK2 );

	PORT_SetPinMux( HW_TFC_ANDATA_PORT, HW_TFC_ANDATA_PIN_POT1, kPORT_PinDisabledOrAnalog );
	PORT_SetPinMux( HW_TFC_ANDATA_PORT, HW_TFC_ANDATA_PIN_POT2, kPORT_PinDisabledOrAnalog );
#endif

	adc16_config_t adc16conf;
	ADC16_GetDefaultConfig( &adc16conf );
	//adc16conf.clockDivider = kADC16_ClockDivider4;
	ADC16_Init( HW_TFC_ANDATA_ADC, &adc16conf );
	ADC16_EnableHardwareTrigger( HW_TFC_ANDATA_ADC, false );
	ADC16_DoAutoCalibration( HW_TFC_ANDATA_ADC );
}

void HW_TFC_ANDATA_OnOff( int onoff )
{
	if ( onoff )
	{
		_s_andata_chnl_idx = 0;
		_s_andata_chnl_conf.channelNumber = _s_andata_chnls[ _s_andata_chnl_idx ].channel;

		EnableIRQ( HW_TFC_ANDATA_IRQn );
		ADC16_SetChannelConfig( HW_TFC_ANDATA_ADC, 0, &_s_andata_chnl_conf );
	}
	else
	{
		DisableIRQ( HW_TFC_ANDATA_IRQn );
	}
}

uint16_t HW_TFC_ANDATA_getVal( uint32_t chnl )
{
	if ( chnl < anLast )
		return _s_andata_chnls[ chnl ].value;
	return 0xFFFF;
}

/** *************************************************************************
 * Cameras
 */

#ifdef K64F12_SERIES

#define HW_TFC_CAMERA_ADC				ADC0
#define HW_TFC_CAMERA_IRQHandler		ADC0_IRQHandler
#define HW_TFC_CAMERA_IRQn				ADC0_IRQn
#define HW_TFC_CAMERA_PORT_SIG			PORTC
#define HW_TFC_CAMERA_GPIO_SIG			GPIOC
#define HW_TFC_CAMERA_CLOCK1			kCLOCK_Adc0
#define HW_TFC_CAMERA_CLOCK2			kCLOCK_PortC
#define HW_TFC_CAMERA_PIN_SI			5
#define HW_TFC_CAMERA_PIN_CLK			7
#define HW_TFC_CAMERA0_CHNL				0
#define HW_TFC_CAMERA1_CHNL				19

#endif // K64F12_SERIES

#ifdef K66F18_SERIES

#define HW_TFC_CAMERA_ADC				ADC1
#define HW_TFC_CAMERA_IRQHandler		ADC1_IRQHandler
#define HW_TFC_CAMERA_IRQn				ADC1_IRQn
#define HW_TFC_CAMERA_PORT_SIG			PORTD
#define HW_TFC_CAMERA_GPIO_SIG			GPIOD
#define HW_TFC_CAMERA_CLOCK1			kCLOCK_Adc0
#define HW_TFC_CAMERA_CLOCK2			kCLOCK_PortD
#define HW_TFC_CAMERA_PIN_SI			12
#define HW_TFC_CAMERA_PIN_CLK			13
#define HW_TFC_CAMERA0_CHNL				14
#define HW_TFC_CAMERA1_CHNL				15

#endif // K66F18_SERIES

#define _HW_CAMERA_CLK_DELAY()				asm( "NOP; NOP; NOP; NOP; NOP;" )

#define _HW_NUM_OF_CAMERAS					1

#if ( _HW_NUM_OF_CAMERAS > 2 )
#error Number of cameras can not be greater then 2!
#endif

/**
 * Data for camera sampling
 */
static volatile uint16_t _sa_camera_buf[ _HW_NUM_OF_CAMERAS ][ 2 ][ TFC_CAMERA_LINE_LENGTH ];
static volatile uint16_t *_sp_camera_work_buf[ _HW_NUM_OF_CAMERAS ];
static volatile uint16_t *_sp_camera_image_buf[ _HW_NUM_OF_CAMERAS ];
static volatile uint32_t _s_camera_image_ready = 0;

static volatile int _s_camera_idx = 0;				// selected camera
static volatile int _s_camera_line_idx = -1;		// current sample in line
static uint32_t _s_camera_channels[ 2 ] = { HW_TFC_CAMERA0_CHNL, HW_TFC_CAMERA1_CHNL };

//adc16_channel_config_t _camera_chnl_conf = { HW_TFC_CAMERA0_CHNL, false, true };

void HW_TFC_CAMERA_OnOff( int onoff );

/**
 * Set SI signal to 0/1
 * @param level
 */
static inline void HW_TFC_CAMERA_SI_Set( int level )
{
	if ( level )
		GPIO_PortSet( HW_TFC_CAMERA_GPIO_SIG, 1 << HW_TFC_CAMERA_PIN_SI );
	else
		GPIO_PortClear( HW_TFC_CAMERA_GPIO_SIG, 1 << HW_TFC_CAMERA_PIN_SI );
}

/**
 * Set CLK signal to 0/1
 * @param level
 */
static inline void HW_TFC_CAMERA_CLK_Set( int level )
{
	if ( level )
		GPIO_PortSet( HW_TFC_CAMERA_GPIO_SIG, 1 << HW_TFC_CAMERA_PIN_CLK );
	else
		GPIO_PortClear( HW_TFC_CAMERA_GPIO_SIG, 1 << HW_TFC_CAMERA_PIN_CLK );
}

/**
 * Interrupt handler for AD converter of cameras
 */
extern "C" { void HW_TFC_CAMERA_IRQHandler(); }

void HW_TFC_CAMERA_IRQHandler()
{
	if ( !_s_camera_line_idx )
	{
		HW_TFC_CAMERA_SI_Set( 0 );
		_HW_CAMERA_CLK_DELAY();
	}

	if ( !_s_camera_idx )
	{
		HW_TFC_CAMERA_CLK_Set( 0 );
		_HW_CAMERA_CLK_DELAY();
	}

	if ( _s_camera_line_idx < TFC_CAMERA_LINE_LENGTH )
	{
		_sp_camera_work_buf[ _s_camera_idx++ ][ _s_camera_line_idx ] = ADC16_GetChannelConversionValue( HW_TFC_CAMERA_ADC, 0 );

		if ( _s_camera_idx == _HW_NUM_OF_CAMERAS )
		{
			_s_camera_idx = 0;
			_s_camera_line_idx++;
			HW_TFC_CAMERA_CLK_Set( 1 );
		}

		adc16_channel_config_t chc = { _s_camera_channels[ _s_camera_idx ], true, false };
		ADC16_SetChannelConfig( HW_TFC_CAMERA_ADC, 0, &chc );
	}
	else
	{
		ADC16_GetChannelConversionValue( HW_TFC_CAMERA_ADC, 0 );


		// swap buffers
		for ( int i = 0; i < _HW_NUM_OF_CAMERAS; i++ )
		{
			volatile uint16_t *swap = _sp_camera_work_buf[ i ];
			_sp_camera_work_buf[ i ] = _sp_camera_image_buf[ i ];
			_sp_camera_image_buf[ i ] = swap;
		}

		_s_camera_image_ready = ~( ( ~0 ) << _HW_NUM_OF_CAMERAS );
		_s_camera_line_idx = -1;
		HW_TFC_CAMERA_OnOff( 0 );
		//DisableIRQ( HW_TFC_CAMERA_IRQn );
#ifdef SDK_OS_FREE_RTOS
		portBASE_TYPE *xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(Camera_semaphore, xHigherPriorityTaskWoken);
		printf("Image captured! @%d\n", xTaskGetTickCountFromISR());
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
#endif
	}
}

/**
 * Initialization of camera interfaces
 */
void HW_TFC_CAMERA_Init()
{
	CLOCK_EnableClock( HW_TFC_CAMERA_CLOCK1 );
	CLOCK_EnableClock( HW_TFC_CAMERA_CLOCK2 );

	PORT_SetPinMux( HW_TFC_CAMERA_PORT_SIG, HW_TFC_CAMERA_PIN_SI, kPORT_MuxAsGpio );
	PORT_SetPinMux( HW_TFC_CAMERA_PORT_SIG, HW_TFC_CAMERA_PIN_CLK, kPORT_MuxAsGpio );
	// ADC0_DP0 for camera 0
	// ADC0_DM0 for camera 1

    gpio_pin_config_t pin_output = { kGPIO_DigitalOutput, 0 };
    GPIO_PinInit( HW_TFC_CAMERA_GPIO_SIG, HW_TFC_CAMERA_PIN_SI, &pin_output );
    GPIO_PinInit( HW_TFC_CAMERA_GPIO_SIG, HW_TFC_CAMERA_PIN_CLK, &pin_output );

    HW_TFC_CAMERA_SI_Set( 0 );
    HW_TFC_CAMERA_CLK_Set( 0 );

	adc16_config_t adc16conf;
	ADC16_GetDefaultConfig( &adc16conf );
	adc16conf.clockDivider = kADC16_ClockDivider4;

	ADC16_Init( HW_TFC_CAMERA_ADC, &adc16conf );
	ADC16_EnableHardwareTrigger( HW_TFC_CAMERA_ADC, false );
	ADC16_DoAutoCalibration( HW_TFC_CAMERA_ADC );

	bzero( ( void * ) _sa_camera_buf, sizeof( _sa_camera_buf ) );

	for ( int i = 0; i < _HW_NUM_OF_CAMERAS; i++ )
	{
		_sp_camera_work_buf[ i ] = _sa_camera_buf[ i ][ 0 ];
		_sp_camera_image_buf[ i ] = _sa_camera_buf[ i ][ 1 ];
	}
}

/**
 * Switch cameras On or Off
 * @param onoff
 */
void HW_TFC_CAMERA_OnOff( int onoff )
{
	if ( onoff )
	{
		HW_TFC_CAMERA_SI_Set( 1 );
		_HW_CAMERA_CLK_DELAY();
		HW_TFC_CAMERA_CLK_Set( 1 );

		_s_camera_line_idx = 0;
		_s_camera_idx = 0;
		adc16_channel_config_t chc = { _s_camera_channels[ _s_camera_idx ], true, false };
		ADC16_SetChannelConfig( HW_TFC_CAMERA_ADC, 0, &chc );
		EnableIRQ( HW_TFC_CAMERA_IRQn );
	}
	else
	{
		DisableIRQ( HW_TFC_CAMERA_IRQn );
	}
}

uint32_t HW_TFC_CAMERA_isReady()
{
	return _s_camera_image_ready;
}

void HW_TFC_CAMERA_getImage( int chnl, uint16_t *line, int len )
{
	memcpy( line, ( void * ) _sp_camera_image_buf[ chnl ], sizeof( uint16_t ) * len );
	_s_camera_image_ready &= ~( 1 << chnl );
}

/** *************************************************************************
 * Timer
 */

#if defined( K64F12_SERIES ) && ( HW_TFC_PCB_VERSION_MAJOR == 1 )

#define HW_TFC_TICKER_FTM			FTM2				// FTM used
#define HW_TFC_TICKER_IRQHandler	FTM2_IRQHandler		// Interrupt handler
#define HW_TFC_TICKER_IRQn			FTM2_IRQn			// IRQn

#else

#define HW_TFC_TICKER_FTM			FTM1				// FTM used
#define HW_TFC_TICKER_IRQHandler	FTM1_IRQHandler		// Interrupt handler
#define HW_TFC_TICKER_IRQn			FTM1_IRQn			// IRQn

#endif

#define HW_TFC_TICKER_FREQ			1000			    // Frequency in Hz
#define HW_TFC_TICKER_ANDATA_PERIOD	10					// Period of analog data capturing
#define HW_TFC_TICKER_CAMERA_PERIOD	10					// Period of camera data capturing

/**
 * Data for timers and periodic trigger of AD conversion
 */
volatile uint32_t HW_TFC_Ticker_Up[ HW_TFC_NUM_TICKERS ], HW_TFC_Ticker_Down[ HW_TFC_NUM_TICKERS ];
volatile uint32_t HW_TFC_TimeStamp = 0;

static volatile uint32_t _HW_TFC_camera_ticks = HW_TFC_TICKER_CAMERA_PERIOD;
static volatile uint32_t _HW_TFC_andata_ticks = HW_TFC_TICKER_ANDATA_PERIOD / 2;
static          uint32_t _HW_TFC_Ticker_1ms_ticks = 0;
static volatile uint32_t _HW_TFC_Ticker_ms = 0;

/**
 * Interrupt handler for periodic timer
 */
extern "C" { void HW_TFC_TICKER_IRQHandler(); }

void HW_TFC_TICKER_IRQHandler()
{
	_HW_TFC_Ticker_ms++;

	// global count down timers
	for ( int i = 0; i < HW_TFC_NUM_TICKERS; i++ )
	{
		if ( HW_TFC_Ticker_Down[ i ] ) HW_TFC_Ticker_Down[ i ]--;
		HW_TFC_Ticker_Up[ i ]++;
	}

	_HW_TFC_camera_ticks--;
	if ( !_HW_TFC_camera_ticks )
	{
		HW_TFC_CAMERA_OnOff( 1 );
		_HW_TFC_camera_ticks = HW_TFC_TICKER_CAMERA_PERIOD;
		HW_TFC_TimeStamp++;
	}

	_HW_TFC_andata_ticks--;
	if ( !_HW_TFC_andata_ticks )
	{
		HW_TFC_ANDATA_OnOff( 1 );
		_HW_TFC_andata_ticks = HW_TFC_TICKER_ANDATA_PERIOD;

		if ( _s_hw_tfc_start_servo )
		{
			FTM_StartTimer( HW_TFC_SERVO_FTM, kFTM_SystemClock );
			_s_hw_tfc_start_servo = 0;
			_s_hw_tfc_servo_is_on = 1;
		}
	}

	FTM_ClearStatusFlags( HW_TFC_TICKER_FTM, kFTM_TimeOverflowFlag );
}

void HW_TFC_Ticker_Init()
{
    ftm_config_t ftmconf;

	FTM_GetDefaultConfig( &ftmconf );
	ftmconf.prescale = kFTM_Prescale_Divide_4;

	FTM_Init( HW_TFC_TICKER_FTM, &ftmconf);
	_HW_TFC_Ticker_1ms_ticks = MSEC_TO_COUNT( 1, CLOCK_GetBusClkFreq() / 4 );
	HW_TFC_TICKER_FTM->MOD = _HW_TFC_Ticker_1ms_ticks;

	FTM_EnableInterrupts( HW_TFC_TICKER_FTM, kFTM_TimeOverflowInterruptEnable );
}

void HW_TFC_Ticker_OnOff( int onoff )
{
	if ( onoff )
	{
		EnableIRQ( HW_TFC_TICKER_IRQn );
		FTM_StartTimer( HW_TFC_TICKER_FTM, kFTM_SystemClock );
	}
	else
	{
		FTM_StopTimer( HW_TFC_TICKER_FTM );
		DisableIRQ( HW_TFC_TICKER_IRQn );
	}
}

/** *************************************************************************
 * PORTs and PINs
 */
#ifdef K64F12_SERIES

#define HW_TFC_SWPIN_PORT			PORTB
#define HW_TFC_SWPIN_GPIO			GPIOB
#define HW_TFC_SWPIN_CLOCK			kCLOCK_PortB
#define HW_TFC_SWPIN_SW1			2
#define HW_TFC_SWPIN_SW2			3
#define HW_TFC_SWPIN_SW3			9
#if ( HW_TFC_PCB_VERSION_MAJOR == 1 )
#define HW_TFC_SWPIN_SW4			19
#else
#define HW_TFC_SWPIN_SW4			10
#endif


#define HW_TFC_BUTPIN_PORT			PORTB
#define HW_TFC_BUTPIN_GPIO			GPIOB
#define HW_TFC_BUTPIN_CLOCK			kCLOCK_PortB
#if ( ( HW_TFC_PCB_VERSION_MAJOR == 1 ) && ( HW_TFC_PCB_VERSION_MINOR == 1 ) )
#define HW_TFC_BUTPIN_BUTA			20
#define HW_TFC_BUTPIN_BUTB			23
#else
#define HW_TFC_BUTPIN_BUTA			23
#define HW_TFC_BUTPIN_BUTB			20
#endif

#define HW_TFC_LEDPIN_PORT			PORTC
#define HW_TFC_LEDPIN_GPIO			GPIOC
#define HW_TFC_LEDPIN_CLOCK			kCLOCK_PortC
#define HW_TFC_LEDPIN_LED1			8
#define HW_TFC_LEDPIN_LED2			9
#define HW_TFC_LEDPIN_LED3			10
#define HW_TFC_LEDPIN_LED4			11

#if ( HW_TFC_PCB_VERSION_MAJOR == 1 )
#define HW_TFC_SPEED0_PORT			PORTC
#define HW_TFC_SPEED0_GPIO			GPIOC
#define HW_TFC_SPEED0_CLOCK			kCLOCK_PortC
#define HW_TFC_SPEED0_PIN			16
#define HW_TFC_SPEED0_IRQHandler	PORTC_IRQHandler
#define HW_TFC_SPEED0_IRQn			PORTC_IRQn

#define HW_TFC_SPEED1_PORT			PORTB
#define HW_TFC_SPEED1_GPIO			GPIOB
#define HW_TFC_SPEED1_CLOCK			kCLOCK_PortB
#define HW_TFC_SPEED1_PIN			18
#define HW_TFC_SPEED1_IRQHandler	PORTB_IRQHandler
#define HW_TFC_SPEED1_IRQn			PORTB_IRQn
#else
#endif

#endif // K64F12_SERIES

#ifdef K66F18_SERIES

#define HW_TFC_SWPIN_PORT			PORTB
#define HW_TFC_SWPIN_GPIO			GPIOB
#define HW_TFC_SWPIN_CLOCK			kCLOCK_PortB
#define HW_TFC_SWPIN_SW1			7
#define HW_TFC_SWPIN_SW2			6
#define HW_TFC_SWPIN_SW3			5
#define HW_TFC_SWPIN_SW4			4

#define HW_TFC_BUTPIN_PORT			PORTA
#define HW_TFC_BUTPIN_GPIO			GPIOA
#define HW_TFC_BUTPIN_CLOCK			kCLOCK_PortA
#define HW_TFC_BUTPIN_BUTA			26
#define HW_TFC_BUTPIN_BUTB			27

#define HW_TFC_LEDPIN_PORT			PORTA
#define HW_TFC_LEDPIN_GPIO			GPIOA
#define HW_TFC_LEDPIN_CLOCK			kCLOCK_PortA
#define HW_TFC_LEDPIN_LED1			6
#define HW_TFC_LEDPIN_LED2			7
#define HW_TFC_LEDPIN_LED3			8
#define HW_TFC_LEDPIN_LED4			9

#endif // K66F18_SERIES

#define _INPIN_MASK( b, pin, bit ) 	( ( b ) >> ( ( pin ) - ( bit ) ) & ( 1 << ( bit ) ) )
#define _OUTPIN_MASK( b, pin, bit ) ( ( ( b ) & ( 1 << ( bit ) ) ) << ( ( pin ) - ( bit ) ) )

void HW_TFC_IOPIN_Init()
{
	CLOCK_EnableClock( HW_TFC_SWPIN_CLOCK );
	CLOCK_EnableClock( HW_TFC_BUTPIN_CLOCK );
	CLOCK_EnableClock( HW_TFC_LEDPIN_CLOCK );

	PORT_SetPinMux( HW_TFC_SWPIN_PORT, HW_TFC_SWPIN_SW1, kPORT_MuxAsGpio );
	PORT_SetPinMux( HW_TFC_SWPIN_PORT, HW_TFC_SWPIN_SW2, kPORT_MuxAsGpio );
	PORT_SetPinMux( HW_TFC_SWPIN_PORT, HW_TFC_SWPIN_SW3, kPORT_MuxAsGpio );
	PORT_SetPinMux( HW_TFC_SWPIN_PORT, HW_TFC_SWPIN_SW4, kPORT_MuxAsGpio );
	PORT_SetPinMux( HW_TFC_BUTPIN_PORT, HW_TFC_BUTPIN_BUTA, kPORT_MuxAsGpio );
	PORT_SetPinMux( HW_TFC_BUTPIN_PORT, HW_TFC_BUTPIN_BUTB, kPORT_MuxAsGpio );

	PORT_SetPinMux( HW_TFC_LEDPIN_PORT, HW_TFC_LEDPIN_LED1, kPORT_MuxAsGpio );
	PORT_SetPinMux( HW_TFC_LEDPIN_PORT, HW_TFC_LEDPIN_LED2, kPORT_MuxAsGpio );
	PORT_SetPinMux( HW_TFC_LEDPIN_PORT, HW_TFC_LEDPIN_LED3, kPORT_MuxAsGpio );
	PORT_SetPinMux( HW_TFC_LEDPIN_PORT, HW_TFC_LEDPIN_LED4, kPORT_MuxAsGpio );

	gpio_pin_config_t pin_input =
	{ kGPIO_DigitalInput, 0 };

	GPIO_PinInit( HW_TFC_SWPIN_GPIO, HW_TFC_SWPIN_SW1, &pin_input );
	GPIO_PinInit( HW_TFC_SWPIN_GPIO, HW_TFC_SWPIN_SW2, &pin_input );
	GPIO_PinInit( HW_TFC_SWPIN_GPIO, HW_TFC_SWPIN_SW3, &pin_input );
	GPIO_PinInit( HW_TFC_SWPIN_GPIO, HW_TFC_SWPIN_SW4, &pin_input );
	GPIO_PinInit( HW_TFC_BUTPIN_GPIO, HW_TFC_BUTPIN_BUTA, &pin_input );
	GPIO_PinInit( HW_TFC_BUTPIN_GPIO, HW_TFC_BUTPIN_BUTB, &pin_input );

	gpio_pin_config_t pin_output =
	{ kGPIO_DigitalOutput, 0 };

	GPIO_PinInit( HW_TFC_LEDPIN_GPIO, HW_TFC_LEDPIN_LED1, &pin_output );
	GPIO_PinInit( HW_TFC_LEDPIN_GPIO, HW_TFC_LEDPIN_LED2, &pin_output );
	GPIO_PinInit( HW_TFC_LEDPIN_GPIO, HW_TFC_LEDPIN_LED3, &pin_output );
	GPIO_PinInit( HW_TFC_LEDPIN_GPIO, HW_TFC_LEDPIN_LED4, &pin_output );
}

uint32_t HW_TFC_INPIN_DIPSwitch()
{
	uint32_t b = HW_TFC_SWPIN_GPIO->PDIR;

	b = _INPIN_MASK( b, HW_TFC_SWPIN_SW1, 0 ) | _INPIN_MASK( b, HW_TFC_SWPIN_SW2, 1 ) |
		_INPIN_MASK( b, HW_TFC_SWPIN_SW3, 2 ) | _INPIN_MASK( b, HW_TFC_SWPIN_SW4, 3 );

	return b & 0xF;
}

uint32_t HW_TFC_INPIN_ABSwitch()
{
	uint32_t b = HW_TFC_BUTPIN_GPIO->PDIR;

	b = _INPIN_MASK( b, HW_TFC_BUTPIN_BUTA, 0 ) | _INPIN_MASK( b, HW_TFC_BUTPIN_BUTB, 1 );

	return b & 0x3;
}

void HW_TFC_OUTPIN_LED( uint32_t led, uint32_t level )
{
	static uint32_t leds_pin[ 4 ] = { HW_TFC_LEDPIN_LED1, HW_TFC_LEDPIN_LED2, HW_TFC_LEDPIN_LED3, HW_TFC_LEDPIN_LED4 };
	if ( led > 3 ) return;
	if ( level )
	{
		GPIO_PortSet( HW_TFC_LEDPIN_GPIO, 1 << leds_pin[ led ] );
	}
	else
	{
		GPIO_PortClear( HW_TFC_LEDPIN_GPIO, 1 << leds_pin[ led ] );
	}
}

void HW_TFC_OUTPIN_LEDs( uint32_t leds )
{
	uint32_t mask = ( 1 << HW_TFC_LEDPIN_LED1 ) | ( 1 << HW_TFC_LEDPIN_LED2 ) |
					( 1 << HW_TFC_LEDPIN_LED3 ) | ( 1 << HW_TFC_LEDPIN_LED4 );
	uint32_t set = _OUTPIN_MASK( leds, HW_TFC_LEDPIN_LED1, 0 ) | _OUTPIN_MASK( leds, HW_TFC_LEDPIN_LED2, 1 ) |
				   _OUTPIN_MASK( leds, HW_TFC_LEDPIN_LED3, 2 ) | _OUTPIN_MASK( leds, HW_TFC_LEDPIN_LED4, 3 );
	GPIO_PortSet( HW_TFC_LEDPIN_GPIO, set );
	GPIO_PortClear( HW_TFC_LEDPIN_GPIO, set ^ mask );
}

/** *************************************************************************
 * SPEED Pins for RC
 */

#if defined ( K64F12_SERIES ) && ( HW_TFC_PCB_VERSION_MAJOR == 1 )
#define HW_TFC_RC_FTM				FTM1				// FTM used for pulse with measuring

/**
 * Interrupt handler for pin state change
 */
extern "C" {
  void HW_TFC_SPEED0_IRQHandler();
  void HW_TFC_SPEED1_IRQHandler();
}

struct hw_tfc_speed_pin_irq_data_t
{
	GPIO_Type *gpio;
	uint32_t pin;
	uint32_t up_ms;
	uint32_t up_ticks;
	uint32_t width_us;
};

static volatile hw_tfc_speed_pin_irq_data_t _hw_tfc_speed_s[ 2 ] =
{
	{ HW_TFC_SPEED0_GPIO, HW_TFC_SPEED0_PIN, 0, 0, 0 },
	{ HW_TFC_SPEED1_GPIO, HW_TFC_SPEED1_PIN, 0, 0, 0 }
};

static uint32_t _HW_TFC_RC_1ms_ticks = 0;

void _HW_TFC_SPEED01_IRQHandler( volatile hw_tfc_speed_pin_irq_data_t *data );

void HW_TFC_SPEED0_IRQHandler()
{
	_HW_TFC_SPEED01_IRQHandler( _hw_tfc_speed_s );
}

void HW_TFC_SPEED1_IRQHandler()
{
	_HW_TFC_SPEED01_IRQHandler( _hw_tfc_speed_s + 1 );
}

void _HW_TFC_SPEED01_IRQHandler( volatile hw_tfc_speed_pin_irq_data_t *data )
{
	int32_t tmp_ticks = HW_TFC_RC_FTM->CNT;

	GPIO_ClearPinsInterruptFlags( data->gpio, 1U << data->pin );

	if ( GPIO_ReadPinInput( data->gpio, data->pin ) ) // pin == 1, rising edge
	{
		data->up_ms = _HW_TFC_Ticker_ms;
		data->up_ticks = tmp_ticks;
	}
	else // pin == 0, falling edge
	{
		if ( _HW_TFC_Ticker_ms - data->up_ms > 2 ) return; // ignore long time
		tmp_ticks -= data->up_ticks;
		if ( tmp_ticks < 0 ) tmp_ticks += 0xFFFF;
		data->width_us = 1000 * tmp_ticks  / _HW_TFC_RC_1ms_ticks;
	}
}

void HW_TFC_SPEEDPIN_Init()
{
	CLOCK_EnableClock( HW_TFC_SPEED0_CLOCK );
	CLOCK_EnableClock( HW_TFC_SPEED1_CLOCK );

	PORT_SetPinMux( HW_TFC_SPEED0_PORT, HW_TFC_SPEED0_PIN, kPORT_MuxAsGpio );
	PORT_SetPinMux( HW_TFC_SPEED1_PORT, HW_TFC_SPEED1_PIN, kPORT_MuxAsGpio );

	gpio_pin_config_t pin_input =
	{ kGPIO_DigitalInput, 0 };

	GPIO_PinInit( HW_TFC_SPEED0_GPIO, HW_TFC_SPEED0_PIN, &pin_input );
	GPIO_PinInit( HW_TFC_SPEED1_GPIO, HW_TFC_SPEED1_PIN, &pin_input );

    port_pin_config_t pin_conf;
    pin_conf.driveStrength = kPORT_LowDriveStrength;
    pin_conf.lockRegister = kPORT_UnlockRegister;
    pin_conf.mux = kPORT_MuxAsGpio;
    pin_conf.openDrainEnable = kPORT_OpenDrainDisable;
    pin_conf.passiveFilterEnable = kPORT_PassiveFilterDisable;
    pin_conf.pullSelect = kPORT_PullUp;
    pin_conf.slewRate = kPORT_SlowSlewRate;

    PORT_SetPinConfig( HW_TFC_SPEED0_PORT, HW_TFC_SPEED0_PIN, &pin_conf );
    PORT_SetPinConfig( HW_TFC_SPEED1_PORT, HW_TFC_SPEED1_PIN, &pin_conf );

}

void HW_TFC_RC_Init()
{
	HW_TFC_SPEEDPIN_Init();

	ftm_config_t ftmconf;

	FTM_GetDefaultConfig( &ftmconf );
	ftmconf.prescale = kFTM_Prescale_Divide_32;

	FTM_Init( HW_TFC_RC_FTM, &ftmconf);
	_HW_TFC_RC_1ms_ticks = MSEC_TO_COUNT( 1, CLOCK_GetBusClkFreq() / 32 );

	HW_TFC_RC_FTM->MOD = 0xFFFF;
}

void HW_TFC_RC_OnOff( uint32_t onoff )
{
	if ( onoff )
	{
	    FTM_StartTimer( HW_TFC_RC_FTM, kFTM_SystemClock );
	    PORT_SetPinInterruptConfig( HW_TFC_SPEED0_PORT, HW_TFC_SPEED0_PIN, kPORT_InterruptEitherEdge );
	    PORT_SetPinInterruptConfig( HW_TFC_SPEED1_PORT, HW_TFC_SPEED1_PIN, kPORT_InterruptEitherEdge );
	    EnableIRQ( HW_TFC_SPEED0_IRQn );
	    EnableIRQ( HW_TFC_SPEED1_IRQn );
	}
	else
	{
		FTM_StopTimer( HW_TFC_RC_FTM );
	    DisableIRQ( HW_TFC_SPEED0_IRQn );
	    DisableIRQ( HW_TFC_SPEED1_IRQn );
	    PORT_SetPinInterruptConfig( HW_TFC_SPEED0_PORT, HW_TFC_SPEED0_PIN, kPORT_InterruptOrDMADisabled );
	    PORT_SetPinInterruptConfig( HW_TFC_SPEED1_PORT, HW_TFC_SPEED1_PIN, kPORT_InterruptOrDMADisabled );
	}
}

uint32_t HW_TFC_RC_getPulse_us( uint32_t channel )
{
	uint32_t tmp = _hw_tfc_speed_s[ channel ].width_us;
	if ( tmp )
		_hw_tfc_speed_s[ channel ].width_us = 0;
	return tmp;
}

#else

#define HW_TFC_SPEED_FTM			FTM3			// selected FTM for both servos
#define HW_TFC_SPEED_IRQHandler		FTM3_IRQHandler // IRQ handler
#define HW_TFC_SPEED_IRQn			FTM3_IRQn		// IRQn
#define HW_TFC_SPEED_PORT			PORTD			// Port with both FTM_CHx
#define HW_TFC_SPEED_CLOCK			kCLOCK_PortD	// Clock gate for PORTD
#define HW_TFC_SPEED_PIN_ALTx		kPORT_MuxAlt4
#define HW_TFC_SPEED0_PIN			0				// Pin number for speed 0
#define HW_TFC_SPEED1_PIN			2				// Pin number for speed 1
#define HW_TFC_SPEED_CHNL_D0		kFTM_Chnl_0
#define HW_TFC_SPEED_CHNL_D1		kFTM_Chnl_1
#define HW_TFC_SPEED_CHNL_0			kFTM_Chnl_0
#define HW_TFC_SPEED_CHNL_1			kFTM_Chnl_1
#define HW_TFC_SPEED_CHNL_1_FLAG	kFTM_Chnl1Flag
#define HW_TFC_SPEED_CHNL_1_INTEN	kFTM_Chnl1InterruptEnable
#define HW_TFC_SPEED_CHNL_2			kFTM_Chnl_2
#define HW_TFC_SPEED_CHNL_3			kFTM_Chnl_3
#define HW_TFC_SPEED_CHNL_3_FLAG	kFTM_Chnl3Flag
#define HW_TFC_SPEED_CHNL_3_INTEN	kFTM_Chnl3InterruptEnable

static uint32_t _HW_TFC_RC_1ms_ticks = 0;

static volatile uint32_t _hw_tfc_speed_us[ 2 ] = { 0, 0 };

extern "C" { void HW_TFC_SPEED_IRQHandler(); }

void HW_TFC_SPEED_IRQHandler()
{
	uint32_t status = FTM_GetStatusFlags( HW_TFC_SPEED_FTM );
	if ( status & HW_TFC_SPEED_CHNL_1 )
	{
		uint32_t up_time = HW_TFC_SPEED_FTM->CONTROLS[ HW_TFC_SPEED_CHNL_0 ].CnV;
		uint32_t down_time = HW_TFC_SPEED_FTM->CONTROLS[ HW_TFC_SPEED_CHNL_1 ].CnV;
		_hw_tfc_speed_us[ 0 ] = ( ( up_time < down_time ) ? ( down_time - up_time ) : ( 0xFFFF - up_time + down_time ) ) * 1000 /*ms to us*/ / _HW_TFC_RC_1ms_ticks;
		FTM_ClearStatusFlags( HW_TFC_SPEED_FTM, HW_TFC_SPEED_CHNL_1_FLAG );
	}
	if ( status & HW_TFC_SPEED_CHNL_3 )
	{
		uint32_t up_time = HW_TFC_SPEED_FTM->CONTROLS[ HW_TFC_SPEED_CHNL_2 ].CnV;
		uint32_t down_time = HW_TFC_SPEED_FTM->CONTROLS[ HW_TFC_SPEED_CHNL_3 ].CnV;
		_hw_tfc_speed_us[ 1 ] = ( ( up_time < down_time ) ? ( down_time - up_time ) : ( 0xFFFF - up_time + down_time ) ) * 1000 /*ms to us*/ / _HW_TFC_RC_1ms_ticks;
		FTM_ClearStatusFlags( HW_TFC_SPEED_FTM, HW_TFC_SPEED_CHNL_3_FLAG );
	}
#ifdef SDK_OS_FREE_RTOS
	portBASE_TYPE *xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(RC_semaphore, xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
#endif
}


void HW_TFC_RC_Init()
{
    CLOCK_EnableClock( HW_TFC_SPEED_CLOCK );

    PORT_SetPinMux( HW_TFC_SPEED_PORT, HW_TFC_SPEED0_PIN, HW_TFC_SPEED_PIN_ALTx );
    PORT_SetPinMux( HW_TFC_SPEED_PORT, HW_TFC_SPEED1_PIN, HW_TFC_SPEED_PIN_ALTx );

    ftm_config_t ftmconf;
    //ftm_chnl_pwm_signal_param_t ftmpar;

	FTM_GetDefaultConfig( &ftmconf );
	ftmconf.prescale = kFTM_Prescale_Divide_32;

	FTM_Init( HW_TFC_SPEED_FTM, &ftmconf );

	HW_TFC_SPEED_FTM->MOD = 0xFFFF;
	_HW_TFC_RC_1ms_ticks = MSEC_TO_COUNT( 1, CLOCK_GetBusClkFreq() / 32 );

	ftm_dual_edge_capture_param_t dual_cap_par;
	dual_cap_par.mode = kFTM_Continuous;
	dual_cap_par.currChanEdgeMode = kFTM_RisingEdge;
	dual_cap_par.nextChanEdgeMode = kFTM_FallingEdge;
	FTM_SetupDualEdgeCapture( HW_TFC_SPEED_FTM, HW_TFC_SPEED_CHNL_D0, &dual_cap_par, 0  );
	FTM_SetupDualEdgeCapture( HW_TFC_SPEED_FTM, HW_TFC_SPEED_CHNL_D1, &dual_cap_par, 0  );
}

static int _s_hw_tfc_rc_is_on = 0;

void HW_TFC_RC_OnOff( uint32_t onoff )
{
	if ( !onoff == !_s_hw_tfc_rc_is_on ) return;

	_s_hw_tfc_rc_is_on = onoff;

	if ( onoff )
	{
		HW_TFC_SPEED_FTM->CNT = 0;
		EnableIRQ( HW_TFC_SPEED_IRQn );
		FTM_EnableInterrupts( HW_TFC_SPEED_FTM, HW_TFC_SPEED_CHNL_1_INTEN | HW_TFC_SPEED_CHNL_3_INTEN );
		FTM_StartTimer( HW_TFC_SPEED_FTM, kFTM_SystemClock );
	}
	else
	{
		FTM_StopTimer( HW_TFC_SPEED_FTM );
		FTM_DisableInterrupts( HW_TFC_SPEED_FTM, HW_TFC_SPEED_CHNL_1_INTEN | HW_TFC_SPEED_CHNL_3_INTEN );
		DisableIRQ( HW_TFC_SPEED_IRQn );
	}
}

uint32_t HW_TFC_RC_getPulse_us( uint32_t channel )
{
	if ( !_s_hw_tfc_rc_is_on ) return 0;

	channel = channel != 0;

	uint32_t tmp = _hw_tfc_speed_us[ channel ];
	if ( tmp )
		_hw_tfc_speed_us[ channel ] = 0;
	return tmp;
}


#endif

