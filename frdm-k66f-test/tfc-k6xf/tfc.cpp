/*
*/

#include <tfc.h>
#include <tfc_k6xf.h>
#include "board.h"
#include "fsl_debug_console.h"

void TFC::InitAll()
{
	HW_TFC_Ticker_Init();
	HW_TFC_IOPIN_Init();
	HW_TFC_SERVO_Init();
	HW_TFC_MOTOR_Init();
	HW_TFC_ANDATA_Init();
	HW_TFC_CAMERA_Init();

	HW_TFC_Ticker_OnOff( 1 );

	m_setting.pwm_max = TFC_PWM_DEFAULT_MAX;
	m_setting.servo_center[ 0 ] = TFC_SERVO_DEFAULT_CENTER;
	m_setting.servo_center[ 1 ] = TFC_SERVO_DEFAULT_CENTER;
	m_setting.servo_max_lr[ 0 ] = TFC_SERVO_DEFAULT_MAX_LR;
	m_setting.servo_max_lr[ 1 ] = TFC_SERVO_DEFAULT_MAX_LR;

	bzero( &m_control, sizeof( m_control ) );
}

/****************************************************************************
 * Digital Input and Output
 * LEDs & Switches
 */

void TFC::setLED( uint32_t led, uint32_t val )
{
	if ( val )
		m_control.leds |= 1 << led;
	else
		m_control.leds &= ~( 1 << led );

	HW_TFC_OUTPIN_LED( led, val );
}

void TFC::setLEDs( uint32_t leds )
{
	m_control.leds = leds;
	HW_TFC_OUTPIN_LEDs( leds );
}

void TFC::setBatteryLEDLevel( uint32_t bat_level )
{
	if ( bat_level > 4 ) bat_level = 0;
	uint32_t leds = 0x0F >> ( 4 - bat_level );
	setLEDs( leds );
}

uint32_t TFC::getDIPSwitch()
{
	return HW_TFC_INPIN_DIPSwitch();
}

uint32_t TFC::getPushButton( uint32_t channel )
{
	if ( !channel )
		return ( HW_TFC_INPIN_ABSwitch() & 1 ) != 0;
	else
		return ( HW_TFC_INPIN_ABSwitch() & 2 ) != 0;
}

/****************************************************************************
 * Analog Values
 */

uint32_t TFC::ReadADC( uint32_t andata_chnl )
{
	return HW_TFC_ANDATA_getVal( andata_chnl );
}

// return value in range <-ANDATA_MINMAX,ANDATA_MINMAX>
// result have to accept clockwise rotation of potentiometers!
int32_t TFC::ReadPot_i( uint32_t Channel )
{
    int val = ReadADC( anPOT_1 + ( Channel != 0 ) );
    return - ( val * 2 * TFC_ANDATA_MINMAX / TFC_ADC_MAXVAL - TFC_ANDATA_MINMAX );
}

// return value in range <-1.0, 1.0>
float TFC::ReadPot_f( uint32_t Channel )
{
    return ( ( float ) TFC::ReadPot_i( Channel ) ) / TFC_ANDATA_MINMAX;
}

// return value in range <0,ANDATA_MINMAX>
uint32_t TFC::ReadFB_i( uint32_t Channel )
{
	int val = ReadADC( anFB_A + ( Channel != 0 ) );
	return val * TFC_ANDATA_MINMAX / TFC_ADC_MAXVAL;
}

// return current in Amperes
float TFC::ReadFB_f( uint32_t Channel )
{
    int val = ReadADC( anFB_A + ( Channel != 0 ) );
    // Power Supply 3.3V
    // Current FB=16mA ~ Output=6A
    // Resistor 220Ohm
    return ( ( float ) val ) / TFC_ADC_MAXVAL * 3.3 / 220.0 / 0.016 * 6;
}

// return value in range <0-ANDATA_MINMAX>
uint32_t TFC::ReadBatteryVoltage_i()
{
    int val = ReadADC( anBAT );
    return val * TFC_ANDATA_MINMAX / TFC_ADC_MAXVAL;
}

// return voltage in Volts
float TFC::ReadBatteryVoltage_f()
{
    int val = ReadADC( anBAT );
    return ( ( float ) val ) / TFC_ADC_MAXVAL * 3.3 * 5.7; // * ((47000.0+10000.0)/10000.0);
}

/****************************************************************************
 * Camera
 */

uint32_t TFC::ImageReady( uint32_t channel )
{
	return HW_TFC_CAMERA_isReady() & ( 1 << ( channel != 0 ) );
}

void TFC::getImage( uint32_t channel, uint16_t *img, uint32_t length )
{
	HW_TFC_CAMERA_getImage( channel != 0, img, length );
}

/****************************************************************************
 * Servo and Motors
 */

void TFC::setServoCalibration( uint32_t channel, uint32_t center, uint32_t max_lr )
{
	m_setting.servo_center[ channel != 0 ] =
			MIN( MAX( center, TFC_SERVO_DEFAULT_CENTER - TFC_SERVO_MAX_LR ), TFC_SERVO_DEFAULT_CENTER + TFC_SERVO_MAX_LR );
	m_setting.servo_max_lr[ channel != 0 ] = MIN( max_lr, TFC_SERVO_MAX_LR );
}

// set maximal PWM duty cycle
void TFC::setPWMMax( uint32_t max )
{
	m_setting.pwm_max = MIN( max, TFC_PWM_MINMAX );
}


void TFC::ServoOnOff( uint32_t onoff )
{
	m_control.servo_onoff = onoff;
	HW_TFC_SERVO_OnOff( onoff );
}

void TFC::setServo_i( uint32_t channel, int32_t position )
{
	channel = channel != 0;
	int32_t servo_center = m_setting.servo_center[ channel ];
	int32_t servo_max_lr = m_setting.servo_max_lr[ channel ];
	position = MIN( MAX( position,  -TFC_SERVO_MINMAX ), TFC_SERVO_MINMAX );

	uint32_t usec = servo_center + position * servo_max_lr / TFC_SERVO_MINMAX;
	HW_TFC_SERVO_Set( channel, usec );

	m_control.servo_pos[ channel ] = position;
}

int32_t TFC::getServo_i( uint32_t channel )
{
	return m_control.servo_pos[ channel != 0 ];
}

void TFC::setServo_f( uint32_t channel, float position )
{
	setServo_i( channel, position * TFC_SERVO_MINMAX );
}

void TFC::MotorPWMOnOff( uint32_t onoff )
{
	HW_TFC_MOTOR_OnOff( onoff );

	m_control.pwm_onoff = onoff;
}

void TFC::setMotorPWM_i( int32_t pwm_a, int32_t pwm_b )
{
	pwm_a = MIN( MAX( pwm_a, -m_setting.pwm_max ), m_setting.pwm_max );
	pwm_b = MIN( MAX( pwm_b, -m_setting.pwm_max ), m_setting.pwm_max );

	HW_TFC_MOTOR_SetPWM( pwm_a, pwm_b );

	m_control.pwm[ 0 ] = pwm_a;
	m_control.pwm[ 1 ] = pwm_b;
}

int32_t TFC::getMotorPWM_i( uint32_t channel )
{
	return m_control.pwm[ channel != 0 ];
}

void TFC::setMotorPWM_f( float a_pwm, float b_pwm )
{
	setMotorPWM_i( a_pwm * TFC_PWM_MINMAX, b_pwm * TFC_PWM_MINMAX );
}

/****************************************************************************
 * RC
 */

// initialization of SPEED0 and SPEED1 pins for RC
void TFC::InitRC()
{
	HW_TFC_RC_Init();
}

// enable/disable pulses detection on SPEED0 and SPEED1 pins
void TFC::RCOnOff( uint32_t onoff )
{
	HW_TFC_RC_OnOff( onoff );
}

// get last pulse width on channel 0 or 1
uint32_t TFC::getRCPulse( uint32_t channel )
{
	return HW_TFC_RC_getPulse_us( channel != 0 );
}


