/**
@file tfc.h
*/

#ifndef __TFC_H
#define __TFC_H

#include <stdint.h>

#define TFC_VERSION "20171213"

#define __TFC_EMBEDED__ 1

/** Resolution of the camera. */
#define TFC_CAMERA_LINE_LENGTH		128			

/** Specifies the integer range for PWM, as in <-::TFC_PWM_MINMAX, ::TFC_PWM_MINMAX>, which is used in this application's methods 
	TFC::getMotorPWM_i() and TFC::setMotorPWM_i(). 
*/
#define TFC_PWM_MINMAX				1000			

/** Specifies the integer range for servo, as in <-::TFC_SERVO_MINMAX, ::TFC_SERVO_MINMAX>, which is used in this application's methods 
	TFC::getServo_i() and TFC::setServo_i(). 
*/
#define TFC_SERVO_MINMAX			1000			

/** Maximum value of analog sample obtained from ADC by the method TFC::ReadADC(). */
#define TFC_ADC_MAXVAL				0x0FFF			

/** Specifies the integer ranges for analog values, as in <-::TFC_ANDATA_MINMAX, ::TFC_ANDATA_MINMAX> or <0, ::TFC_ANDATA_MINMAX>, which are used in this application's methods
	TFC::ReadPot_i(), TFC::ReadFB_i() and TFC::ReadBatteryVoltage_i().
*/
#define TFC_ANDATA_MINMAX			1000			

/** Default center position of servos - pulse width in microseconds. */
#define TFC_SERVO_DEFAULT_CENTER	1500		

/** Specifies the default offset from ::TFC_SERVO_DEFAULT_CENTER, which corresponds to fully turned wheels to either side. */
#define TFC_SERVO_DEFAULT_MAX_LR	200			

/** Specifies the maximal offset from the servos' center, which can be used in calibration by method TFC::setServoCalibration(). */
#define TFC_SERVO_MAX_LR			400			

/** Specifies the default integer range for PWM, as in <-::TFC_PWM_DEFAULT_MAX, ::TFC_PWM_DEFAULT_MAX>, which can be changed in the method
	TFC::setPWMMax() up to <-::TFC_PWM_MINMAX, ::TFC_PWM_MINMAX>. 
*/
#define TFC_PWM_DEFAULT_MAX			200		

/** Maximal allowed -/+ PWM duty cycle for the underlying HW. */
#define HW_TFC_PWM_MAX				1000

/** Minimal allowed length of pulse for steering servo for the underlying HW */
#define HW_TFC_SERVO_MIN			500
/** Maximal allowed length of pulse for steering servo  for the underlying HW*/
#define HW_TFC_SERVO_MAX		    2500

/** Command used in a packet for sending/receiving data. */
#define CMD_DATA					1

/** Command used in a packet for calibration. */
#define CMD_SETTING					2

/** Command used in a packet for controlling features of the board or receiving data from them. */
#define CMD_CONTROL					3

/** Packet header. */
#define STX							0x2
/** Packet footer. */
#define ETX							0x3

/** @brief Order of analog data in an array. These values are used as a parameter for a method TFC::ReadADC(). */
enum tfc_andata_chnl_enum 
{ 
	anPOT_1,	///< Potentiometer 1.
	anPOT_2,	///< Potentiometer 2.
	anFB_A,		///< Feedback A from the H-Bridge.
	anFB_B,		///< Feedback B from the H-Bridge.
	anBAT,		///< Battery voltage.
	anLast		///< Last member used as a size of an array.
};


/** 
@brief Empty data packet

This packet contains only necessary parameters, no data. \n
When using variation of this packet to send/receive data, the data should be put in a place of ::data. \n
There can be some of the data structure (::tfc_data_s, ::tfc_setting_s or ::tfc_control_s) in one packet.

@code
  Protocol specification:
    length = 1 + 1 + 2 + N + 1
    bytes	 1	      2        1          N         1
    types  uint_8  uint16_6  uint8_t   N*uint8_t  uint8_t
   purpose  STX     length   CMD_xx     struct      ETX
@endcode
*/
#pragma pack(push,1)
struct tfc_protocol_empty_s
{
	uint8_t stx;	///< Each packet must start with the ::STX byte.
	uint16_t length;///< Length of the whole packet.
	uint8_t cmd;	///< Type of command, can be either	::CMD_DATA, ::CMD_SETTING or ::CMD_CONTROL.
	char data[0];	///< Placeholder for a data.
	uint8_t etx;	///< Each packet must end with the ::ETX byte.
};

/**
@brief Structure containing analog data and data from camera.

This data structure contains data corresponding to a certain timestamp. It is s usually sent in packet (::tfc_protocol_empty_s) with ::CMD_DATA. \n
*/
struct tfc_data_s
{
	uint32_t	timestamp;						///< Number of the sample.
	uint16_t	adc[ anLast ];					///< All analog data specified in ::tfc_andata_chnl_enum.
	uint8_t		dip_sw;							///< Values of DIP switches.
	uint8_t		push_sw;						///< Values of push buttons.
	uint16_t	image[ TFC_CAMERA_LINE_LENGTH ];///< One line from the camera.
	uint32_t	_padding;						///< Padding bytes.
};

/**
@brief Structure for calibration of the individual features.

This data structure is usually sent in packet (::tfc_protocol_empty_s) with ::CMD_SETTING in the beginning of an application. \n
*/
struct tfc_setting_s
{
	uint16_t	servo_center[ 2 ];			///< Center of the servos.
	uint16_t	servo_max_lr[ 2 ];			///< Offset from the center of the servos corresponding to their maximal rotation.
	uint16_t	pwm_max;					///< Maximal PWM for motors.
	uint16_t	_padding;					///< Padding bytes.
};

/**
@brief Structure for controlling individual features or receiving their data.

This data structure is usually sent in packet (::tfc_protocol_empty_s) with ::CMD_CONTROL. \n
*/
struct tfc_control_s
{
	uint8_t		leds;						///< Values of the 4 LEDs in the form of a low nibble.
	uint8_t 	pwm_onoff;					///< On/off value of the motors {0,1}.
	uint8_t 	servo_onoff;				///< On/off value of the servos {0,1}.
	uint8_t 	_padding1;					///< Padding byte.
	int16_t 	pwm[ 2 ];					///< PWM values of the motors <-::TFC_PWM_MINMAX, ::TFC_PWM_MINMAX>.
	int16_t 	servo_pos[ 2 ];				///< Positions of the servos <-::TFC_SERVO_MAX_LR, ::TFC_SERVO_MAX_LR>.
};
#pragma pack(pop)

/**
 * @brief Main class for controlling all the features.
 */
class TFC
{
public:
	TFC() {}

	/**
	 * Initialize all variables to it's default values and initialize all HW parts.
	 */
	void InitAll();

	/** @name LEDs
	 These methods are used to control LEDs.\n
	 */
	///@{

	/**
	 * Set a single LED.
	 * @param led The LED number in the range <0,3>.
	 * @param val The LED setting On/Off.
	*/
	void setLED( uint32_t led, uint32_t val );

	/**
	 * Set multiple LEDs.
	 * @param leds 4 LEDs in the form of a low nibble (the lowest 4 bits of the variable), where each bit value {0,1} turns on/off its corresponding LEDs.
	 */
	void setLEDs( uint32_t leds );

	/**
	 * Set LEDs corresponding to the battery level.
	 * @param bat_level Value in range <0,4>.
	 */
	void setBatteryLEDLevel( uint32_t bat_level );
	///@}


	/** @name Buttons and DIP switch
	 These methods are used to obtain values from the DIP switches or push buttons.\n
	 */
	///@{

	/**
	* Get current value of the DIP switches.
	* @return The four bit value of the DIP switches.
	*/
	uint32_t getDIPSwitch();
	
	/**
	 * Get current value of a push button.
	 * @param channel Corresponds to a button, 0 - the first button, any other value - the second button.
	 * @return Current state of a button 0 - released, 1 - pressed.
	 */
	uint32_t getPushButton( uint32_t channel );
	///@}


	/**
	 * Get analog value directly from the ADC.
	 * @param andata_chnl Type of a analog value to get. It can be any value from ::tfc_andata_chnl_enum.
	 * @return Value corresponding to the selected channel in the range <0, ::TFC_ADC_MAXVAL>.
	 */

	uint32_t ReadADC( uint32_t andata_chnl );

	/** @name Potentiometers
	 These methods are used to obtain current position of the potentiometers.\n
	 @code
	 The potentiometers have their own value range,                     +-------------------------|-------------------------+
	 which looks like this:                                             0                    "Midpoint"                    MAX
	                                                                    |                         |                         |
	 When the value is obtained from the ADC,                           +-------------------------|-------------------------+
	 the range is translated to this:                                   0                 TFC_ADC_MAXVAL/2          TFC_ADC_MAXVAL
	                                                                    |                         |                         |
	 The ReadPot_i() method translates the ADC values                   +-------------------------|-------------------------+
	 to this range:                                             -TFC_ANDATA_MINMAX                0               TFC_ANDATA_MINMAX
	                                                                    |                         |                         |
	 The ReadPot_f() method further translates                          +-------------------------|-------------------------+
	 the value to this range:                                         -1.0                        0                        1.0
	 @endcode
	 */
	///@{

	/**
	 * Get integer value of a potentiometer.

	 * @param Channel 0 - the first potentiometer, any other value - the second potentiometer.
	 * @return Current value in range <-::TFC_ANDATA_MINMAX , ::TFC_ANDATA_MINMAX >.
	 */
	int32_t ReadPot_i( uint32_t Channel );

	/**
	 * Get float value of a potentiometer. \n

	 * @param Channel 0 - the first potentiometer, any other value - the second potentiometer.
	 * @return Value in range <-1.0, 1.0>.
	 */
	float ReadPot_f( uint32_t Channel );
	///@}


	/** @name Feedback
	 These methods are used to obtain current a H-Bridge feedback value.\n

	 @code
	 When the value of the feedback is obtained                         +---------------------------------------------------+
	 from the ADC, the range looks like this:                           0                                            TFC_ADC_MAXVAL
	                                                                    |                                                   |
	 The ReadFB_i() method translates the ADC values                    +---------------------------------------------------+
	 to this range:                                                     0                                         TFC_ANDATA_MINMAX
	                                                                    |                                                   |
	 The ReadFB_f() method further translates                           +---------------------------------------------------+
	 the value to the current electrical current in Amps:               0                                               5.625 [Amp]
	 @endcode
	 */
	///@{

	/**
	* Get integer feedback value.
	* @param Channel 0 - the first FB channel, any other value - the second FB channel.
	* @return Value in range <0, ::TFC_ANDATA_MINMAX>.
	*/
	uint32_t ReadFB_i( uint32_t Channel );

	/**
	* Get feedback value as current in Amperes.
	* @param Channel 0 - the first FB channel, any other value - the second FB channel.
	* @return Current in Amperes.
	*/
	float ReadFB_f( uint32_t Channel );
	///@}


	/** @name Battery
	 These methods are used to obtain the current battery voltage.\n

	 @code
	 When the voltage of the battery is obtained                        +---------------------------------------------------+
	 from the ADC, the range looks like this:                           0                                          TFC_ADC_MAXVAL
	                                                                    |                                                   |
	 The ReadBatteryVoltage_i() method translates                       +---------------------------------------------------+
	 the ADC values to this range:                                      0                                        TFC_ANDATA_MINMAX
	                                                                    |                                                   |
	 The ReadBatteryVoltage_f() method further translates               +---------------------------------------------------+
	 the value to the voltage of the battery:                           0                                                18.81 [V]
	 @endcode
	 */
	///@{

	/**
	 * Get integer representation of the voltage of the battery.
	 * @return Value in range <0, ::TFC_ANDATA_MINMAX>.
	 */
	uint32_t ReadBatteryVoltage_i();

	/**
	 * Get voltage of battery.
	 * @return Value in Volts.
	 */
	float ReadBatteryVoltage_f();
	///@}


	/** @name Camera
	 These methods are used to obtain data from a camera.\n
	 */
	///@{

	/**
	 * Check whether the camera image is ready or not.
	 * @param channel 0 - the first camera, any other value - the optional second camera.
	 * @return 0 - the image is not ready, 1 - the image is ready.
	 */
	uint32_t ImageReady( uint32_t channel );

	/**
	 * Get image (line) from a camera.
	 * @param channel 0 - the first camera, any other value - the optional second camera.
	 * @param img Pointer to an uint16_t array representing the line image. Data in range <0, TFC_ADC_MAXVAL>
	 * @param length Size of the array, should be ::TFC_CAMERA_LINE_LENGTH.
	 */

	void getImage( uint32_t channel, uint16_t *img, uint32_t length );
	///@}


	/** @name Servos
	 These methods are used to control servos.\n

	 @code
	 The servo are controlled by PWM, specifically
	 by the pulse width duration in microseconds,                       +-------------------------|-------------------------+
	 so their value range looks by default like this:      (TFC_SERVO_DEFAULT_CENTER  (TFC_SERVO_DEFAULT_CENTER)  (TFC_SERVO_DEFAULT_CENTER
	 (or it can be calibrated by setServoCalibration())    -TFC_SERVO_DEFAULT_MAX_LR)                             +TFC_SERVO_DEFAULT_MAX_LR)
	                                                                    |                         |                         |
	 The methods getServo_i() and setServo_i()                          +-------------------------|-------------------------+
	 use value in this range:                                    -TFC_SERVO_MINMAX                0                  TFC_SERVO_MINMAX
	                                                                    |                         |                         |
	 The setServo_f() method further translates                         +-------------------------|-------------------------+
	 the values to this range:                                        -1.0                        0                        1.0
	 @endcode

	 */
	///@{

	/**
	 * Servo calibration.
	 * @param channel Channel 0 - the first servo, any other value - the second servo.
	 * @param center Pulse width in microseconds corresponding to the center position of the servo.
	 * @param max_lr Offset from center to either side corresponding to the maximal rotation, in the form of a pulse width in microseconds.
	 */

	void setServoCalibration( uint32_t channel, uint32_t center, uint32_t max_lr );
	
	/**
	 * Enable or disable both servos.
	 * @param onoff Disable or enable servos {0, 1}.
	 */
	void ServoOnOff( uint32_t onoff );

	/**
	 * Set servo position.
	 * @param channel Channel 0 - the first servo, any other value - the second servo.
	 * @param position Position in range <-::TFC_SERVO_MINMAX , ::TFC_SERVO_MINMAX>.
	 */

	void setServo_i( uint32_t channel, int32_t position );

	/**
	 * Get current position of servo.
	 * @param channel Channel 0 - the first servo, any other value - the second servo.
	 * @return Value in range <- ::TFC_SERVO_MINMAX , ::TFC_SERVO_MINMAX>.
	 */
	int32_t getServo_i( uint32_t channel );

	/**
	 * Set servo position.
	 * @param channel Channel 0 - the first servo, any other value - the second servo.
	 * @param position Value in range <-1.0, 1.0>.
	 */
	void setServo_f( uint32_t channel, float position );
	///@}


	/** @name Motors
	 These methods are used to control the power and the direction of rotation of motors.\n

	 @code
	 The motors are controlled by PWM, so their
	 duty cycle range looks like this:                                  +-------------------------|-------------------------+
	 (negative values correspond to reverse direction):               -100%                       0%                       100%
	                                                                    |                         |                         |
	 The methods getMotorPWM_i() and setMotorPWM_i()                    +-------------------------|-------------------------+
	 use value in this range:                                    -TFC_PWM_MINMAX                  0                  TFC_PWM_MINMAX
	                                                                    |                         |                         |
	 The setMotorPWM_f() method further translates                      +-------------------------|-------------------------+
	 the values to this range:                                        -1.0                        0                        1.0       
	 @endcode

	 */
	///@{

	/**
	 * Set maximal PWM duty cycle for the motors (for safety reasons).
	 * This limits the power of motors to the given value, so when the maximal value is set by this method for example to 300 and then the method
	 * TFC::setMotorPWM_i() will be called with values greater than this one, the motors power will be reduced to just 300.
	 * @param max The maximum width of pulse in the range <0, ::TFC_PWM_MINMAX>.
	 */
	void setPWMMax( uint32_t max );

	/**
	 * Enable or disable both motors.
	 * @param onoff Disable or enable motors {0,1}.
	 */
	void MotorPWMOnOff( uint32_t onoff );

	/**
	 * Set PWM for both motors in range <-::TFC_PWM_MINMAX , ::TFC_PWM_MINMAX>.
	 * If the values are greater than the maximum set by the method TFC::setPWMMax(), the motors will only run at that set maximum.
	 * @param pwm_a Value for the first motor.
	 * @param pwm_b Value for the second motor.
	 */
	void setMotorPWM_i( int32_t pwm_a, int32_t pwm_b );

	/**
	 * Get the current PWM integer value of a motor.
	 * @param channel_ab Channel 0 - the first motor, any other value - the second motor.
	 * @return Value in range <-::TFC_PWM_MINMAX , ::TFC_PWM_MINMAX>.
	 */
	int32_t getMotorPWM_i( uint32_t channel_ab );

	/**
	 * Set PWM of both motors in range <-1.0, 1.0>.
	 * @param pwm_a Value for the first motor.
	 * @param pwm_b Value for the second motor.
	 */
	void setMotorPWM_f( float pwm_a, float pwm_b );
	///@}

	/** The instance variables The calibration of the servos and the set motor limit. */
	tfc_setting_s m_setting;

	/** The instance variable used to store current values of LEDs, servos and motors. */
	tfc_control_s m_control;

#ifdef __TFC_EMBEDED__

	/** @name Remote Control
	 These methods are used to control and obtain data from the remote control receiver.
	*/
	///@{

	/**
	 * Initialize SPEED0 and SPEED1 pins used for RC.
	 */
	void InitRC();

	/**
	 * Enable or disable pulses detection on SPEED0 and SPEED1 pins.
	 * @param onoff 0 for disable 1 for enable.
	 */
	void RCOnOff( uint32_t onoff );


	/**
	 * Get last pulse width from the RC. By reading the value is always reset.
	 * @param channel Specify from which channel to get the pulse width {0,1}.
	 * @return The pulse width in microseconds.
	 */
	uint32_t getRCPulse( uint32_t channel );
	///@}


#else

	void setData( s_data *data );
	s_setting *getSetting();
	s_control *getControl();

protected:

	tfc_data_s m_data;
    int m_data_ready;

#endif // __TFC_EMBEDED__

};

#endif // __TFC_H
