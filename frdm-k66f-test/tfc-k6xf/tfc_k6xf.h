#ifndef __TFC_K6xF_H
#define __TFC_K6xF_H

/** @file 
  tfc_k64f.h 
*/

#define NL "\r\n"

#include <stdint.h>

#ifdef SDK_OS_FREE_RTOS

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

extern SemaphoreHandle_t RC_semaphore;
extern SemaphoreHandle_t Camera_semaphore;

#endif

/**
@name Motors
@{
*/

/**
 * @brief 	Initialization of both H-Bridges for motors.
 *
 * This function initializes two pairs of PWM channels.																		\n
 * The pins A1/A2 for the bridge/motor A and pins B1/B2 for the bridge/motor B are used on the board.						\n
 * One pin is also initialized to enable/disable both H-bridges.															\n
 */

void HW_TFC_MOTOR_Init();

/**
 * @brief 	Switch On or Off motors.
 *
 * This function enables or disables PWM of both motors.																	\n
 * When onoff is set to non-zero, function sets PWM of both motors to 0, starts FTM timer and both H-bridges are enabled.	\n
 * When onoff is set to zero, function sets PWM of both motors to 0, stops FTM timer and both H-bridges are disabled.		\n
 * Calling this function with the same value as the current status causes the call to be ignored.							\n
 *
 * @param onoff The Non-zero value - ON, the 0 value - OFF.
 */
void HW_TFC_MOTOR_OnOff( int onoff );

/**
 * @brief 	Set PWM power of motors A and B in range <-::TFC_PWM_MINMAX, ::TFC_PWM_MINMAX>.
 *
 * This function takes two values as arguments, where each belongs to one motor.										\n
 * If the value given is not within the allowed range, it is limited into range <-::TFC_PWM_MINMAX, ::TFC_PWM_MINMAX>.	\n
 * (For the safety reasons both values are limited into range <-::HW_TFC_PWM_MAX, ::HW_TFC_PWM_MAX>						\n
 * Selected value is used to calculate corresponding length of a PWM time period for each channel.						\n
 * PWM values are then set to the both pairs of channels of the timer.													\n
 *
 * @param motor_a 	Value for motor A.
 * @param motor_b	Value for motor B.
 */
void HW_TFC_MOTOR_SetPWM( int motor_a, int motor_b );

/**
* @}
*/

/**
@name Servos
@{
*/

/**
 * @brief 	Initialization of servos.
 *
 * This function initializes FTM timer and allows updating by a Software Trigger.					\n
 * The pins SERVO 0 and 1 are used on the board.													\n
 */
void HW_TFC_SERVO_Init();

/**
 * @brief	Switch On or Off servos.
 *
 * This function enables or disables FTM timer of both servos.															\n
 * When onoff is set to non-zero value, the function enables FTM timer.													\n
 * When onoff is set to 0, function enables interrupt, which disables the timer at the end of the current PWM period,
 * to prevent a distortion of the current pulse.																		\n
 * Calling this function with the same value as the current status causes the call to be ignored.						\n
 //
 *
 * @param onoff	The non-zero value - ON, the 0 value - OFF
 */
void HW_TFC_SERVO_OnOff( int onoff );

/**
 * @brief Set position of servo in microseconds.
 *
 * This function takes two arguments, first is used to determine which servo is being set.                           .	\n
 * Second parameter is value to set the position of the servo.															\n
 * If the value given is not within the allowed range, it is set into range
 * <-::TFC_SERVO_MAX_LR+::TFC_SERVO_DEFAULT_CENTER, ::TFC_SERVO_MAX_LR+::TFC_SERVO_DEFAULT_CENTER>.						\n
 * Selected value is used to calculate corresponding length of a PWM time period.										\n
 * PWM value is then assigned to selected servo's FTM timer channel.													\n
 *
 * @param servo The servo channel 0 or 1
 * @param pos 	Position of servo in microseconds.
 */
void HW_TFC_SERVO_Set( int servo, int pos );

/**
* @}
*/

/**
@name AD converter
@{
*/

/**
 * @brief	Initialization of AD converter for analog inputs.
 * 
 * This function initializes ADC, which reads values of potentiometers, H-bridge feedbacks and battery voltage.	\n
 */
void HW_TFC_ANDATA_Init();

/**
 * @brief 	Switch On or Off the AD conversion.
 * 
 * This function enables or disables interrupt, which starts the ADC and converts all values.			\n
 * Calling this function with the same value as the current status causes the call to be ignored.			\n
 *
 * @param onoff The non-zero value - ON, the 0 value - OFF
 */
void HW_TFC_ANDATA_OnOff( int onoff );

/**
 * @brief 	Get current value of selected analog channel.
 *
 * @param chnl	Selected channel.
 * @return		Value of the analog channel.
 */
uint16_t HW_TFC_ANDATA_getVal( uint32_t chnl );

/**
* @}
*/

/**
@name Cameras
@{
*/

/**
 * @brief	Initialization of the camera interface.
 *
 * This function initializes ADC for cameras.
 * The pins CAMERA 0 and 1 are used on the board.
 */
void HW_TFC_CAMERA_Init();

/**
 * @brief Switch cameras On or Off
 *
 * This function enables or disables interrupt, which starts the ADC conversion of pixels [::TFC_CAMERA_LINE_LENGTH] for both cameras.		\n
 * Calling this function with the same value as the current status causes the call to be ignored.											\n
 *
 * @param onoff The non-zero value - ON, the 0 value - OFF
 */
void HW_TFC_CAMERA_OnOff( int onoff );

/**
 * @brief Function to check whether the image is processed or not.

 * @return 0 - not ready, everything else - ready
 */
uint32_t HW_TFC_CAMERA_isReady();

/**
 * @brief Function to copy camera image of selected camera from internal buffer into array.
 *
 * @param chnl A camera channel.
 * @param line Pointer to an array, where the picture should be stored.
 * @param len Size of an array, where the image is being loaded.
 */
void HW_TFC_CAMERA_getImage( int chnl, uint16_t *line, int len );

/**
* @}
*/
///Number of up/down counters.
#define HW_TFC_NUM_TICKERS				4
///Timer counters, counting up.
extern volatile uint32_t HW_TFC_Ticker_Up[HW_TFC_NUM_TICKERS];
///Timer counters, counting down.
extern volatile uint32_t HW_TFC_Ticker_Down[HW_TFC_NUM_TICKERS];
///Number of captured camera images.
extern volatile uint32_t HW_TFC_TimeStamp;


/**
@name Timer
@{
*/

/**
 * @brief Initialization of timer.
 * 
 * This function initializes FTM timer, which ensures fluent run of the program.
 * The camera and analog values ADCs are being enabled periodically.
 */
void HW_TFC_Ticker_Init();

/**
 * @brief Switch On or Off the timer.
 *
 * @param onoff The non-zero value - ON, the 0 value - OFF
 */
void HW_TFC_Ticker_OnOff( int onoff );

/**
* @}
*/

/**
@name Ports & Pins
@{
*/

/**
 * @brief Initialization of PORTs and Pins.
 */
void HW_TFC_IOPIN_Init();

/**
 * @brief Get state of DIP switches.
 * @return Value where last four bits represents state of DIP switches.
 */
uint32_t HW_TFC_INPIN_DIPSwitch();

/**
 * @brief Get state of push buttons.
 * @return Value where last two bits represents the state of push switches.
 */
uint32_t HW_TFC_INPIN_ABSwitch();

/**
 * @brief Settings of LEDs 1-4
 * @param leds Low nibble specify state of four LEDs
 */
void HW_TFC_OUTPIN_LEDs( uint32_t leds );

/**
* @brief Settings of individual LEDs.
* @param led LED ID <0,3>
* @param level Logical 1 - ON, logical 0 - OFF
*/

void HW_TFC_OUTPIN_LED( uint32_t led, uint32_t level );

/**
* @}
*/

/**
@name RC Speed pins
@{
*/

/**
* @brief Initialization of SPEED pins and FTM timer.

* The pins SPEED 0 and 1 are used on the board.
*/
void HW_TFC_RC_Init();

/**
* @brief Function to turn RC control on or off.
*
* This function enables/disables FTM timer and interrupts.
* Calling this function with the same value as the current status causes the call to be ignored.	\n
*
*  @param onoff The non-zero value - ON, the 0 value - OFF
*/
void HW_TFC_RC_OnOff( uint32_t onoff );

/**
* @brief Function to get the width of pulse.
* @param channel Channel of SPEED pins
* @return Width of pulse in microseconds.
*/
uint32_t HW_TFC_RC_getPulse_us( uint32_t channel );

/**
* @}
*/

#endif //TFC_K6xF_H

