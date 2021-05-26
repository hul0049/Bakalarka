/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    frdm-k66f-test.cpp
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"
#include "fsl_debug_console.h"
#include "tfc_k6xf.h"
#include "tfc.h"
#include "enet.h"



/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

TFC tfc;
Enet enet;

struct WifiFrame
{
        uint32_t timestamp;                        	// number of sample
        int16_t        servo;     			// -1000/1000
        int16_t        pwm[ 2 ];     			// -1000/1000
        int16_t        fb[ 2 ];         		// 0-1000
        int16_t		   dc[ 2 ];
        int16_t		   speed;
        uint16_t       image[ TFC_CAMERA_LINE_LENGTH ];	// line camera image
} __attribute__((packed));

#define UDP_SEND_DATA 1

#define DIAG_PRINTF PRINTF

#define LED_CENTER( pot ) ( ( pot ) < -5 ? 0x01 : ( ( pot ) > 5 ? 0x2 : 0x03 ) )

void Diagnostic() {
	tfc.ServoOnOff(1);

	switch (tfc.getDIPSwitch() >> 1) {
	case 0: // Switch A-B & Potentiometers & test LEDs
	{
		tfc.setServo_i(0, 0);
		tfc.MotorPWMOnOff(0);

		DIAG_PRINTF("SwA: %d SwB: %d Pot1: %5d Pot2: %5d BAT: %5.2f" NL,
				tfc.getPushButton(0), tfc.getPushButton(1), tfc.ReadPot_i(0),
				tfc.ReadPot_i(1), tfc.ReadBatteryVoltage_f());
		uint8_t bat = 0;
		if (tfc.getPushButton(0))
			bat |= 0x3;
		if (tfc.getPushButton(1))
			bat |= 0xC;
		tfc.setLEDs(bat);
		break;
	}
	case 1: // Test Servo
	{
		tfc.MotorPWMOnOff(0);
		//tfc.setLEDs( 0 );

		int pa = tfc.ReadPot_i(0);
		int pb = tfc.ReadPot_i(1);

		tfc.setLEDs( LED_CENTER( pa ) | ( LED_CENTER( pb ) << 2));
		DIAG_PRINTF("DutyCycle: %4d" NL, TFC_SERVO_DEFAULT_CENTER + pa);

		tfc_setting_s curset = tfc.m_setting;
		tfc.m_setting.servo_center[0] = TFC_SERVO_DEFAULT_CENTER + pa;
		tfc.m_setting.servo_max_lr[0] = TFC_SERVO_MAX_LR;

		tfc.setServo_i(0, 0);

		tfc.m_setting = curset;

		break;
	}
	case 2: // Test Motors
	{
		tfc.setServo_i(0, 0);
		tfc.MotorPWMOnOff(1);
		tfc.setLEDs(0);

		int pa = tfc.ReadPot_i(0);
		int pb = tfc.ReadPot_i(1);
		tfc.setLEDs( LED_CENTER( pa ) | ( LED_CENTER( pb ) << 2));
		DIAG_PRINTF("Pot1: %5d Pot2: %5d FB-A: %4d FB-B: %4d" NL, pa, pb,
				tfc.ReadFB_i(0), tfc.ReadFB_i(1));

		tfc_setting_s curset = tfc.m_setting;
		tfc.m_setting.pwm_max = TFC_PWM_MINMAX;

		tfc.setMotorPWM_i(pa, pb);

		tfc.m_setting = curset;
		break;
	}
	case 4: // Test Camera
	{
		tfc.setServo_i(0, 0);
		tfc.MotorPWMOnOff(0);
		tfc.setLEDs(0);

		uint16_t line[ TFC_CAMERA_LINE_LENGTH];

		tfc.getImage(0, line, TFC_CAMERA_LINE_LENGTH);

		DIAG_PRINTF("Line:" NL);
		for (uint16_t i = 0; i < TFC_CAMERA_LINE_LENGTH; i++) {
			if (i % 16 == 0)
				DIAG_PRINTF( NL);
			DIAG_PRINTF(" %03X", line[i]);
		}
		DIAG_PRINTF( NL);
		break;
	}
	default:
		tfc.setServo_i(0, 0);
		tfc.MotorPWMOnOff(0);
	}
}

/*
 * @brief   Application entry point.
 */
int main(void) {

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
#endif

	tfc.InitAll();
	tfc.setServoCalibration(0,TFC_SERVO_DEFAULT_CENTER,500);
	tfc.setPWMMax(TFC_PWM_MINMAX);

    /*
     * UDP INIT
     * */
#if UDP_SEND_DATA
    tfc.setLED(0, 1);
    tfc.setLED(1, 1);
    tfc.setLED(2, 1);
    tfc.setLED(3, 1);

    PRINTF("Initializing enet...");


     //enet.init(sizeof(nxpbc::SendData), 4444);
   // enet.init(5,4444);
    enet.init(sizeof(WifiFrame), 4444);
    PRINTF("OK\r\n");
#endif

	/* Force the counter to be placed into memory. */
	/* Enter an infinite loop, just incrementing a counter. */
	PRINTF("jedeme...\r\n");

	WifiFrame wifiFrames;

		bool saveWifi = false;

		if (tfc.ImageReady(0))
		{
			saveWifi = true;
		}

	u_int32_t i = 0;
	for (;;) {
        enet.check();
		//TFC_Task must be called in your main loop.  This keeps certain processing happy (I.E. Serial port queue check)
		//   TFC_Task();

		// If DIP switch 1 is lowhen run MCP, else Demo program
		if (tfc.getDIPSwitch() & 0x01)
			// Run Demo Program
			Diagnostic();
		else {
//			enet.send(&car->sendData_, sizeof(nxpbc::SendData));

			/*if(i % 100000 == 0){
				enet.send("Ahoj\n", 5);
			}*/
			i++;
			if (saveWifi)
			{
					wifiFrames.fb[0] = (int16_t)(tfc.ReadFB_f(0)*100);
					wifiFrames.fb[1] = (int16_t)(tfc.ReadFB_f(1)*100);
					wifiFrames.timestamp = 0;
					tfc.getImage(0, wifiFrames.image, TFC_CAMERA_LINE_LENGTH);

					wifiFrames.pwm[0] = 5;//(int16_t)leftEngine;
					wifiFrames.pwm[1] = 10;//(int16_t)rightEngine;
					wifiFrames.servo = 5;//(int16_t)turning;
					wifiFrames.dc[0]= 10;
					wifiFrames.dc[1] = 15;
					wifiFrames.speed = 100;

							//const char *data[] = { "1420", (const char *) wifiFrames[currentWifiFrameIndex] } ; //(const char *) wifiFrame };
							//const char *data[] = {(const char *) wifiFrames};
							//currentWifiFrameIndex = (currentWifiFrameIndex + 1)%3;
							//esp.cmd_start( ecmd_IPSend, data);
					enet.send(&wifiFrames, sizeof(wifiFrames));
			}
			// Run MCP
			//MasterControlProgram();
//					enet.send("Ahoj\n", 5);
		}
	} // end of infinite for loop
	return 0;
}
