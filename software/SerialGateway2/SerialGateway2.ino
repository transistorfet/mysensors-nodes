/**
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
* Copyright (C) 2013-2019 Sensnology AB
* Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
*
* Documentation: http://www.mysensors.org
* Support Forum: http://forum.mysensors.org
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
*******************************
*
* DESCRIPTION
* The ArduinoGateway prints data received from sensors on the serial link.
* The gateway accepts input on serial which will be sent out on radio network.
*
* The GW code is designed for Arduino Nano 328p / 16MHz
*
* Wire connections (OPTIONAL):
* - Inclusion button should be connected between digital pin 3 and GND
* - RX/TX/ERR leds need to be connected between +5V (anode) and digital pin 6/5/4 with resistor 270-330R in a series
*
* LEDs (OPTIONAL):
* - To use the feature, uncomment any of the MY_DEFAULT_xx_LED_PINs
* - RX (green) - blink fast on radio message received. In inclusion mode will blink fast only on presentation received
* - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
* - ERR (red) - fast blink on error during transmission error or receive crc error
*
*/

// Enable debug prints to serial monitor
#define MY_DEBUG


// Enable and select radio type attached
#define MY_RADIO_RF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

// Set LOW transmit power level as default, if you have an amplified NRF-module and
// power your radio separately with a good regulator you can turn up PA level.
#define MY_RF24_PA_LEVEL RF24_PA_LOW

// Enable serial gateway
#define MY_GATEWAY_SERIAL

// Define a lower baud rate for Arduinos running on 8 MHz (Arduino Pro Mini 3.3V & SenseBender)
#if F_CPU == 8000000L
#define MY_BAUD_RATE 38400
#endif

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
//#define MY_INCLUSION_BUTTON_FEATURE

// Inverses behavior of inclusion button (if using external pullup)
//#define MY_INCLUSION_BUTTON_EXTERNAL_PULLUP

// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
//#define MY_INCLUSION_MODE_BUTTON_PIN  3

// Set blinking period
#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Inverses the behavior of leds
//#define MY_WITH_LEDS_BLINKING_INVERSE

// Flash leds on rx/tx/err
// Uncomment to override default HW configurations
//#define MY_DEFAULT_ERR_LED_PIN 4  // Error led pin
//#define MY_DEFAULT_RX_LED_PIN  6  // Receive led pin
//#define MY_DEFAULT_TX_LED_PIN  5  // the PCB, on board LED

#include <MySensors.h>
#include <IRremote.h> 

#define IR_SEND_PIN		3
#define IR_RECEIVE_PIN		4

#define SENSORID_IR		1

void setup()
{
	IrReceiver.begin(IR_RECEIVE_PIN);
	IrSender.begin(IR_SEND_PIN); 
}

void presentation()
{
	sendSketchInfo("Serial Gateway + IR", "1.0");
	present(SENSORID_IR, S_IR);
}

void loop()
{
	if (IrReceiver.decode()) {
		//Serial.print(ir_data.decode_type);
		//Serial.write(" ");
		//Serial.print(ir_data.bits);
		//Serial.write(" ");
		//Serial.print(ir_data.value, HEX);
		//Serial.write(" ");
		//Serial.print(ir_data.panasonicAddress, HEX);
		//Serial.write("\n");

		//if (ir_data.decode_type == SONY) {
		//	// Sony Power On/Off
		//	irsend.sendSony(ir_data.value, ir_data.bits);
		//	if (ir_data.value != 0x481 && ir_data.value != 0xC81) {
		//		delay(50);
		//		irsend.sendSony(ir_data.value, ir_data.bits);
		//		delay(50);
		//		irsend.sendSony(ir_data.value, ir_data.bits);
		//	}
		//	Serial.print("Sending ir: ");
		//	Serial.print(ir_data.value);
		//	Serial.print("\n");
		//}

		if (IrReceiver.decodedIRData.protocol == NEC) {
			IrSender.sendNEC(IrReceiver.decodedIRData.address, IrReceiver.decodedIRData.command, 1);
		}
		IrReceiver.enableIRIn();
	}
}

void receive(const MyMessage &message) {
	if (message.type == V_IR_SEND && message.destination == 0 && message.sensor == SENSORID_IR) {
		unsigned long code;
		code = message.getLong();

		//irsend.sendNEC(code, 32);

		// TV Power Off Example: 0;1;1;0;32;2704
		IrSender.sendSony(code, 12);
		delay(50);
		IrSender.sendSony(code, 12);
		delay(50);
		IrSender.sendSony(code, 12);
		delay(50);

		IrReceiver.enableIRIn();
	} 
}

