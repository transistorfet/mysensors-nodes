
#include <MySensor.h>  
#include <SPI.h>

#include <DallasTemperature.h>
#include <OneWire.h>
#include <IRremote.h>
 
#define PIN_IRSEND		3
#define PIN_IRRECV		4
#define PIN_ONE_WIRE_BUS	5

#define SENSORID_IR		1
#define SENSORID_TEMP		20

#define MAX_ATTACHED_DS18B20	16

MySensor gw;
boolean receivedConfig = false;
boolean metric = true; 

unsigned long SLEEP_TIME = 30000; // Sleep time between reads (in milliseconds)
unsigned long last_time = 0;

OneWire oneWire(PIN_ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors = 0;

// Initialize temperature message
MyMessage msg(0, V_TEMP);

IRsend irsend;
IRrecv irrecv(PIN_IRRECV);

decode_results ir_data;

void setup()
{
	Serial.begin(115200);

	// Startup OneWire 
	sensors.begin();

	// Startup and initialize MySensors library. Set callback for incoming messages. 
	gw.begin(incomingMessage, AUTO, true); 

	// Send the sketch version information to the gateway and Controller
	gw.sendSketchInfo("IR Repeater", "1.0");

	// Fetch the number of attached temperature sensors  
	numSensors = sensors.getDeviceCount();

	// Present all sensors to controller
	for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {   
		gw.present(SENSORID_TEMP + i, S_TEMP);
	}

	irrecv.enableIRIn();
	gw.present(SENSORID_IR, S_IR);
}

void loop()
{
	// Process incoming messages (like config from server)
	gw.process(); 

	if (millis() - last_time > SLEEP_TIME) {
		last_time = millis();

		// Fetch temperatures from Dallas sensors
		sensors.requestTemperatures();

		// Read temperatures and send them to controller 
		for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {
			float temperature = gw.getConfig().isMetric ? sensors.getTempCByIndex(i) : sensors.getTempFByIndex(i);

			// Only send data if temperature has changed and no error
			if (lastTemperature[i] != temperature && temperature != -127.00) {

				// Send in the new temperature
				gw.send(msg.setSensor(SENSORID_TEMP + i).set(temperature, 4));
				lastTemperature[i] = temperature;
			}
		}
	}
	//gw.sleep(SLEEP_TIME);

	if (irrecv.decode(&ir_data)) {
		Serial.print(ir_data.decode_type);
		Serial.write(" ");
		Serial.print(ir_data.bits);
		Serial.write(" ");
		Serial.print(ir_data.value, HEX);
		Serial.write(" ");
		Serial.print(ir_data.panasonicAddress, HEX);
		Serial.write("\n");

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

		if (ir_data.decode_type == NEC) {
			irsend.sendNEC(ir_data.value, ir_data.bits);
		}
		irrecv.enableIRIn();
	}
}

void incomingMessage(const MyMessage &message) {
	if (message.type == V_IR_SEND && message.sensor == SENSORID_IR) {
		unsigned long code;

		code = message.getLong();
		/*
		irsend.sendSony(code, 12);
		if (code != 0x481 && code != 0xC81) {
			delay(50);
			irsend.sendSony(code, 12);
			delay(50);
			irsend.sendSony(code, 12);
		}
		*/

		irsend.sendNEC(code, 32);
		irrecv.enableIRIn();
	} 
}

