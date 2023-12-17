
#include <MySensor.h>  
#include <SPI.h>

#include <DallasTemperature.h>
#include <OneWire.h>
#include <RCSwitch.h> 

#include <DHT.h>

#define PIN_ONE_WIRE_BUS	3
#define PIN_RCSWITCH315		4
#define PIN_HUMIDITY		5
#define PIN_RCSWITCH433		6

#define SENSORID_RCSWITCH_W1	1
#define SENSORID_RCSWITCH_E1	2
#define SENSORID_RCSWITCH_E2	3
#define SENSORID_RCSWITCH_E3	4
#define SENSORID_RCSWITCH_E4	5
#define SENSORID_RCSWITCH_E5	6
#define SENSORID_TEMP		20
#define SENSORID_HUMIDITY	40

#define MAX_ATTACHED_DS18B20	16

MySensor gw;
boolean receivedConfig = false;
boolean metric = true; 

unsigned long SLEEP_TIME = 30000; // Sleep time between reads (in milliseconds)
unsigned long last_time = 0;

RCSwitch mySwitch315 = RCSwitch();
RCSwitch mySwitch433 = RCSwitch();

OneWire oneWire(PIN_ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors = 0;

DHT dht;
float lastHumidity;

MyMessage msg_temp(0, V_TEMP);
MyMessage msg_hum(0, V_HUM);

void setup()
{
	Serial.begin(115200);

	// Startup and initialize MySensors library. Set callback for incoming messages. 
	gw.begin(incomingMessage, AUTO, true); 

	// Send the sketch version information to the gateway and Controller
	gw.sendSketchInfo("Switches+Sensor Node", "1.0");

	// Setup Woodward Switch
	mySwitch315.enableTransmit(PIN_RCSWITCH315);
	woodwardSendSwitch(gw.loadState(SENSORID_RCSWITCH_W1));
	gw.present(SENSORID_RCSWITCH_W1, S_LIGHT);

	// Setup ElekCity Switches
	mySwitch433.enableTransmit(PIN_RCSWITCH433);
	mySwitch433.setProtocol(1, 189);

	for (char sid = SENSORID_RCSWITCH_E1; sid <= SENSORID_RCSWITCH_E5; sid++) {
		gw.present(sid, S_LIGHT);
		elekcitySendSwitch(sid, gw.loadState(sid));
	}

	// Startup OneWire 
	sensors.begin();

	// Fetch the number of attached temperature sensors  
	numSensors = sensors.getDeviceCount();

	// Present all sensors to controller
	for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {   
		gw.present(SENSORID_TEMP + i, S_TEMP);
	}

	dht.setup(PIN_HUMIDITY);
	gw.present(SENSORID_HUMIDITY, S_HUM);
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
				gw.send(msg_temp.setSensor(SENSORID_TEMP + i).set(temperature, 4));
				lastTemperature[i] = temperature;
			}
		}

		float humidity = dht.getHumidity();
		if (lastHumidity != humidity) {
			gw.send(msg_hum.setSensor(SENSORID_HUMIDITY).set(humidity, 1));
			lastHumidity = humidity;
		}
	}
	//gw.sleep(SLEEP_TIME);
}

void woodwardSendSwitch(bool state)
{
	if (state)
		mySwitch315.sendTriState("0FFFFFF01111");
		//"0001 0101 0101 0100 1111 1111"
		//mySwitch315.send(0x1554FF, 24);
	else
		mySwitch315.sendTriState("0FFFFFF00000");
		//"0001 0101 0101 0100 0000 0000"
		//mySwitch315.send(0x155400, 24);
}

long int elekcityCodes[][2] = {
	// { On-Code, Off-Code }
	{ 0x115533, 0x11553C },
	{ 0x1155C3, 0x1155CC },
	{ 0x115703, 0x11570C },
	{ 0x115D03, 0x115D0C },
	{ 0x117503, 0x11750C }
};

void elekcitySendSwitch(char sensor, bool state)
{
	mySwitch433.send(elekcityCodes[sensor - SENSORID_RCSWITCH_E1][state ? 0 : 1], 24);
}

void incomingMessage(const MyMessage &message)
{
	if (message.type == V_LIGHT) {
		if (message.sensor == SENSORID_RCSWITCH_W1) {
			woodwardSendSwitch(message.getBool());

			// Store state in eeprom
			gw.saveState(message.sensor, message.getBool());
		}
		else if (message.sensor >= SENSORID_RCSWITCH_E1 && message.sensor <= SENSORID_RCSWITCH_E5) {
			bool val = message.getBool();

			// send to switch
			elekcitySendSwitch(message.sensor, val);

			// Store state in eeprom
			gw.saveState(message.sensor, val);
		}
		else
			return;

		// Write some debug info
		Serial.print("Incoming change for sensor:");
		Serial.print(message.sensor);
		Serial.print(", New status: ");
		Serial.println(message.getBool());
	} 
}


