
#include <MySensor.h>  
#include <SPI.h>

#include <SFE_BMP180.h>
#include <TSL2561.h>
#include <Wire.h>

#include <DHT.h>

#define PIN_HUMIDITY		5

#define SENSORID_TEMP		20
#define SENSORID_HUMIDITY	40
#define SENSORID_PRESSURE	60
#define SENSORID_LIGHT		80

MySensor gw;
boolean receivedConfig = false;
boolean metric = true; 

unsigned long SLEEP_TIME = 30000; // Sleep time between reads (in milliseconds)
unsigned long last_time = 0;

SFE_BMP180 pressure;
float lastTemperature;
float lastPressure;

TSL2561 tsl(TSL2561_ADDR_FLOAT);
float lastLight;

DHT dht;
float lastHumidity;

MyMessage msg_temp(0, V_TEMP);
MyMessage msg_hum(0, V_HUM);
MyMessage msg_pres(0, V_PRESSURE);
MyMessage msg_light(0, V_LIGHT_LEVEL);

void setup()
{
	// Turn ADC and Comparator off
	ADCSRA = 0;
	ADMUX = 0;
	ACSR = ACD;
	// Turn off digital input on analog pins
	DIDR0 = 0x7f;
	DIDR1 = 0x7f;
	// Turn off Timer 2
	TCCR2B = 0;

	Serial.begin(115200);

	// Startup and initialize MySensors library. Set callback for incoming messages. 
	gw.begin(incomingMessage, AUTO, false); 

	// Send the sketch version information to the gateway and Controller
	gw.sendSketchInfo("Outdoor Weather Node", "1.0");

	if (pressure.begin()) {
		gw.present(SENSORID_TEMP, S_TEMP);
		gw.present(SENSORID_PRESSURE, S_BARO);
	}


	if (tsl.begin()) {
		// You can change the gain on the fly, to adapt to brighter/dimmer light situations
		//tsl.setGain(TSL2561_GAIN_0X);         // set no gain (for bright situtations)
		tsl.setGain(TSL2561_GAIN_16X);      // set 16x gain (for dim situations)

		// Changing the integration time gives you a longer time over which to sense light
		// longer timelines are slower, but are good in very low light situtations!
		tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
		//tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  // medium integration time (medium light)
		//tsl.setTiming(TSL2561_INTEGRATIONTIME_402MS);  // longest integration time (dim light)
		gw.present(SENSORID_LIGHT, S_LIGHT_LEVEL);
	}


	dht.setup(PIN_HUMIDITY);
	gw.present(SENSORID_HUMIDITY, S_HUM);
}
  
void loop()
{
	// Process incoming messages (like config from server)
	gw.process(); 

	sendPressure();

	//float light = tsl.getLuminosity(TSL2561_FULLSPECTRUM);
	uint32_t lum = tsl.getFullLuminosity();
	uint16_t ir, full;
	ir = lum >> 16;
	full = lum & 0xFFFF;
	float light = tsl.calculateLux(full, ir);
	if (light != lastLight) {
		gw.send(msg_light.setSensor(SENSORID_LIGHT).set(light, 0));
		lastLight = light;
	}

	float humidity = dht.getHumidity();
	if (lastHumidity != humidity) {
		gw.send(msg_hum.setSensor(SENSORID_HUMIDITY).set(humidity, 1));
		lastHumidity = humidity;
	}

	gw.sleep(SLEEP_TIME);
}


void sendPressure()
{
	char status;
	double T,P,p0,a;

	// You must first get a temperature measurement to perform a pressure reading.

	// Start a temperature measurement:
	// If request is successful, the number of ms to wait is returned.
	// If request is unsuccessful, 0 is returned.

	status = pressure.startTemperature();
	if (status != 0)
	{
		// Wait for the measurement to complete:

		delay(status);

		// Retrieve the completed temperature measurement:
		// Note that the measurement is stored in the variable T.
		// Use '&T' to provide the address of T to the function.
		// Function returns 1 if successful, 0 if failure.

		status = pressure.getTemperature(T);
		if (status != 0) {
			// Start a pressure measurement:
			// The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
			// If request is successful, the number of ms to wait is returned.
			// If request is unsuccessful, 0 is returned.

			//Serial.print("temperature: ");
			//Serial.print(T);
			//Serial.print("\n");

			if (T != lastTemperature) {
				gw.send(msg_temp.setSensor(SENSORID_TEMP).set(T, 4));
				lastTemperature = T;
			}

			status = pressure.startPressure(3);
			if (status != 0) {
				// Wait for the measurement to complete:
				delay(status);

				// Retrieve the completed pressure measurement:
				// Note that the measurement is stored in the variable P.
				// Use '&P' to provide the address of P.
				// Note also that the function requires the previous temperature measurement (T).
				// (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
				// Function returns 1 if successful, 0 if failure.

				status = pressure.getPressure(P,T);
				if (status != 0) {
					if (P != lastPressure) {
						gw.send(msg_pres.setSensor(SENSORID_PRESSURE).set(P, 4));
						lastPressure = P;
					}
				}
				else Serial.println("error retrieving pressure measurement\n");
			}
			else Serial.println("error starting pressure measurement\n");
		}
		else Serial.println("error retrieving temperature measurement\n");
	}
	else Serial.println("error starting temperature measurement\n");
}

void incomingMessage(const MyMessage &message)
{

}

