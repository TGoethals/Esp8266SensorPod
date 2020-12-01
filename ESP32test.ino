// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       ESP32test.ino
    Created:	23/11/2018 19:03:55
    Author:     MAGLORPC\Tom
*/

// Define User Types below here or use a .h file
//


// Define Function Prototypes that use User Types below here or use a .h file
//

//#include <MHZ19.h>
//#include <BH1750.h>
#include <DHTesp.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <MHZ19.h>

#include <BH1750FVI.h>

// Define Functions below here or use other .ino or cpp files
//

//Device settings
//living .50
//gangbeneden .38
//kelder .35
//gangboven .27
//wc .10
//badkamer .33
//outside .21
//beneden (co2,pm) .13
//living2 .14
//waskot
const char* temptopic = "maglorhome/waskot/temp";
const char* humtopic = "maglorhome/waskot/humidity";
const char* pirtopic = "maglorhome/waskot/pir"; 
const char* luxtopic = "maglorhome/waskot/lux";
const char* hbtopic = "maglorhome/hb";
const char* pm25topic = "maglorhome/waskot/pm25";
const char* pm10topic = "maglorhome/waskot/pm10";
const char* aqi25topic = "maglorhome/waskot/aqi25";
const char* aqi10topic = "maglorhome/waskot/aqi10";
const char* co2topic = "maglorhome/waskot/co2";

const char* hostname = "espwaskot";

const int sleepTimeMsec = 50;

//WIFI + MQTT

char* ssid = "WiFi-2.4-09cc";
char* password = "netwerkske";
const char* mqttServer = "192.168.1.110";
const int mqttPort = 1883;
const char* mqttUser = "eat";
const char* mqttPassword = "shit";
const char* mqttClientName = "waskotClient";

WiFiClient espClient;
PubSubClient client(espClient);

const int mqttKeepAlive = 5;
int mqttSkipWindows;
int mqttCounter;

//DHT22 temp+humidity

#define DHTPIN 5     // what pin we're connected to
#define DHTTYPE DHTesp::DHT22   // DHT22

const int dhtReportingTimeSec = 60;
int dhtSkipWindows;
int dhtCounter;

DHTesp dht; // Initialize DHT sensor for normal 16mhz Arduino

float hum;  //Stores humidity value
float temp; //Stores temperature value

//MHZ19 CO2

const int coReportingTimeSec = 60;
int coSkipWindows;
int coCounter;

//fix this via serial.swap()
//crap thing *NEEDS* hardware serial (UART0), so no debug output nor other sensors can work with it
//except that PM25/10 crap
//const int mhz_rx_pin = 13; //Serial rx pin no
//const int mhz_tx_pin = 15; //Serial tx pin no

MHZ19 *mhz19_uart = new MHZ19();

//Light sensor

BH1750FVI LightSensor(BH1750FVI::k_DevModeContHighRes);

const int luxReportingTimeSec = 60;
int luxSkipWindows;
int luxCounter;

//PM2.5

#define PIN_PM10  12
#define PIN_PM25  14

unsigned long sampletime_ms = 20000;
long concentrationPM25 = 0;
long concentrationPM10 = 0;

//PIR

bool pirTriggered;
#define PIRPIN 4


// The setup() function runs once each time the micro-controller starts
void setup()
{
	Serial.begin(9600);

	initDHT();
	initPIR();
	//initLux();
	//initPM();
	//initCO2();

	initWifi();
	initMQTT();
	initOTA();
}

void initCO2() {
	Serial.swap();
	//mhz19_uart->begin(mhz_rx_pin, mhz_tx_pin);
	mhz19_uart->setAutoCalibration(false);
	
	//delay(20 * 60 * 1000);
	//mhz19_uart->calibrateZero();
	
	/*while (mhz19_uart->isWarming())
	{
		Serial.print("MH-Z19 now warming up...  status:");
		Serial.println(mhz19_uart->getStatus());
		delay(1000);
	}*/

	coSkipWindows = (coReportingTimeSec * 1000.0f) / sleepTimeMsec;
	coCounter = 0;
}

void initPM() {
	pinMode(PIN_PM10, INPUT);
	pinMode(PIN_PM25, INPUT);
}

void initOTA() {
	ArduinoOTA.setHostname(hostname);
	ArduinoOTA.onStart([]() {
		String type;
		if (ArduinoOTA.getCommand() == U_FLASH)
			type = "sketch";
		else // U_SPIFFS
			type = "filesystem";

		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		//Serial.println("Start updating " + type);
	});
	ArduinoOTA.onEnd([]() {
		//Serial.println("\nEnd");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		//Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
	});
	ArduinoOTA.onError([](ota_error_t error) {
		//Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});
	ArduinoOTA.begin();
	Serial.println("Ready");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
}

void initLux() {
	LightSensor.begin();
	luxSkipWindows = (luxReportingTimeSec * 1000.0f) / sleepTimeMsec;
	luxCounter = 0;
}

void initDHT() {
	dht.setup(DHTPIN, DHTesp::DHT22);
	dhtCounter = 0;
	dhtSkipWindows = (dhtReportingTimeSec * 1000.0f) / sleepTimeMsec;
}

void initPIR() {
	pinMode(PIRPIN, INPUT);
	pirTriggered = false;
}

void initWifi() {
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	while (WiFi.waitForConnectResult() != WL_CONNECTED) {
		Serial.println("Connection Failed! Rebooting...");
		delay(5000);
		ESP.restart();
	}

	Serial.println("Connected to the WiFi network");
}

void initMQTT() {
	if (WiFi.status() != WL_CONNECTED) {
		ESP.restart();
	}

	client.setServer(mqttServer, mqttPort);

	//while (!client.connected()) {
		//Serial.println("Connecting to MQTT...");

		if (client.connect(mqttClientName, mqttUser, mqttPassword)) {
			Serial.println("connected");
		}
		else {
			Serial.print("failed with state ");
			Serial.print(client.state());
			delay(2000);
			ESP.restart();
		}
	//}

	mqttCounter = 0;
	mqttSkipWindows = ((1000 * mqttKeepAlive) / sleepTimeMsec);
}

// Add the main program code into the continuous loop() function
void loop()
{
	delay(sleepTimeMsec);

	ArduinoOTA.handle();
	readDht();
	readPir();
	//readLux();
	//readPM();
	//readCO2();
	keepAlive();
}

void keepAlive() {
	mqttCounter++;
	client.loop();
	if (mqttCounter == mqttSkipWindows) {
		publishMqtt(hbtopic, "1");

		mqttCounter = 0;
	}
}

void readCO2() {

	//coCounter++;
	int co2 = -1;
	int tries = 0;
	while (co2 < 0 && tries < 10) {
		//if (coCounter >= coSkipWindows) {
		measurement_t m = mhz19_uart->getMeasurement();
		co2 = m.co2_ppm;
		//Serial.print("co2: ");
		//Serial.println(co2);
		//Serial.print("temp: ");
		//Serial.println(m.temperature);
		if (m.co2_ppm > 0) {
			char* message = new char[6];
			sprintf(message, "%d", m.co2_ppm);
			publishMqtt(co2topic, message);
			delete message;
		}
		delay(200);
		tries++;
		//coCounter = 0;
	//}
	}
}

void readLux() 
{
	luxCounter++;

	if (luxCounter >= luxSkipWindows) {
		uint16_t lux = LightSensor.GetLightIntensity();

		char* message = new char[10];
		sprintf(message, "%d", lux);
		publishMqtt(luxtopic, message);
		delete message;

		luxCounter = 0;
	}
}

void readDht() {
	//Read data and store it to variables hum and temp
	dhtCounter++;

	if (dhtCounter >= dhtSkipWindows) {
		hum = dht.getHumidity();
		temp = dht.getTemperature();

		dhtCounter = 0;

		if (!isnan(hum)) {
			//we're probably good
			char* message = new char[75];
			sprintf(message, "%.2f", temp);
			publishMqtt(temptopic, message);

			sprintf(message, "%.2f", hum);
			publishMqtt(humtopic, message);
			delete message;
		}
	}
}

//https://andypi.co.uk/2016/08/19/weather-monitoring-part-2-air-quality-sensing-with-shinyei-ppd42ns/
//https://en.wikipedia.org/wiki/Air_quality_index
void readPM() {
	//get PM 2.5 density of particles under 2.5 μm.
	concentrationPM25 = getPM(PIN_PM25);
	float ugm3 = calcPPMV(concentrationPM25, false);
	int aqi = calcAQI25(ugm3);
	//Serial.print("PM25: ");
	//Serial.println(ugm3);
	//Serial.println(aqi);

	char* message = new char[75];
	sprintf(message, "%.2f", ugm3);
	publishMqtt(pm25topic, message);

	sprintf(message, "%d", aqi);
	publishMqtt(aqi25topic, message);

	//get PM 10 - density of particles under 10 μm.
	concentrationPM10 = getPM(PIN_PM10);
	ugm3 = calcPPMV(concentrationPM10, true);
	aqi = calcAQI10(ugm3);
	//Serial.print("PM10: ");
	//Serial.println(ugm3);
	//Serial.println(aqi);

	sprintf(message, "%.2f", ugm3);
	publishMqtt(pm10topic, message);

	sprintf(message, "%d", aqi);
	publishMqtt(aqi10topic, message);
	delete message;
}

void readPir() {
	int val = digitalRead(PIRPIN);
	//Serial.println(val);
	if (val == HIGH && !pirTriggered) {
		//Serial.println("interrupt up");
		pirTriggered = true;
		publishMqtt(pirtopic, "1");
	}
	else if (val == LOW && pirTriggered) {
		//Serial.println("interrupt down");
		pirTriggered = false;
		publishMqtt(pirtopic, "0");
	}
}

void publishMqtt(const char *topic, char *message) {
	Serial.println(topic);
	Serial.println(message);
	if (!client.publish(topic, message)) {
		//initMQTT();
		//client.publish(topic, message);
		ESP.restart();
	}

}

long getPM(int pin)
{
	unsigned long lowpulseoccupancy = 0;
	unsigned long starttime = millis();
	unsigned long endtime;
	unsigned long lpomsec;
	long concentration = 0;
	float ratio = 0;

	while (true) {
		//lowpulseoccupancy represents the Lo Pulse Occupancy Time(LPO Time)
		lowpulseoccupancy += pulseIn(pin, LOW);
		endtime = millis();

		if ((endtime - starttime) > sampletime_ms)
		{
			//Serial.print("lowpulseoccupancy: ");
			//Serial.println(lowpulseoccupancy);

			ratio = lowpulseoccupancy / (sampletime_ms*10.0);  // Integer percentage 0=>100
			//concentration is a figure that has physical meaning. It's calculated from the characteristic graph below by using the LPO time.
			concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve
			return concentration;
		}
	}
}

float calcPPMV(long concentration, bool pm10)
{
	float densitypm = 1.65 * pow(10, 12);
	// Assume the radius of a particle in the PM2.5 channel is .44 µm
	float rpm = 0.44 * pow(10, -6);
	if (pm10) {
		rpm = 0.88 * pow(10, -6);
	}
	// Volume of a sphere = 4 / 3 * pi * radius ^ 3
	float volpm = (4 / 3) * 3.14 * pow(rpm, 3);
	// mass = density * volume
	float masspm = densitypm * volpm;
	// parts / m3 = parts / foot3 * 3531.5
	// µg / m3 = parts / m3 * mass in µg
	float concentration_ugm3 = concentration * 3531.5 * masspm;
	return concentration_ugm3;
}

int calcAQI25(float ugm3) {
	float cbreakpointspm25[7][4] = { {0.0, 12, 0, 50},
	{12.1, 35.4, 51, 100 },
	{35.5, 55.4, 101, 150},
	{55.5, 150.4, 151, 200},
	{150.5, 250.4, 201, 300},
	{250.5, 350.4, 301, 400},
	{350.5, 500.4, 401, 500} };

	int aqi = 0;
	if (ugm3 > 500.4) {
		aqi = 500;
	}
	else {
		for (int i = 0; i < 7; i++) {
			if (cbreakpointspm25[i][0] <= ugm3 && ugm3 <= cbreakpointspm25[i][1]) {
				float Clow = cbreakpointspm25[i][0];
				float	Chigh = cbreakpointspm25[i][1];
				float	Ilow = cbreakpointspm25[i][2];
				float	Ihigh = cbreakpointspm25[i][3];
				aqi = (((Ihigh - Ilow) / (Chigh - Clow))*(ugm3 - Clow)) + Ilow;
			}
		}
	}
	return aqi;
}

int calcAQI10(float ugm3) {
	float cbreakpointspm10[7][4] = { {0.0, 54, 0, 50},
	{54.1, 154, 51, 100 },
	{154.1, 254, 101, 150},
	{254.1, 354, 151, 200},
	{354.1, 424, 201, 300},
	{424.1, 504, 301, 400},
	{504.1, 604, 401, 500} };

	int aqi = 0;
	if (ugm3 > 604) {
		aqi = 604;
	}
	else {
		for (int i = 0; i < 7; i++) {
			if (cbreakpointspm10[i][0] <= ugm3 && ugm3 <= cbreakpointspm10[i][1]) {
				float Clow = cbreakpointspm10[i][0];
				float	Chigh = cbreakpointspm10[i][1];
				float	Ilow = cbreakpointspm10[i][2];
				float	Ihigh = cbreakpointspm10[i][3];
				aqi = (((Ihigh - Ilow) / (Chigh - Clow))*(ugm3 - Clow)) + Ilow;
			}
		}
	}
	return aqi;
}
