
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <Adafruit_BME280.h>

#define DEBUG_LEVEL 3
#define DEBUG_ERROR 1
#define DEBUG_MIN_INFO 2
#define DEBUG_MAX_INFO 3
#define DEBUG_OUT(level, fmt, ...) if(DEBUG_LEVEL>=level) Serial.printf_P( (PGM_P)PSTR(fmt), ## __VA_ARGS__ )

// Set Wifi settings here
#define WLANSSID "..."
#define WLANPWD "..."

// Fill in static IP details...
IPAddress staticIP(192, 168, 0, xx);
IPAddress gateway(192, 168, 0, 1);
IPAddress dns(xx, xx, xx, xx);
IPAddress subnet(255, 255, 255, 0);

// Set Thingspeak channel settings here
// Replace xxxxxxx by the Thingspeak channel number below
#define URL_THINGSPEAK "/channels/xxxxxxx/bulk_update.json"
// Fill in the Write API key, see https://thingspeak.com/channels/xxxxxxx/api_keys
#define APIKEY_THINGSPEAK "..."

#define HOST_THINGSPEAK "api.thingspeak.com"
�#define PORT_THINGSPEAK 80

IPAddress thingspeakIP(34,226,171,107);

const char TXT_CONTENT_TYPE_JSON[] PROGMEM = "application/json";


Adafruit_BME280 bme280;

const int trigPin = 12;    // Trig D6, GPIO12
const int echoPin = 13;    // Echo D7, GPIO13
const int switchPin = 15;  // D8, GPIO15, on an esp8266 NodeMcu can only be D1, D2 or D8 because other pins are high in deep sleep, but D1, D2 are the default pins for BME280
const int modePin = 14;    // D5, GPIO14, allows measurement mode: 1, or OTA mode: 0

const double minVoltage=2.7;
const int RA1=100;
const int RA2=470;


// Number of distance measurements, take the average
const byte numMeasuresDistance=3;
const byte numMeasuresBattery=10;

const double capacity=0.0; // l
const double fullDistance=0.0; //cm
const double area=-0.10; //m2
const double deepsleepTime=3600.0; // seconds
//const double deepsleepTime=50.0; // seconds

// modeResult: 0: OTA, modeResult: 1: measurement mode
int modeResult;

 
// calculates speed of sound in m/s
// t: temperature in °C
// rh: relative humidity in percentage
// p: pressure in Pa
// Xc: mole fraction of carbon dioxide
double speedOfSound(double t, double rh, double p, double Xc=400.0e-6) {
  double t_kel=t+273.15;

  // Davis R S 1992 Equation for the determination of the density
  // of moist air (1981/91) Metrologia 29 67–70 
  // enhancement factor
  double enh = 3.14e-8*p+1.00062+t*t*5.6e-7;
  // vapour pressure at saturation
  double psv = exp(t_kel*t_kel*1.2378847e-5-1.9121316e-2*t_kel+33.93711047-6.3431645e3/t_kel);
  // absolute humidity, mole fraction of water vapour
  double Xw = rh/100.0*enh*psv/p;

  //Speed calculated using the method of Cramer from
  //JASA vol 93 pg 2510
  double C1 = 0.603055*t + 331.5024 - t*t*5.28e-4 + (0.1495874*t + 51.471935 -t*t*7.82e-4)*Xw;
  double C2 = (-1.82e-7+3.73e-8*t-t*t*2.93e-10)*p+(-85.20931-0.228525*t+t*t*5.91e-5)*Xc;
  double C3 = Xw*Xw*2.835149 + p*p*2.15e-13 -Xc*Xc*29.179762 - 4.86e-4*Xw*p*Xc;
  return C1 + C2 - C3;
}

double speedOfSoundSimple(double t) {
  return 20.05*sqrt(273.15+t);
}

void setup() {
  // Start timing
  unsigned long starttime=micros();
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(switchPin, OUTPUT);
  pinMode(modePin, INPUT_PULLUP);

  //digitalWrite(trigPin, LOW);
  digitalWrite(switchPin, HIGH);

  connectWifi();


  // 0x76 is the default, but if it does not work, it might be 0x77
  bool bme280Started=bme280.begin(0x76);
  if(bme280Started) DEBUG_OUT(DEBUG_MIN_INFO, "BME280 found\n");
  else DEBUG_OUT(DEBUG_ERROR, "BME280 not found\n");

  bme280.setSampling(Adafruit_BME280::MODE_FORCED,
                     Adafruit_BME280::SAMPLING_X1, // temperature
                     Adafruit_BME280::SAMPLING_X1, // pressure
                     Adafruit_BME280::SAMPLING_X1, // humidity
                     Adafruit_BME280::FILTER_OFF);

  

  modeResult=digitalRead(modePin);

  
  DEBUG_OUT(DEBUG_MIN_INFO, "OTA pin: %s\n", modeResult==0?"yes":"no");

  if(!modeResult) {
    ArduinoOTA.onStart([]() {
      const char* type=ArduinoOTA.getCommand() == U_FLASH?"sketch":"filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      DEBUG_OUT(DEBUG_MIN_INFO, "Start updating: %s\n", type); 
    });
    ArduinoOTA.onEnd([]() {
      DEBUG_OUT(DEBUG_MIN_INFO, "End\n"); 
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      DEBUG_OUT(DEBUG_MIN_INFO, "Progress: %d\n", progress/(total/100));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      DEBUG_OUT(DEBUG_ERROR, "Error[%u]: ", error);
      //Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        DEBUG_OUT(DEBUG_ERROR, "Auth Failed\n", error);
      } else if (error == OTA_BEGIN_ERROR) {
        DEBUG_OUT(DEBUG_ERROR, "Begin Failed\n", error);
      } else if (error == OTA_CONNECT_ERROR) {
        DEBUG_OUT(DEBUG_ERROR, "Connect Failed\n", error);
      } else if (error == OTA_RECEIVE_ERROR) {
        DEBUG_OUT(DEBUG_ERROR, "Receive Failed\n", error);
      } else if (error == OTA_END_ERROR) {
        DEBUG_OUT(DEBUG_ERROR, "End Failed\n", error);
      }
    });
    ArduinoOTA.begin();
  } else {

    double v=340.0;
    double t=NAN;
    double h=NAN;
    double p=NAN;
    if(bme280Started) {
      bme280.takeForcedMeasurement();
      t = static_cast<double>(bme280.readTemperature());
      h = static_cast<double>(bme280.readHumidity());
      p = static_cast<double>(bme280.readPressure());
      DEBUG_OUT(DEBUG_MIN_INFO, "Temperature: %.2f°C\n", t);
      DEBUG_OUT(DEBUG_MIN_INFO, "Humidity: %.2f%%\n", h);
      DEBUG_OUT(DEBUG_MIN_INFO, "Pressure: %.2f\n", p);
    
      v=speedOfSound(isnan(t)?20.0:t, isnan(h)?0.0:h, isnan(p)?101325.0:p);
      DEBUG_OUT(DEBUG_MIN_INFO, "Speed of sound: %.2f m/s\n", v);
    }
    
    double distance=0.0;
    for(byte i=0; i<numMeasuresDistance;i++) {
      double distance0=distanceMeasurement(v);
      DEBUG_OUT(DEBUG_MAX_INFO, "Measurement: %d, distance: %.2f\n", i, distance0);
      distance+= distance0/numMeasuresDistance;
    }
    
    double remainingVolume=capacity+10.0*(fullDistance-distance)*area;

    DEBUG_OUT(DEBUG_MIN_INFO, "Distance: %.2f\n", distance); 

    DEBUG_OUT(DEBUG_MIN_INFO, "Remaining volume: %.2f\n", remainingVolume); 

    double battlevel=0;
    for(byte i=0;i<numMeasuresBattery;i++) {
      double battlevel0=analogRead(A0)/1023.0*(RA2+RA1)/RA1;
      DEBUG_OUT(DEBUG_MAX_INFO, "Measurement: %d, battery level: %.2f\n", i, battlevel0);
      battlevel+=battlevel0/numMeasuresBattery;
    }
    
    DEBUG_OUT(DEBUG_MIN_INFO, "Battery level: %.2f\n", battlevel); 

    if(WiFi.status()==WL_CONNECTED) {
      if(WiFi.hostByName(HOST_THINGSPEAK, thingspeakIP)) {
        DEBUG_OUT(DEBUG_MIN_INFO, "Thingspeak IP: %s\n", thingspeakIP.toString().c_str()); 
      }
      send2thingspeak(remainingVolume, battlevel, t, h, p);
    }

    digitalWrite(switchPin, LOW);

    // Disconnect WiFi, allows proper deep sleep, takes longer to connect after deep sleep
    WiFi.disconnect(true);
    delay(1);

    DEBUG_OUT(DEBUG_MAX_INFO, "Total time active (microseconds): %lu\n", micros()-starttime);

    if(battlevel>=minVoltage) ESP.deepSleep(deepsleepTime*1e6);
    else ESP.deepSleep(5*3600e6);   // Battery voltage too low, protect from over-discharge and put in infinite deep sleep (aka everything too long)  
    
  }
  
}

double distanceMeasurement(double speedOfSound) {
    // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  unsigned long startmeasurement=micros();
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //DEBUG_OUT(DEBUG_MAX_INFO, "Echo pin state: %s\n", digitalRead(echoPin)==0?"low":"high");
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  //pinMode(echoPin, INPUT);
  noInterrupts();
  unsigned long duration = pulseIn(echoPin, HIGH);
  interrupts();
 
  // convert the time into a distance
  double cm = (duration/2.0)*speedOfSound/10000.0;

  DEBUG_OUT(DEBUG_MIN_INFO, "Duration (microseconds): %lu\n", duration);
  DEBUG_OUT(DEBUG_MIN_INFO, "Distance (cm): %.2f\n", cm);
  DEBUG_OUT(DEBUG_MAX_INFO, "Measurement time (microseconds): %lu\n", micros()-startmeasurement);
  

  return cm;
  
}




int connectWifi() {
  unsigned long startwifi=micros();
  unsigned int retry_count = 0;
  WiFi.forceSleepWake();
  delay(1);

  DEBUG_OUT(DEBUG_MIN_INFO, "WiFi status: %d\n", WiFi.status());
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);

  DEBUG_OUT(DEBUG_MIN_INFO, "Static IP: %s\n", staticIP.toString().c_str()) ; 
  WiFi.config(staticIP, dns, gateway, subnet);
  WiFi.begin(WLANSSID, WLANPWD); // Start WiFI

  DEBUG_OUT(DEBUG_MIN_INFO, "Connecting to %s\n", WLANSSID);

  while ((WiFi.status() != WL_CONNECTED) && (retry_count < 40)) {
    delay(500);
    DEBUG_OUT(DEBUG_MIN_INFO, ".");
    retry_count++;
  }
  DEBUG_OUT(DEBUG_MIN_INFO, "\n");
 
  int success=WiFi.status();
  if(success==WL_CONNECTED) {
    DEBUG_OUT(DEBUG_MIN_INFO, "WiFi connected\nIP address: %s\n", WiFi.localIP().toString().c_str());
  } else {
    DEBUG_OUT(DEBUG_ERROR, "Failed to connect\n");
  }
  DEBUG_OUT(DEBUG_MAX_INFO, "Number of tries: %d\n", retry_count);
  DEBUG_OUT(DEBUG_MAX_INFO, "Connecting time (microseconds): %lu\n", micros()-startwifi);

  return success;
}



void send2thingspeak(double distance, double battlevel, double t, double h, double p) {

  char tempText[20];
  if(isnan(t)) sprintf(tempText, "");
  else sprintf(tempText, ",\"field3\":\"%.1f\"", t);
  char humText[20];
  if(isnan(h)) sprintf(humText, "");
  else sprintf(humText, ",\"field4\":\"%.1f\"", h);
  char pressureText[25];
  if(isnan(p)) sprintf(pressureText, "");
  else sprintf(pressureText, ",\"field5\":\"%.2f\"", p);

  

  char dataThingspeak[125];
  sprintf(dataThingspeak,
          "[{\"delta_t\":\"0\","
          "\"field1\":\"%.2f\","
          "\"field2\":\"%.2f\"%s%s%s}]",
          distance, battlevel, tempText, humText, pressureText);
  
  DEBUG_OUT(DEBUG_MIN_INFO, "Sending to ThingSpeak api\n");
  sendData(dataThingspeak, thingspeakIP, APIKEY_THINGSPEAK, PORT_THINGSPEAK, URL_THINGSPEAK);
}


/*****************************************************************
/* send data to thingspeak                                         *
/*****************************************************************/
void sendData(const char* dataThingspeak, const IPAddress& host, const char* apikey, const int httpPort, const char* url) {

  char dataThingspeakSend[35+strlen(dataThingspeak)+strlen(apikey)];
  sprintf(dataThingspeakSend, "{\"write_api_key\":\"%s\",\"updates\":%s}", apikey, dataThingspeak); 


  unsigned long start_send=micros();
  DEBUG_OUT(DEBUG_MIN_INFO, "Start connecting to %s\n", host.toString().c_str());

  char requestHead[170];

  sprintf(requestHead,
  "POST %s HTTP/1.1\r\n"
  "HOST: %s\r\n"
  "Content-Type: application/json\r\n"
  "Content-Length: %d\r\n"
  "Connection: close\r\n\r\n",
  url, host.toString().c_str(), strlen(dataThingspeakSend));

  WiFiClient client;

  client.setNoDelay(true);
  client.setTimeout(20000);

  if (!client.connect(host, httpPort)) {
    DEBUG_OUT(DEBUG_ERROR, "Connection failed!\n");
    return;
  }

  DEBUG_OUT(DEBUG_MIN_INFO, "Requesting URL: %s\n", url);
  DEBUG_OUT(DEBUG_MAX_INFO, "Request header:\n%s\n", requestHead);
  DEBUG_OUT(DEBUG_MAX_INFO, "Sending data:\n%s\n", dataThingspeakSend);

  client.print(requestHead);

  client.println(dataThingspeakSend);

  delay(10);

  // Read reply from server and print them
  DEBUG_OUT(DEBUG_MIN_INFO, "Reply: ");
  while(!client.available()){
    delay(100);
    DEBUG_OUT(DEBUG_MIN_INFO, ".");
  }
  // Read all the lines of the reply from server and print them to Serial
  DEBUG_OUT(DEBUG_MIN_INFO, "\n");
  while(client.available()){
    char c = client.read();
    DEBUG_OUT(DEBUG_MAX_INFO, "%c", c);
  }
  DEBUG_OUT(DEBUG_MIN_INFO, "End connection\n");
  DEBUG_OUT(DEBUG_MIN_INFO, "Time for sending data (microseconds): %lu\n", micros()-start_send);
  wdt_reset(); // nodemcu is alive
  yield();

}


void loop() {
  if(!modeResult) {
    ArduinoOTA.handle();
  }
  
  DEBUG_OUT(DEBUG_MIN_INFO, "OTA pin: %s\n", digitalRead(modePin)==0?"yes":"no");
  DEBUG_OUT(DEBUG_MIN_INFO, "IP address: %s\n", WiFi.localIP().toString().c_str());

}
