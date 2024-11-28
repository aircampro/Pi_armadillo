// simple stack chan as a talking clock or environment monitor
//
#include <M5Stack.h>
#include <WiFi.h>
#include <time.h>
#include <stdint.h>
#include "AquesTalkTTS.h"
#include "M5UnitENV.h"

//const char* licencekey = "XXX-XXX-XXX";                                       // AquesTalk-ESP licencekey
const char* licencekey = NULL;                                                  // AquesTalk-ESP licencekey

SCD4X scd4x;

// sync the time with the ntp server
const char* ssid       = "WIFI-SSID";                                            // WiFi SSID
const char* password   = "PASSWORD";                                             // WiFi PW
const char* ntpServer  =  "ntp.jst.mfeed.ad.jp";                                 // ntp server address
const long  gmtOffset_sec = 9 * 3600;
const int daylightOffset_sec = 0;
char time_str[100];
int16_t modenum = 0;

void setup() {
    Serial.begin(115200);
	M5.begin();
    int16_t iret = 0;
	
    if (!scd4x.begin(&Wire, SCD4X_I2C_ADDR, 21, 22, 400000U)) {                  // connect env monitor in i2c
        Serial.println("Couldn't find SCD4X");
        while (1) delay(1);
    }

    uint16_t error;
    // stop potentially previously started measurement
    error = scd4x.stopPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    }

    // Start Measurement
    error = scd4x.startPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    }

    Serial.println("Waiting for first measurement... (5 sec)");
	
    // Init Text-to-Speech (AquesTalk-ESP + I2S + Internal-DAC)
    iret = TTS.create(licencekey);
    if (iret) {
       Serial.print("ERR: TTS_create():");
       Serial.println(iret);
    }
}

void loop() {
	char koe[200];
	char tmp[200];
	char hum[200];
    int16_t iret1 = 0;
    float value = 0.0f;
	M5.update();
    if (scd4x.update())  // readMeasurement will return true when fresh data is available
    {
        Serial.println();

        value = scd4x.getCO2();		
        Serial.print(F("CO2(ppm):"));
        Serial.print(value);
        sprintf(koe,"CO2 is %f ppm",value);

        Serial.print(, 1);
        value = scd4x.getTemperature();		
        Serial.print(F("\tTemperature(C):"));
        Serial.print(value);
        sprintf(tmp,"Temperature is %f Celcius",value);
		
        value = scd4x.getHumidity();		
        Serial.print(F("\tHumidity(%RH):"));
        Serial.print(value);
        sprintf(hum,"Humidity is %f percent Relative Humidity",value);
		
        Serial.println();
    } else {
        Serial.print(F("."));
    }
	if (M5.BtnA.wasPressed()) {
       iret1 = TTS.play(koe, 100);
       iret1 = TTS.play(hum, 100);	
       iret1 = TTS.play(tmp, 100);
	}
	if (M5.BtnB.wasPressed()) {
       iret1 = TTS.play(time_str, 100);
	}
    delay(1000);
}

void taskClock(void *arg)
{

  //connect to WiFi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println(" CONNECTED");
  
  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  getLocalTime(&timeinfo);  

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  for(;;){
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
    }
    else {
      Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
	  sprintf(time_str,"time is hour %d minute %d seconds %d",timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    }
    delay(1000);
  }
  
}