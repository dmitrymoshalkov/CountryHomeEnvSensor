//
// CountryHomeEnvSensor
//
// Country home environment sensor
// Developed with [embedXcode](http://embedXcode.weebly.com)
//
// Author 		Dmitry Moshalkov
// 				Dmitry Moshalkov
//
// Date			09.08.15 13:49
// Version		<#version#>
//
// Copyright	© Dmitry Moshalkov, 2015
// Licence		<#license#>
//
// See         ReadMe.txt for references
//

#include "Arduino.h"
#include <SPI.h>
#include <DHT.h>
#include <BH1750.h>
#include <MySensor.h>


#define NODE_ID 5

#define CHILD_ID_EXT_HUM 3  //AM2302
#define CHILD_ID_EXT_TEMP 4
#define HUMIDITY_SENSOR_DIGITAL_PIN 3
#define CHILD_ID_LIGHT 2

int samplingTime = 280;
uint16_t lastlux;
boolean metric = true;          // Celcius or fahrenheid
float lastTemp = -1;
float lastHum = -1;
long previousLighttMillis = 0;        // last time the sensors are updated
long LightsensorInterval = 30000;     // interval at which we will take a measurement ( 30 seconds)
long previousDHTMillis = 0;        // last time the sensors are updated
long DHTsensorInterval = 120000;     // interval at which we will take a measurement ( 30 seconds)


DHT dht;
BH1750 lightSensor;


MySensor gw;

MyMessage LightMsg(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
MyMessage msgExtHum(CHILD_ID_EXT_HUM, V_HUM);
MyMessage msgExtTemp(CHILD_ID_EXT_TEMP, V_TEMP);

void setup()
{
      Serial.begin(115200);
    Serial.println("Begin setup");
    // Initialize library and add callback for incoming messages
    gw.begin(NULL, NODE_ID, false);
    
    // Send the sketch version information to the gateway and Controller
    gw.sendSketchInfo("Country home env sensor", "1.0");
    

    
    // Register all sensors to gateway (they will be created as child devices)

    gw.present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
    gw.present(CHILD_ID_EXT_HUM, S_HUM);
    gw.present(CHILD_ID_EXT_TEMP, S_TEMP);
    
    
    dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN);
    
    metric = gw.getConfig().isMetric;
    
    lightSensor.begin();
    
    
    //Enable watchdog timer
        wdt_enable(WDTO_8S);
    
    Serial.println("End setup");  
}





void checkLight()
{
    
    unsigned long currentLightMillis = millis();
    if(currentLightMillis - previousLighttMillis > LightsensorInterval) {
        // Save the current millis
        previousLighttMillis = currentLightMillis;
        // take action here:
        // uint16_t lux = lightSensor.GetLightIntensity();
        uint16_t lux = lightSensor.readLightLevel();// Get Lux value
        Serial.print("Light: ");
        Serial.println(lux);
        if (lux != lastlux) {
            gw.send(LightMsg.set(lux));
            lastlux = lux;
        } 
        
        
    }    
    
}



void checkHum()
{
    unsigned long currentDHTMillis = millis();
    if(currentDHTMillis - previousDHTMillis > DHTsensorInterval) {
        // Save the current millis
        previousDHTMillis = currentDHTMillis;
        // take action here:
        
        //DHT 2121
        float temperature = dht.getTemperature();
        if (isnan(temperature)) {
            Serial.println("Failed reading temperature from DHT");
        } else if (temperature != lastTemp) {
            lastTemp = temperature;
            if (!metric) {
                temperature = dht.toFahrenheit(temperature);
            }
            gw.send(msgExtTemp.set(temperature, 1));
            Serial.print("T: ");
            Serial.println(temperature);
        }
        
        float humidity = dht.getHumidity();
        if (isnan(humidity)) {
            Serial.println("Failed reading humidity from DHT");
        } else if (humidity != lastHum) {
            lastHum = humidity;
            gw.send(msgExtHum.set(humidity, 1));
            Serial.print("H: ");
            Serial.println(humidity);
        }
        
        // end dht21
        
        
    }    
    
}



void loop(){
    
    
    checkHum();
    
    checkLight();
    
    
    // Alway process incoming messages whenever possible
    gw.process();
    
    //reset watchdog timer
        wdt_reset();
}



