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


#include <avr/wdt.h>
#include <SPI.h>
#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>
#include <MySensor.h>
#include <Adafruit_BMP085.h>
#include <Bounce2.h>


#define NODE_ID 5


#define RADIO_RESET_DELAY_TIME 20 //Задержка между сообщениями
#define MESSAGE_ACK_RETRY_COUNT 5  //количество попыток отсылки сообщения с запросом подтверждения


#define CHILD_ID_EXT_HUM 3  //DHT22
#define CHILD_ID_EXT_TEMP 4
#define CHILD_ID_LIGHT 2
#define CHILD_ID_OUT_HUM 5  //AM2301
#define CHILD_ID_OUT_TEMP 6
#define CHILD_BARO 7
#define CHILD_ID_DOOR 8


#define REBOOT_CHILD_ID 100
#define RECHECK_SENSOR_VALUES          102


#define HUMIDITY_SENSOR_DIGITAL_PIN 3
#define EXTHUMIDITY_SENSOR_DIGITAL_PIN 3
#define DOOR_SENSOR_DIGITAL_PIN 6

int samplingTime = 280;
uint16_t lastlux=10000;
boolean metric = true;          // Celcius or fahrenheid
float lastTemp = -1;
float lastHum = -1;
long previousLighttMillis = 0;        // last time the sensors are updated
long LightsensorInterval = 60000;     // interval at which we will take a measurement ( 30 seconds)
long previousDHTMillis = 0;        // last time the sensors are updated
long DHTsensorInterval = 120000;     // interval at which we will take a measurement ( 30 seconds)
long previousAM2301Millis = 0;        // last time the sensors are updated
long AM2301sensorInterval = 120000;     // interval at which we will take a measurement ( 30 seconds)
float lastAM2301Temp = -1;
float lastAM2301Hum = -1;
long PresssensorInterval = 120000;
long previousPressMillis = 0;
float lastPressure = -1;  
int oldDebouncerState=-1;

unsigned long previousMSMillis=0;
unsigned long MSsensorInterval=60000;

boolean boolMotionSensorDisabled = false;
boolean boolRecheckSensorValues = false;


boolean gotAck=false; //подтверждение от гейта о получении сообщения 
int iCount = MESSAGE_ACK_RETRY_COUNT;


DHT dht;
BH1750 lightSensor;
Adafruit_BMP085 bmp = Adafruit_BMP085();      // Digital Pressure Sensor 

Bounce debouncer = Bounce(); 

MySensor gw;

MyMessage LightMsg(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
MyMessage msgExtHum(CHILD_ID_EXT_HUM, V_HUM);
MyMessage msgExtTemp(CHILD_ID_EXT_TEMP, V_TEMP);
MyMessage pressureMsg(CHILD_BARO, V_PRESSURE);
MyMessage DoorMsg(CHILD_ID_DOOR, V_TRIPPED);

void setup()
{
      Serial.begin(115200);
    Serial.println("Begin setup");
    // Initialize library and add callback for incoming messages
    gw.begin(incomingMessage, NODE_ID, false);
    
    // Send the sketch version information to the gateway and Controller
    gw.sendSketchInfo("Country home env sensor", "1.0");
        gw.wait(RADIO_RESET_DELAY_TIME);         
    
 if (!bmp.begin()) {
    //Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) { }
  }  
    
    // Register all sensors to gateway (they will be created as child devices)

    gw.present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
        gw.wait(RADIO_RESET_DELAY_TIME);     

    gw.present(CHILD_ID_EXT_HUM, S_HUM);
        gw.wait(RADIO_RESET_DELAY_TIME);     

    gw.present(CHILD_ID_EXT_TEMP, S_TEMP);
        gw.wait(RADIO_RESET_DELAY_TIME);         
    
    dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN);
    
    metric = gw.getConfig().isMetric;
    
    lightSensor.begin();
  
      gw.present(CHILD_BARO, S_BARO);  
          gw.wait(RADIO_RESET_DELAY_TIME);     

    // Setup the button
  pinMode(DOOR_SENSOR_DIGITAL_PIN,INPUT);
  // Activate internal pull-up
  digitalWrite(DOOR_SENSOR_DIGITAL_PIN,HIGH);
  
  // After setting up the button, setup debouncer
  debouncer.attach(DOOR_SENSOR_DIGITAL_PIN);
  debouncer.interval(5);
 
   gw.present(CHILD_ID_DOOR, S_DOOR); 
        gw.wait(RADIO_RESET_DELAY_TIME);     

//reboot sensor command
     gw.present(REBOOT_CHILD_ID, S_BINARY);  
        gw.wait(RADIO_RESET_DELAY_TIME);     

//reget sensor values
  gw.present(RECHECK_SENSOR_VALUES, S_LIGHT); 
        gw.wait(RADIO_RESET_DELAY_TIME);   

 
 // Send initial state of sensors to gateway  
  debouncer.update();
  int value = debouncer.read();
  gw.send(DoorMsg.set(value==HIGH ? 1 : 0));  
     
  
    //Enable watchdog timer
        wdt_enable(WDTO_8S);
    
    Serial.println("End setup");  
}





void checkLight()
{
    
    unsigned long currentLightMillis = millis();
    if(currentLightMillis - previousLighttMillis > LightsensorInterval || boolRecheckSensorValues) {
        // Save the current millis
        previousLighttMillis = currentLightMillis;
        // take action here:
        // uint16_t lux = lightSensor.GetLightIntensity();
        uint16_t lux = lightSensor.readLightLevel();// Get Lux value
        Serial.print("Light: ");
        Serial.println(lux);
        if (lux != lastlux || boolRecheckSensorValues) {
            gw.send(LightMsg.set(lux));
            lastlux = lux;
        } 
        
        
    }    
    
}



void checkHum()
{
    unsigned long currentDHTMillis = millis();
    if(currentDHTMillis - previousDHTMillis > DHTsensorInterval || boolRecheckSensorValues) {
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


void checkPressure()
{

unsigned long currentPressMillis = millis();
if(currentPressMillis - previousPressMillis > PresssensorInterval || boolRecheckSensorValues) {
    // Save the current millis 
previousPressMillis = currentPressMillis;   

  float pressure = bmp.readSealevelPressure(146)/100; // 146 meters above sealevel
   float temperature = bmp.readTemperature();     
  if (pressure != lastPressure) {
    gw.send(pressureMsg.set((pressure * 0.75006375541921), 0));
    lastPressure = pressure;
  }  

  Serial.print("Pressure = ");
  Serial.print((pressure * 0.75006375541921));
  Serial.println(" mmHg");
}  
}


void incomingMessage(const MyMessage &message) {


  if (message.isAck())
  {
    gotAck = true;
    return;
  }


    if ( message.sensor == REBOOT_CHILD_ID ) {
             wdt_enable(WDTO_30MS);
              while(1) {};

     }

    if ( message.sensor == RECHECK_SENSOR_VALUES ) {
         
         if (message.getBool() == true)
         {
            boolRecheckSensorValues = true;
         }

     }  

        return;      
} 



void loop(){
   
   debouncer.update();
  // Get the update value
  int value = debouncer.read();
 
  if (value != oldDebouncerState || boolRecheckSensorValues) {

    //Отсылаем состояние сенсора с подтверждением получения
    iCount = MESSAGE_ACK_RETRY_COUNT;

    while( !gotAck && iCount > 0 )
    {
      gw.send(DoorMsg.set(value==HIGH ? 1 : 0), true);  // Send motion value to gw
      gw.wait(RADIO_RESET_DELAY_TIME);
      iCount--;
    }
    gotAck = false;

     oldDebouncerState = value;
         Serial.print("Door: ");
        Serial.println(value);
  } 
    
    checkHum();
    
    checkLight();
    
      checkPressure();

    if (boolRecheckSensorValues)
      {
       boolRecheckSensorValues = false;
      }
      
    // Alway process incoming messages whenever possible
    gw.process();
    
    //reset watchdog timer
        wdt_reset();
}



