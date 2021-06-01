#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>

#include "ACS712.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVO_MIN 105
#define SERVO_MAX 415

// our servo # counter
uint8_t servonum = 0;
int SERVO_TARGET[16];
int SERVO_CURRENT[16];

#define SDA_PIN 16
#define SCL_PIN 4
#define BUTTON 13

ACS712 sensor(ACS712_30A, A0);
//ACS712 sensor(ACS712_05B, A0);

int value = 105;
unsigned long long t;
boolean bounce=false;
boolean first=true;
boolean test=true;
boolean flipflop=false;
boolean measure_time=false;
int n=0;
int count=0;

//-WiFi----------------------------------------
WiFiMulti wifiMulti;
WiFiClient wifi_client;
boolean wifi_connected=false;

//-MqTT section---------------------------------
IPAddress server(192,168,43,100);
PubSubClient mqtt_client(wifi_client);
boolean mqtt_connected=false;
boolean mqtt_subscribed=false;
int cmdid=-1;
int servo_num=-1;
int servo_angle=-1;
float ImA;

//----------------------------------------------
int STATE = 0;

//----------------------------------------------

SemaphoreHandle_t xMutex;
unsigned long long t0;
//----------------------------------------------

void setup() {
  Serial.begin(9600);
  Serial.println("Robot Control Board v 0.000");

  xMutex = xSemaphoreCreateMutex();
  
  pinMode(BUTTON, INPUT_PULLUP);
    
  Wire.begin(SDA_PIN, SCL_PIN);
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true,true);
  delay(50);

  //WIFI  
  WiFi.begin("Mobile","*******");

  //MQTT
  mqtt_client.setServer(server, 60000);
  mqtt_client.setCallback(mqtt_callback);

  //ACS
  //int zero = sensor.calibrate();
  for(int i=0;i<16;i++){
    
    SERVO_TARGET[i]=0;
    SERVO_CURRENT[i]=0;
    }
  
  STATE=0;

  //disable WDT
  disableCore0WDT();
  disableCore1WDT();
    
  //task :-)
  xTaskCreatePinnedToCore(
                      taskMotor,         /* Task function. */
                      "taskMotor",       /* String with name of task. */
                      10000,            /* Stack size in bytes. */
                      NULL,             /* Parameter passed as input of the task */
                      10,                /* Priority of the task. */
                      NULL,             /* Task handle. */
                      0);            

  delay(500);
    
  xTaskCreatePinnedToCore(
                      taskControl,         /* Task function. */
                      "taskControl",       /* String with name of task. */
                      20000,            /* Stack size in bytes. */
                      NULL,             /* Parameter passed as input of the task */
                      5,                /* Priority of the task. */
                      NULL,             /* Task handle. */
                      1);   
}

void loop() {

  vTaskDelete(NULL);
  }


void taskMotor( void * parameter ) {

  while(true){
    
    yield();
    vTaskDelay(0);
      
    xSemaphoreTake( xMutex, portMAX_DELAY );

    if(millis()-t0>100){

      float _ImA = sensor.getCurrentDC();

      ImA = (ImA + _ImA)/2;
      //Serial.println(ImA);
      
      t0=millis();
      }

    //Serial.println("-");

    boolean servo_update=false;
    unsigned long long tm=millis();
    
    //CONTROL
    for(int i=0;i<16;i++){

      if(SERVO_CURRENT[i]!=SERVO_TARGET[i]){

        Serial.print("SERVO ");Serial.print(i);Serial.print(" = ");Serial.println(SERVO_TARGET[i]);
        SERVO_CURRENT[i]=SERVO_TARGET[i];            
        pwm.setPWM(i, 0, pulseWidth(SERVO_CURRENT[i]));        

        servo_update=true;
        }
      
      }

    if(servo_update){
      Serial.print("Receive Servo Motor control T=");
      Serial.println(millis()-tm);
      }
    
    xSemaphoreGive( xMutex );
    

  }//while

}
  
unsigned long long t2=millis();

void taskControl( void * parameter ) {

  while(true){

    yield();
    vTaskDelay(50);

    xSemaphoreTake( xMutex, portMAX_DELAY );

    //Serial.println("/");
    
    //NETWORK
    if(mqtt_connected){

      if(millis()-t2>200){

        int _ImA = ImA*100;
        char temp[16];  
        sprintf(temp,"%d",_ImA);

        String payload = String(temp);
        mqtt_client.publish("app/control/servo/status/ima",payload.c_str());

        t2=millis();
        }
        
      mqtt_client.loop();                
      }
    
    switch(STATE){
  
      case 0:
        
        if(WiFi.status() == WL_CONNECTED) {
            
            STATE=10;
            }
            
        break;
  
      case 10:
        
        {
          IPAddress local = WiFi.localIP();
          Serial.print("WiFi Local IP:");
          Serial.println(local);
          STATE=20;
        }
        break;
  
      case 20:
  
        STATE=30;
        wifi_connected=true;
          
        if (WiFi.status() != WL_CONNECTED) {
    
          Serial.println("WiFi handle disconnect");
          wifi_connected=false;
          STATE=0;
          }
          
        break;
  
      case 30:
        
        if (!mqtt_client.connected()) {
  
            mqtt_connected=false;
            mqtt_subscribed=false;          
            
            char RAND[5];
            #if defined(ESP8266)
              sprintf(RAND,"%04d",ESP8266TrueRandom.random(1,9999));
            #endif
            
            #if defined(ESP32)
              sprintf(RAND,"%04d",random(1,9999));
            #endif
            
            String CLIENT_ID = "id"+String(RAND);
            
            Serial.println("mqtt connect "+CLIENT_ID);
            if (mqtt_client.connect( CLIENT_ID.c_str(), "****", "*****")) {
    
              mqtt_connected=true;
              
              Serial.println("connected mqtt wifi");
              /*
              // Once connected, publish an announcement...
              //mqtt_wifi_client.publish("outTopic", "hello world");
              // ... and resubscribe
              Serial.println("Subscribe to /app/command/network");
              mqtt_wifi_client.subscribe("/app/command/network");          
              */
              Serial.println("Subscribe to app/control/servo/+");
              mqtt_client.subscribe("app/control/servo/+");          
              mqtt_client.subscribe("app/control/servo");          
              
              mqtt_subscribed=true;            
              }
          
            }
  
        STATE=40;
    
        break;

      case 40:
        
        if (!mqtt_client.connected()) {

          mqtt_connected=false;
          mqtt_subscribed=false;   
          
          STATE=20;
          }
        break;
        
      default:
        break;
      }

    xSemaphoreGive( xMutex );

    }//while
  }
