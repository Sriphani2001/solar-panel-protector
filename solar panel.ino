#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <Arduino_LSM6DS3.h>
// Sensor pins
#define sensorPower 7
#define sensorPin 8
#define dirPin 10
#define stepPin 11
#define stepsPerRevolution 700
#include<Servo.h>
Servo Myservo;

boolean stati=false;


///////please enter your SSID and Password
//char ssid[] = "IOTLab";        // your network SSID (IOTLab) Use it in the Labb
char ssid[] = "realme3";
char pass[] = "0123456789";    // your network password (use for WPA, or use as key for WEP)
//char ssid[] = "AlperWIFI";
//char pass[] = "2212716582"; 
int status = WL_IDLE_STATUS;     // the WiFi radio's status

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// MQTT server setup
//const char broker[] = "192.168.1.11"; 
const char broker[] = "185.57.107.24"; 
int        port     = 1883;
const char topic[]  = "P11";

//Test varaibles

String PoParam="";
String DID;
float Xaxis, Yaxis, Zaxis;
const long interval = 100;
unsigned long Millis = 0;
String subMessage = "";
String subMessage2 = "";
String subMessage3 = "";
String subMessage4= "";
String subMessage5= "";
//float subMessage6= "";
boolean stat=0;
String subString ="Led is OFF";
String subString2 ="rain status";
String subString3 ="smoke status";
String subString4 ="presence detection";
String subString5 ="day status";
String subString6 ="speedw";

float x, y, z;
int smokeA0 = A0;
int const trigPin = 4;
int const echoPin = 3;
int const trigPin2 = 6;
int const echoPin2 = 5;

int LED = 13;
float startTime;
float endTime;
float duration;
float speed0;
byte timerRunning;
float prev;

int pos;


void setup() {
   pinMode(LED, OUTPUT);
   pinMode(sensorPower, OUTPUT);
  digitalWrite(sensorPower, LOW);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  pinMode(trigPin2, OUTPUT); 
  pinMode(echoPin2, INPUT);
  Myservo.attach(2);
    pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

   pinMode(smokeA0, INPUT);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
    
  }

    while (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
  }

  // subscribe to a topic
  mqttClient.subscribe(topic);

  // you're connected now, so print out the data:
  Serial.println("You're connected to the network");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

   if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  Serial.print(IMU.accelerationSampleRate());

}

void loop() {
  int val = readSensor();
 

 int sv = analogRead(A5);
  StaticJsonDocument<200> OutMes;
  StaticJsonDocument<200> inMes;

    IMU.readAcceleration(x, y, z);
  
//Data Serialization
   // OutMes[String("ID")]="Device 1";
   
    OutMes["smokestatus"]=(subString3);
    OutMes["Rainstatus"]=(subString2);
    OutMes["presencedetection"]=(subString4);
    OutMes["speedw"]=(speed0);
     // OutMes["daystatus"]=(subString5);  
   serializeJson(OutMes, PoParam);
   //Serial.println(PoParam);

   //Read incoming message
  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    subMessage = "";
   
    // use the Stream interface to print the contents
    while (mqttClient.available()) {
      subMessage = subMessage + (char)mqttClient.read();
    }
   
    //Parse Messages from Sender
    deserializeJson(inMes, subMessage);
    DID=inMes["ID"].as<String>(); 
    Xaxis=inMes["X"]; 
    Yaxis=inMes["Y"];
    Zaxis=inMes["Z"];
    if(DID=="Device 2"){
       Serial.println(subMessage);
      }

     
   

  // Determine status of rain

if(subMessage2 == "") {
      if(val==1){
        Serial.println("Sky is Clear, bro");
        subString2 = "Sky is Clear, bro";
        }
        else{
           Serial.println("Man, It's raining");
           subString2 = "Man, It's raining";
          } 
  }
          
if (subMessage3 == ""){          
  if (sv< 30){
    Serial.println(" Smokey gas over panel");
    subString3 = "Smokey gas over panel";
  }
  else{
    Serial.println(" Air is fair");
    subString3 = "Air is fair";
  }
}
// for bird scaring
  int duration, distance, duration2, distance2;
  digitalWrite(trigPin, HIGH); 
  delay(1);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  // Distance is half the duration devided by 29.1 (from datasheet)
  distance = (duration/2) / 29.1;
  digitalWrite(trigPin2, HIGH); 
  delay(1);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = (duration2/2) / 29.1;
  //for first us sensor.
  if (subMessage4 == ""){  
    if (distance <= 40 && distance >= 0)
    {
      
     Serial.println("Something is there, action is taken");
     subString4 = "Something is there";
      for(pos=0;pos<=180;pos++){
      Myservo.write(pos);
     delay(1);
    }
    }
    else { 
     Serial.println("CLEAR");
      subString4 = "nothing on the panel";
    }
  }
  //for second us sensor.
  if (distance2 <= 40 && distance2 >= 0)
    {
      
     Serial.println("Something is there action is taken");
      
      for(pos=0;pos<=180;pos++){
      Myservo.write(pos);
     delay(1);
    }
    }
    else {
     Serial.println("CLEAR");
    }
    // wind sensor model
   int analogValue = analogRead(A2)/4;
  if (timerRunning == 0 && analogValue > 200) {
   // Serial.println(" obstacle detected");
    prev =endTime;
    startTime = millis();
    timerRunning = 1;
  } 
  else if(timerRunning == 1)
    {
   if( analogValue > 200)
   {
    digitalWrite(LED, LOW);
   }
    else if (analogValue <= 200)
    {
      endTime = millis();
    timerRunning = 0;
    digitalWrite(LED, HIGH);
    }
    }
duration =  startTime-prev;
    
    speed0= (28*0.03*1000)/duration ;
     Serial.print ("speed(in m/hr): ");
    Serial.println (speed0);
  
  
  
  delay(500);  // Take a reading every second
  Serial.println();
  delay(1000);
    
    //Taking Action acording to subscribed message
    if(subMessage == "ONOFF") {
      if(stat==1){
        digitalWrite(LED_BUILTIN, LOW);
        stat=0;
        subString = "Led is OFF";
        }
        else if(stat==0){
           digitalWrite(LED_BUILTIN, HIGH);
           stat=1;
           subString = "Led is ON";
          }
     
    } 
  }

  //Publishing measured data through MQTT
   if(millis()-Millis>interval){
       mqttClient.beginMessage(topic);
        mqttClient.print(PoParam);
        mqttClient.endMessage();
        Millis=millis();
  }

  delay(10);
  PoParam="";
 
}
int readSensor() {
  digitalWrite(sensorPower, HIGH);  // Turn the sensor ON
  delay(10);              // Allow power to settle
  int val = digitalRead(sensorPin); // Read the sensor output
  digitalWrite(sensorPower, LOW);   // Turn the sensor OFF
  return val;             // Return the value

 
}