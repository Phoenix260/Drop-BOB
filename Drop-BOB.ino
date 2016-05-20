

/* This sketch is created for use of a cold drip coffeemaker with a THING DEV from Sparkfun using the ESP8266 wifi chip. 
Copyright: Bobby Lumia, may be used and modified for personal use only. No resale.
Please reference my project blog: http://bobbobblogs.blogspot.ca/ if useing this sketch in your work.

When uploading to ESP8266 with Version 2.1 in Arduino boards use 80MHz and 115200 to prevent board crashes (known issue with servo)

SUPERNOTE: IN ORDER TO PROGRAM, YOU NEED TO REMOUVE THE JUMPER. // Possibly // Not Not always required
*/
//#include <VarSpeedServo.h> // for the servo - better library with ability to decide speed of servo and wait to complete
#include <Servo.h>  // for the servo
#include <stdlib.h> // Include Standard Library
#include <SimpleTimer.h>
#include <Wire.h>

// for the ESP wifi - Blynk App =====\/
//#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <SPI.h>
#include <ESP8266WiFi.h> 
#include <BlynkSimpleEsp8266.h> 
const char* ssid     = "ASUS-BOB-BOB-Jelly-Jube";
const char* password = "fuck off get your own";
const char auth[] = "58620a59ec64485aa3d5bfd00edae573"; //=====/\

SimpleTimer timer;

//============================= \/ DEFINE PINS \/ ==============

const int ServoPIN = 2; // attaches the servo on pin 2 to the servo object ==================== PIN 2
const int photo_interuptor_PIN = 14; // attaches the photo-interuptor to the 14 pin ============ PIN 14
const int LED_PIN = 5; // attaches the LED to pin 5 (may be other pins on other boards ========== PIN 5
const int WakePin = 16; // wake pin is ESP8266 pin 16. (grounding required to wake) =============PIN 16
const int SleepPin = 12; // sleep pin is 12 (any pin would do). (grounding to sleep to keep consistent) =============PIN 12

//================================\/ PID \/=============

unsigned long lastTime = 0;
double errSum = 0, lastErr = 0, error = 0, dErr = 0;
const double kp = 0.7, ki = 0.4/60000.0, kd = 0; //kp tunes to 1.3 alone

//====================general variables==================================

//VarSpeedServo myservo;  // create servo object to control a servo 
Servo myservo;  // create servo object to control a servo 

long count = 0;
long delta = 0;
int state = HIGH; // HIGH is high signal, not broken light beam.
double lastDrop = millis();
float DPM = 0;
int first_drop = 1; // Avoid the first drop in servo settings
int creep = 0;
int voltage = 0;
int button = 0;
int restart = 0;
int pause = 0;
double slide_time = 0;
bool led_gp5;
float A = 0;
float B = 0;
long uptime = 0;
float Low_LED = millis();
float High_LED = millis();
float Servo_adjust = millis(); //servo adjustment time keeper

// Time until sleep (in seconds):
const int sleepTimeS = 500;

// SERVO SETUP VALUES ======================================================
int set_DPM = 6;
int Servo_Val = 90; // starting servo value (0 is full open 180 is full close)
const int servo_min = 15;
const int servo_max = 195;
const int Servo_update_Speed = 500; // update every X milliseconds
const int Servo_movements = 1;      // how much to update servo position by

// RUNNING AVERAGE VARIABLES =========================================
const int numReadings = 3;             // minimum 2
double readings[numReadings];      // the readings from the analog input (subtract 1 since it starts at zero)
double total = 0;                  // the running total
int index_n = 0;                  // the index_n of the current reading
int start_avg = 0;
// RUNNING AVERAGE VARIABLES =========================================
const int numReadings_avg = 10;             // minimum 2
double readings_avg[numReadings_avg];      // the readings from the analog input (subtract 1 since it starts at zero)
double total_avg = 0;                  // the running total
int index_n_avg = 0;                  // the index_n of the current reading
int start_avg_avg = 0;

float DPM_avg = 0;
//====================================================================

// TUNING VARIABLES =================================================
const int tuning_drops = 3; // how many drops before closing the servo a little in the tuning mode
float add = 0;
float DPM_tune_avg = 0;
//====================================================================

//=========================================================================BLYNK functions & Widgets=====

void pause_requests(){
  while(pause == 1){
     Blynk.run(); 
     
     if (!digitalRead(SleepPin)){
        myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
        delay(15);
        myservo.write(servo_max);
        Serial.println("...Going to Sleep...");
        delay(1000);
        ESP.deepSleep(0, WAKE_RF_DEFAULT); // Sleep forever, until Pin#16 is un-grounded (button un-pushed)
     }
     
     int lapse = millis() - slide_time;

     if (slide_time == 0){                  // If the pause comes from the app, close the servo to prevent drops (true pause)
        myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
        delay(15);
        myservo.write(servo_max);//,30,true);
        Blynk.virtualWrite(V0, servo_max);
     }
     
     if (lapse > 1000 && slide_time > 0){   // if the pause comes from a parameter change
        pause = 0;
        slide_time = 0;
        myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
        delay(15);
        myservo.write(Servo_Val);//,30,true);
        Blynk.virtualWrite(V0, Servo_Val);
        Blynk.virtualWrite(V2, set_DPM);
     }
     
     if (pause == 0) {                       // exit clause to put the valve back after pausing
        myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
        delay(15);
        myservo.write(Servo_Val);//,30,true);
        Blynk.virtualWrite(V0, myservo.read());
     }
  }
}

BLYNK_WRITE(V1){ //V1 pushbutton in Blynk app that simulates a drop of water (for testing) - INPUT
  if (param.asInt() == 1){
    voltage = 5.0;
    button = 1;
  }
}

BLYNK_WRITE(V3){ //V3 is "Blynk slider for DPM setting" - INPUT
  pause = 1;
  slide_time = millis();
  set_DPM = param[0].asInt();
}

BLYNK_WRITE(V6){ //V6 is "Blynk restart button" - INPUT
  restart = param.asInt();
}

BLYNK_WRITE(V8){ //V8 is "Blynk Pause button" - INPUT
  pause = param.asInt();
}

BLYNK_WRITE(V9){ //V9 is "Blynk Manual servo control" - INPUT
  pause = 1;
  slide_time = millis();
  Servo_Val = param.asInt();
}

void open_up(){
  if ((millis()-lastDrop) > 1.25*(60000.0/set_DPM)){ 
    uint32_t temp_delta = millis() - lastDrop;
    float temp_DPM = 60000.0/temp_delta;
    
    Servo_Val = Servo_Val - 0.5*(set_DPM - temp_DPM); // if the servo closed and no drops are comming for too long open it up a little.
    
    if (Servo_Val < servo_min){
      Servo_Val = servo_min;
    }

    myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
    delay(15);  
    myservo.write(Servo_Val);//,30,true);
    Blynk.virtualWrite(V0, myservo.read());
    creep = 0;
  }
}

void tune(){
  for(Servo_Val - 5; Servo_Val < 190; Servo_Val++){
    myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
    delay(15);
    myservo.write(Servo_Val);//,30,true); 
    Blynk.virtualWrite(V0, Servo_Val);
    add = 0;
    
    for(int run_tune = 0; run_tune < tuning_drops; run_tune++){
      
      while(analogRead(photo_interuptor_PIN)>500){
        Blynk.run();
        
        if (!digitalRead(SleepPin)){
          myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
          delay(15);
          myservo.write(servo_max);
          Serial.println("...Going to Sleep...");         
          delay(1000);         
          ESP.deepSleep(0, WAKE_RF_DEFAULT); // Sleep forever, until Pin#16 is un-grounded (button un-pushed)
        }
     
        timer.run();
        pause_requests();
        
        if (millis() - High_LED > 500){ // add LED indicator for tuning
          digitalWrite(LED_PIN, LOW);  
          Low_LED = millis();
          High_LED = INFINITY;
        }
        if (millis() - Low_LED > 500){  // add LED indicator for tuning
          digitalWrite(LED_PIN, HIGH); 
          High_LED = millis();
          Low_LED = INFINITY;
        }
        
        //if (millis() - lastDrop > 2500) myservo.detach(); // turn off servo after 2.5 seconds to reduce jitter noise
        /*wait for a drop ... accept Blynk requests, timer requests, and pause requests*/
      }
      
      count++;
      delta = millis() - lastDrop; //get the difference in time between each drop
      lastDrop = millis(); // remember time of last drop to prevent bouncing.
      DPM = 60000.0 / delta; // get drop per min

      add = add + DPM;
      DPM_avg = add / (run_tune + 1);
      
      Serial.print(delta);Serial.print("\t");Serial.print("ms");
      Serial.print("\t");
      Serial.print(count);
      Serial.print("\t");
      Serial.print(DPM);Serial.print("\t");Serial.print("DPM");
      Serial.print("\t");
      Serial.print(set_DPM);Serial.print("\t");Serial.print("Setpoint");
      Serial.print("\t");
      Serial.print(myservo.read());Serial.print("\t");Serial.println("Servo_val");//*/

      Blynk.virtualWrite(V10, DPM);
      Blynk.virtualWrite(V7, DPM_avg);
      Blynk.virtualWrite(V4, count);
      Blynk.virtualWrite(V0, myservo.read());
      Blynk.virtualWrite(V5, millis() / 1000);
      Blynk.virtualWrite(V2, set_DPM);

      while (millis() - lastDrop < 100) {Blynk.run();/*debounce*/}
    }
    DPM_tune_avg = add / tuning_drops;
    
    if (DPM_tune_avg < set_DPM) {break;}
    if (restart == 1) {restart = 0; break;}
  }
}

BLYNK_WRITE(V11){ //V11 pushbutton in Blynk app for calling up the tune function below - INPUT
  if (param.asInt() == 1){
    tune();
  }
}

//=====================================================================START SETUP===========================

void setup()
{    
  Serial.begin(9600);
  delay(10);

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  /*while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }*/ //don't wait until wifi connects ... in case someone needs to use it out of the box with no setup

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  while(myservo.read() != Servo_Val){
    
    if(myservo.read() > Servo_Val){
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(15);
      myservo.write(myservo.read() - Servo_movements);
    }
    else if(myservo.read() < Servo_Val){
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(15);
      myservo.write(myservo.read() + Servo_movements);
    }

    delay(50);
    
  }
  
  Blynk.begin(auth, ssid, password);
  
  Serial.println("Dropcounter 0.1");
  Serial.println();
  
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0; // Initialize the array
    
  pinMode(photo_interuptor_PIN, INPUT);
  pinMode(ServoPIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SleepPin, INPUT_PULLUP); //Set Sleep monitor pin as INPUT ... also make it pullup to HIGH by default
  pinMode(WakePin, INPUT_PULLUP); //Set Wake monitor pin as INPUT ... also make it pullup to HIGH by default

  // Blink the LED pin during setup (for fun)
  digitalWrite(LED_PIN, LOW);  led_gp5 = LOW;
  delay(500);
  digitalWrite(LED_PIN, HIGH); led_gp5 = HIGH;
  delay(500);
  digitalWrite(LED_PIN, LOW); led_gp5 = LOW;
  delay(500);
  digitalWrite(LED_PIN, HIGH); led_gp5 = HIGH;
  delay(500);
  
  Blynk.virtualWrite(V4, count);
  Blynk.virtualWrite(V7, DPM);
  Blynk.virtualWrite(V2, set_DPM);
  Blynk.virtualWrite(V0, myservo.read());

  //Blynk.tweet("Brewing a fresh pot of Cold Drip Coffee with my Drop-BOB v1.0: Check it out at www.bobbobblogs.blogspot.com");
  
  timer.setInterval(10000L, open_up); // open up the servo every 5 seconds if no drops come ... not a Blynk update

  tune(); //start by tuning the system
}//================================================================================END SETUP========================

void loop(){

  pause_requests(); //accept pause requests

  if ((millis()-uptime) > 1000){
    uptime = millis();
    Blynk.virtualWrite(V5, millis()/1000);
  }
 
  Blynk.run(); //Constant Blynk connection
  
  if (!digitalRead(SleepPin)){
    myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
    delay(15);
    myservo.write(servo_max);
    Serial.println("...Going to Sleep...");         
    delay(1000);         
    ESP.deepSleep(0, WAKE_RF_DEFAULT); // Sleep forever, until Pin#16 is un-grounded (button un-pushed)
  }
  
  timer.run(); // Initiates SimpleTimer

  digitalWrite(LED_PIN, HIGH); // for some odd reason ... LED PIN to "HIGH" means "off"
    
  int raw = analogRead(photo_interuptor_PIN); // read the drop sensor
  
  if(button == 0){
    voltage = 5.0 * raw / 1023; // convert it to voltage
  }
    
  if (voltage < 1.0 && state == HIGH) {//-----------------------------------------------------------------------big loop 1
    
    state = LOW;
    button = 0;

    digitalWrite(LED_PIN, LOW); // for some odd reason ... LED PIN to "LOW" means "on"
    
    count++;
    delta = millis() - lastDrop; //get the difference in time between each drop
    lastDrop = millis(); // remember time of last drop to prevent bouncing.
    DPM = 60000.0 / delta; // get drop per min

    //====================================================================== 3 reading smoothing
    total = total - readings[index_n];  // subtract the last reading:       
    readings[index_n] = DPM; 
    total = total + readings[index_n];   // add the reading to the total:    
    index_n = index_n + 1;                    
    
    if (index_n >= numReadings) {  
      start_avg = 1;      
      index_n = 0;}      
    if (start_avg == 1) {
      DPM = total / numReadings;}   // drop the instantaneou DPM and use the running ave after all initial readings taken

    //======================================================================== 10 reading average
    total_avg = total_avg - readings_avg[index_n_avg];  // subtract the last reading:       
    readings_avg[index_n_avg] = DPM; 
    total_avg = total_avg + readings_avg[index_n_avg];   // add the reading to the total:    
    index_n_avg = index_n_avg + 1;                    
    
    if (index_n_avg >= numReadings_avg) {  
      start_avg_avg = 1;      
      index_n_avg = 0;}      
    if (start_avg_avg == 1) {
      DPM_avg = total_avg / numReadings_avg;}   // drop the instantaneou DPM and use the running ave after all initial readings taken
      
    /*Compute all the working error variables*/
    error = set_DPM - DPM;
    errSum += (error * delta);
    dErr = (error - lastErr) / delta;    
   
    if ( abs(DPM_avg - set_DPM) > 1.0 && first_drop == 0) { //to add forward or reverse bias Uncomment the elseif loop and the DPM condition
      Servo_Val = Servo_Val - kp * error - ki * errSum - kd * dErr; // Set servo change depending on how far away from set_DPM you are at
      if (Servo_Val < servo_min) Servo_Val = servo_min;
      if (Servo_Val > servo_max) Servo_Val = servo_max;
      
      /*
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(15);
      myservo.write(Servo_Val);//,30,true);*/ //REMOUVED TO ALLOW FOR FINER SERVO CONTROL ITERATIVELY
      
    }/*
    else if ( DPM > set_DPM && first_drop == 0) { // This is to give a Forward or Reverse BIAS (commented out)
      Servo_Val = Servo_Val - kp * error - ki * errSum - kd * dErr; // Set servo change depending on how far away from set_DPM you are at
      if (Servo_Val < servo_min) Servo_Val = servo_min;
      if (Servo_Val > servo_max) Servo_Val = servo_max;
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(15);
      myservo.write(Servo_Val);//,30,true);
    }*/
    
    Serial.print(delta);Serial.print("\t");Serial.print("ms");
    Serial.print("\t");
    Serial.print(count);
    Serial.print("\t");
    Serial.print(DPM);Serial.print("\t");Serial.print("DPM");
    Serial.print("\t");
    Serial.print(set_DPM);Serial.print("\t");Serial.print("Setpoint");
    Serial.print("\t");
    Serial.print(myservo.read());Serial.print("\t");Serial.print("Servo_val");//*
    Serial.print("\t");
    Serial.print(kp * error);Serial.print("\t");Serial.print("error");
    Serial.print("\t");
    Serial.print(ki * errSum);Serial.print("\t");Serial.print("errSum");
    Serial.print("\t");
    Serial.print(kd * dErr);Serial.print("\t");Serial.println("dErr");//*/

    /*for (int blah = 0; blah < numReadings_avg; blah++){ // show me what values are stored in the arrays
      Serial.print(readings_avg[blah]);Serial.print("\t");
    }
    Serial.println();
    for (int blah = 0; blah < numReadings; blah++){
      Serial.print(readings[blah]);Serial.print("\t");
    }
    Serial.println();*/
     
    first_drop = 0;

    Blynk.virtualWrite(V10, DPM);
    Blynk.virtualWrite(V7, DPM_avg);
    Blynk.virtualWrite(V4, count);
    Blynk.virtualWrite(V0, myservo.read());
    Blynk.virtualWrite(V2, set_DPM);
    
  }//----------------------------------------------------------------------------------------------------big loop 1

  if (voltage < 1.0 && state == LOW ){  // end of pulse, now we may expect a new one, DEBOUNCE
    if (millis() - lastDrop > 50) state = HIGH; // only go back to state high if some time has passed.
  }

  //if (millis() - lastDrop > 5000) myservo.detach(); // turn off servo after 5 seconds to reduce jitter noise

  lastErr = error;

  if( (millis() - Servo_adjust) > Servo_update_Speed){
    Servo_adjust = millis();
    
    if(myservo.read() > Servo_Val){
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(15);
      myservo.write(myservo.read() - Servo_movements);
      Serial.println("opening valve");
    }
    else if(myservo.read() < Servo_Val){
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(15);
      myservo.write(myservo.read() + Servo_movements);
      Serial.println("closing valve");
    }
  }
  
  if( (millis()-lastDrop) > 180000 && Servo_Val < (servo_min + 2)){
    Serial.println();Serial.println("FINISHED!!!");
    //Blynk.tweet("Brew DONE!!: www.bobbobblogs.blogspot.com");
    myservo.detach();
    while(restart == 0) {Blynk.run();} // when finished do nothing but listen for Blynk app
    myservo.attach(ServoPIN);
    restart = 0;
    lastDrop = millis();
    Servo_Val = 140;
    Serial.println();Serial.print("Restarting");delay(700);Serial.print(" .");delay(700);Serial.print(".");delay(700);Serial.print(".");delay(10);Serial.println(" waiting for drops");Serial.println();
  }
}
