/***********************************************************************************************
    This sketch is created for use of a cold drip coffeemaker with a THING DEV from Sparkfun using the ESP8266 wifi chip.
    Copyright: Bobby Lumia, may be used and modified for personal use only. No resale.
    Please reference my project blog: http://bobbobblogs.blogspot.ca/ if using this sketch in your work.
    When uploading to ESP8266 with Version 2.2 in Arduino boards use 80MHz and 115200 to prevent board crashes (known issue
    with servo on 160Mhz)

    NOTE: IN ORDER TO PROGRAM, YOU NEED TO REMOUVE THE JUMPER. // Possibly // Not Not always required ... actually never
    required. dunno why

    Resources: Blynk Arduino Library: https://github.com/blynkkk/blynk-library/releases/tag/v0.3.1

    Additional Boards Manager: http://arduino.esp8266.com/stable/package_esp8266com_index.json
    Then install ESP8266 in Additional boards to select the Sparkfun ESP8266 Thing Dev

    Development environment specifics:
    Arduino IDE 1.6.9
    SparkFun ESP8266 Thing Dev: https://www.sparkfun.com/products/13711

    NOTES:  - Upload from Aurduino only at 80Mhz
            - On the Blynk App make sure all widgets are "PUSH" ... setting manual intervals slows down this code
            - 
************************************************************************************************/

#include <FS.h>                   //Include File System (to save parameter data). this needs to be first

////////////////////////////////////
// Blynk Virtual Variable Mapping //
////////////////////////////////////

#define SERVO_ANGLE_VIRTUAL_PIN V0
#define SIM_DROP                V1
#define SETPOINT_DPM_VIRT_PIN   V2
#define DPM_SLIDER              V3
#define DROP_COUNT_VIRTUAL_PIN  V4
#define UPTIME_VIRTUAL_PIN      V5
#define RESTART_BTN             V6 // Should change this to a "BREAK" Button ... and Add official reset Button ESP.reset();
#define DPM_avg_VIRTUAL_PIN     V7
#define PAUSE_BTN               V8
#define SERVO_SLIDER            V9
#define DPM_VIRTUAL_PIN         V10
#define RE_TUNE_BTN             V11
#define DropBOB_DEBUG           V12
#define LCD_VIRTUAL             V13 // attach LCD to Virtual 13 complex mode
//! V14 available
//! V15 available
//! V16 available
//! V17 available
#define MODE_DROPDOWN           V18
#define MENU_DROPDOWN           V19
#define SERVO_UPDATE_SPEED_VPIN V20
#define CLEAR_SAVED_SETTINGS    V21

/////////////////////////////////////////
// Hardware Definitions                //
// ESP8266 Thing Dev Board (WRL-13711) //
/////////////////////////////////////////

#define ServoPIN                2    // (SDA) attaches the servo
#define photo_interuptor_PIN    14   // (SCL/SCLK) attaches the photo-interuptor
//! D7 available                     // (TX)
//! D8 available                     // (RX)

#define LED_PIN                 5    // (LED) attaches the LED (may be other pins on other boards
//! D0 available                     // (Used in RESET)
//! D4 available                     //
//! D13 available                    // (MOSI)
#define SleepPin                12   // (MISO) sleep pin. (grounding to sleep to keep consistent)
#define WakePin                 16   // (XPD) wake pin. (grounding required to wake)
//! A0 available                     // (ADC 10-bit 1V)
//! D15 available                    //

////////////////////////////////////
// Included libraries............ //
////////////////////////////////////

#include <Servo.h>  // for the servo
#include <stdlib.h> // Include Standard Library
#include <SimpleTimer.h> //https://github.com/jfturcot/SimpleTimer
#include <Wire.h>
#include <SPI.h>
#include <BlynkSimpleEsp8266.h>
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

////////////////////////////////////
// Variables .................... //
////////////////////////////////////

char blynk_token[33]; //"Token to be entered in browser";
SimpleTimer timer;

WidgetLCD thLCD(LCD_VIRTUAL); // LCD widget. Global intialization.
WiFiManager wifiManager; //WiFiManager. Global intialization.

//================================\/ PID \/=============
unsigned long lastTime = 0;
double errSum = 0, lastErr = 0, error = 0, dErr = 0;
double kp = 0.7, ki = 0.4 / 60000.0, kd = 0; //kp tunes to 1.3 alone
//====================general variables==================================

//VarSpeedServo myservo;  // create servo object to control a servo
Servo myservo;  // create servo object to control a servo

int raw = 1024; //sensor reads High (1024) when no drop
volatile long count = 0;
long lastDropCount = 0;
int numDrops = 0;
long delta = 0;
int state = HIGH; // HIGH is high signal, not broken light beam.
double lastDrop = millis();
float DPM = 0;
int first_drop = 1; // Avoid the first drop in servo settings
volatile int voltage = 5;
int button = 0;
int restart = 0;
int pause = 0;
double slide_time = 0;
bool led_gp5;
float A = 0;
float B = 0;
long uptime = 0;
int update_Sync = 0;
float Low_LED = millis();
float High_LED = millis();
float Servo_adjust = millis(); //servo adjustment time keeper
String botLine = "";
String topLine = "";
int Mode = 1; //Default mode is: Normal mode flag (other Modes are Agressive = 2 and Forced = 3)
double time_now = 0;
int close_increment = 1;
int open_increment = 1;
bool isFirstConnect = true;
int first_tune = 1;
bool shouldSaveConfig = false;
float seconds, minutes, hours, days;
unsigned long interrupt_time = millis();
int tuned = 0;
int debug = 0;

// Time until sleep (in seconds):
const int sleepTimeS = 0;

volatile unsigned long last_interrupt_time = 0;

// SERVO SETUP VALUES ======================================================
float set_DPM = 6;
int Servo_Val = 145; // starting servo value (0 is full open 180 is full close)
int servo_min = 0;
int servo_max = 180; //Servo won't go higher than 180, you can check with "myservo.read()"
int Servo_update_Speed = 500; // update every X milliseconds
int Servo_movements = 1;      // how much to update servo position by
float open_factor = 0.5;
int kick = 0; //This makes the servo kick to remouve play in system (***LOUDER)
int kick_delay = 0; // delay to wait after the kick before going to true value
float DPM_buffer = 1; // leave the servo alone if DPM withing the buffer (abs() + or - DPM_buffer)
float DPM_compare = DPM; //this lets the agressive mode use instantaneous DPM, and regular mode use DPM_avg
float open_bias = 1; // no open or close bias for the valve with 1x
float open_delay = 1.25; // tollerance to letting the opening of the valve wait untill DPM is 1.25x what it should be if drop dropped now
int open_to_drop = 130; // forced mode servo value open_to_drop (this is the default ... it get overwriten by Forced Mode)
int closed_to_stop = 180; // forced mode servo value close_to_stop (this is the default ... It get overwriten with Tune function)
int time_to_stay_closed = 200; //forced mode time_to_stay_open in milliseconds
float hold_value = 0;

// RUNNING AVERAGE VARIABLES =========================================
const int numReadings = 2;             // minimum 2
double readings[numReadings];      // the readings from the analog input (subtract 1 since it starts at zero)
double total = 0;                  // the running total
int index_n = 0;                  // the index_n of the current reading
int start_avg = 0;
// RUNNING AVERAGE VARIABLES =========================================
const int numReadings_avg = 5;             // minimum 2
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

void pause_requests() {
  while (pause == 1) {
    if (debug == 1) {
      Serial.println("pause_requests()");
    }
    //if (Blynk.connected()) {
      Blynk.run();
    //}
    thLCD.clear(); // Clear the LCD
    thLCD.print(0, 0, "Paused ..."); // Print top line
    thLCD.print(0, 1, "Valve Closed"); // Print bottom line

    int lapse = millis() - slide_time;

    if (slide_time == 0) {                 // If the pause comes from the app, close the servo to prevent drops (true pause)
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(15);
      myservo.write(servo_max);
      Blynk.virtualWrite(V0, servo_max);
    }

    if (lapse > 1000 && slide_time > 0) {  // if the pause comes from a parameter change
      pause = 0;
      slide_time = 0;
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(15);
      myservo.write(Servo_Val);
      Blynk.virtualWrite(V0, Servo_Val);
      Blynk.virtualWrite(V2, set_DPM);
    }

    if (pause == 0) {                       // exit clause to put the valve back after pausing
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(15);
      myservo.write(Servo_Val);
      Blynk.virtualWrite(V0, myservo.read());
    }
  }
}

BLYNK_WRITE(SIM_DROP) { //pushbutton in Blynk app that simulates a drop of water (for testing) - INPUT
  voltage = 0;
  button = 1;
}

BLYNK_WRITE(DropBOB_DEBUG) {
  if(param.asInt() == 1){
    debug = 1;
  }
  else if(param.asInt() == 0){
    debug = 0;
  }
}

BLYNK_WRITE(DPM_SLIDER) { //"Blynk slider for DPM setting" - INPUT
  pause = 1;
  slide_time = millis();
  set_DPM = param[0].asInt();
}

BLYNK_WRITE(RESTART_BTN) { //is "Blynk restart button" - INPUT
  restart = param.asInt();
}

BLYNK_WRITE(PAUSE_BTN) { //is "Blynk Pause button" - INPUT
  pause = param.asInt();
}

BLYNK_WRITE(SERVO_SLIDER) { //is "Blynk Manual servo control" - INPUT
  pause = 1;
  slide_time = millis();
  Servo_Val = param.asInt();
}

BLYNK_WRITE(CLEAR_SAVED_SETTINGS) { //is "Blynk's clear ALL Settings config Pin" - INPUT
  wifiManager.resetSettings();  //clear settings
  SPIFFS.format();              //clear File System
  Serial.println("settings cleared ... RESETTING");
  delay(3000);
  ESP.reset();
  delay(5000);
}

void open_up() {
  if ((millis() - lastDrop) > (open_delay * (60000.0 / set_DPM)) && Mode != 3) { //don't use this on forced Mode
    if (debug == 1) {
      Serial.println("open_up()");
    }

    uint32_t temp_delta = millis() - lastDrop;
    float temp_DPM = 60000.0 / temp_delta;

    Servo_Val = Servo_Val - open_factor * (set_DPM - temp_DPM); // if the servo closed and no drops are comming for too long open it up a little.

    if (Servo_Val < servo_min) {
      Servo_Val = servo_min;
    }

    myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
    delay(15);
    myservo.write(Servo_Val);
    //Serial.print("open_up() - Servo value:");Serial.println(Servo_Val);
    if ((millis() - uptime) > 1000) { //update tuning vars ON BLYNK only once per second to save time
      uptime = millis();
      Blynk.virtualWrite(V0, myservo.read());
    }
  }
}

void tune() {
  if (debug == 1) {
    Serial.println("tune()");
  }
  tuned = 0;
  for (/*nothing needed here*/; Servo_Val < servo_max; Servo_Val++) {

    myservo.attach(ServoPIN);
    delay(15);
    myservo.write(Servo_Val);
    add = 0;

    for (int run_tune = 0; run_tune < tuning_drops; run_tune++) {

      while (voltage > 0) {

        drop_no_interupt();

        //if (Blynk.connected()) {
          Blynk.run();
        //}
        timer.run();

        if (first_tune == 1) {
          Servo_Val -= 1;
          myservo.attach(ServoPIN);
          delay(150); //Keep this one slow otherwise it will open too quickly and you have too many drops
          myservo.write(Servo_Val);
          open_to_drop = Servo_Val - 10;
        }

        pause_requests();

        if (millis() - High_LED > 500) { // add LED indicator for tuning
          digitalWrite(LED_PIN, LOW);
          Low_LED = millis();
          High_LED = INFINITY;
        }
        if (millis() - Low_LED > 500) { // add LED indicator for tuning
          digitalWrite(LED_PIN, HIGH);
          High_LED = millis();
          Low_LED = INFINITY;
        }

      }

      voltage = 5.0;
      first_tune = 0;
      delta = millis() - lastDrop; //get the difference in time between each drop
      lastDrop = millis(); // remember time of last drop to prevent bouncing.
      numDrops = count - lastDropCount;
      lastDropCount = count;
      DPM = 60000.0 / (delta / numDrops); // get drop per min

      add = add + DPM;
      DPM_avg = add / (run_tune + 1);

      if (DPM < 1000) {
        Serial.print(delta); Serial.print("\t"); Serial.print("ms");
        Serial.print("\t");
        Serial.print(count);
        Serial.print("\t");
        Serial.print(DPM); Serial.print("\t"); Serial.print("DPM");
        Serial.print("\t");
        Serial.print(set_DPM); Serial.print("\t"); Serial.print("Setpoint");
        Serial.print("\t");
        Serial.print(myservo.read()); Serial.print("\t"); Serial.println("Servo_val");

        if ((millis() - uptime) > 1000) { //update tuning vars ON BLYNK only once per second (to save time
          uptime = millis();

          Blynk.virtualWrite(DPM_VIRTUAL_PIN, DPM);
          Blynk.virtualWrite(DPM_avg_VIRTUAL_PIN, DPM_avg);
          Blynk.virtualWrite(DROP_COUNT_VIRTUAL_PIN, count);
          Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read());
          Blynk.virtualWrite(SETPOINT_DPM_VIRT_PIN, set_DPM);
          thLCD.print(0, 0, "Tuning Mode     "); // Print top line
          uptime = millis();
          botLine = "";
          seconds = (float)millis() / 1000;
          minutes = seconds / 60;
          hours = minutes / 60;
          days = hours / 24;
          seconds = (int)seconds % 60;
          minutes = (int)minutes % 60;
          hours = (int)hours % 24;
          // Construct a string indicating run time
          if (days < 10) botLine += "0"; // Add the leading 0
          botLine += String((int)days) + "-";
          if (hours < 10) botLine += "0"; // Add the leading 0
          botLine += String((int)hours) + ":";
          if (minutes < 10) botLine += "0"; // Add the leading 0
          botLine += String((int)minutes) + ":";
          if (seconds < 10) botLine += "0"; // Add the leading 0
          botLine += String((int)seconds);
          botLine += "     ";
          thLCD.print(0, 1, botLine); // Print bottom line
        }
      }
    }

    DPM_tune_avg = add / tuning_drops;

    if (DPM_tune_avg < set_DPM) {
      closed_to_stop = Servo_Val + 10;
      tuned = 1;
      if (Servo_Val < servo_min) Servo_Val = servo_min;
      if (Servo_Val > servo_max) Servo_Val = servo_max;
      break;
    }
    if (restart == 1) {
      restart = 0;
      break;
    }
  }
}

BLYNK_WRITE(RE_TUNE_BTN) { //pushbutton in Blynk app for calling up the tune function below - INPUT
  if (param.asInt() == 1) {
    tune();
  }
}

BLYNK_WRITE(MODE_DROPDOWN) { //Dropdown selection of MODE - INPUT
  if (param.asInt() == 1) {
    Mode = 1; //Normal mode flag
    kp = 0.7, ki = 0.4 / 60000.0, kd = 0; //std PI(D)
    servo_min = 15; servo_max = 195; //std constraints
    Servo_update_Speed = 500; //std movement
    open_factor = 0.5; //std open factor
    kick = 0; // no kick
    kick_delay = 0; // no kick delay
    DPM_buffer = 1; // 1-DPM buffer
    open_bias = 1; // no open bias
    open_delay = 1.25; // back to normal tollerance
    topLine = "MODE: Normal :) ";
    Servo_movements = 1;      // how much to update servo position by
  }
  if (param.asInt() == 2) {
    Mode = 2; //Agressive mode flag
    kp = 0.6, ki = 0.3 / 60000.0, kd = 700; //derivative term predicts output bu more jitter
    //servo_min = 120; servo_max = 180; //tighter constraints
    Servo_update_Speed = 1000; //reduce the movement speed (mostly when opening up)
    open_factor = 0.25; //let the system equalize longer by halfing the opening up factor.
    kick = 30; // add 40 degrees of kick
    kick_delay = 200; // wait 200 milliseconds before returning
    DPM_buffer = 0; // no buffer, always update
    open_bias = 1; // open half as fast as you close
    open_delay = 1.5; // add some opening tollerance
    topLine = "MODE: AGRESSIVE!";
    Servo_movements = 1;      // how much to update servo position by
  }
  if (param.asInt() == 3) {
    Mode = 3; //Forced mode flag
    //open_to_drop = 150; // servo value --- Let tune decide this one
    //closed_to_stop = 180; //servo value --- Let tune decide this one
    time_to_stay_closed = 100; //in milliseconds
    open_delay = 5; //let it go up to 5x off course before correcting
    topLine = "MODE: FoRcEd ;] ";
    Servo_movements = 1;      // how much to update servo position by
    //Servo_update_Speed = 1500; // **** This becomes the MAIN variable under Forced Mode --- Let tune decide this one
    DPM_buffer = 0.25;
  }
}

BLYNK_WRITE(MENU_DROPDOWN) { //Menu dropdown
  if (param.asInt() == 1) { // BREAK/RESTART
    restart = 1;
  }
  if (param.asInt() == 2) { //PAUSE
    pause = 1;
    delay(15);
    pause_requests();
  }
  if (param.asInt() == 3) { //RE-TUNE
    thLCD.clear(); // Clear the LCD
    thLCD.print(0, 0, "Tuning Mode"); // Print top line
    thLCD.print(0, 1, "Please Wait"); // Print bottom line
    tune();
  }
  if (param.asInt() == 4) {//SIMULATE DROP
    voltage = 0;
  }
  if (param.asInt() == 5) { //CLEAR ALL SAVED SETTINGS
    wifiManager.resetSettings();  //clear settings
    SPIFFS.format();              //clear File System
    Serial.println("settings cleared ... RESETTING");
    delay(3000);
    ESP.reset();
    delay(5000);
  }

}

void print_stats() {
  if (debug == 1) {
    Serial.println("print_stats()");
  }
  if (DPM < 1000) {
    Serial.print(delta); Serial.print("\t"); Serial.print("ms");
    Serial.print("\t");
    Serial.print(count);
    Serial.print("\t");
    Serial.print(DPM_avg); Serial.print("\t"); Serial.print("DPM");
    Serial.print("\t");
    Serial.print(set_DPM); Serial.print("\t"); Serial.print("Setpoint");
    Serial.print("\t");
    if ( (Mode == 1) | (Mode == 2) ) {
      Serial.print(myservo.read());
      Serial.print("\t");
      Serial.print("Servo_val");
    }
    if (Mode == 3) {
      Serial.print(Servo_update_Speed);
      Serial.print("\t");
      Serial.print("Servo_update_Speed");
    }
    Serial.print("\t");
    Serial.print(kp * error); Serial.print("\t"); Serial.print("error");
    Serial.print("\t");
    Serial.print(ki * errSum); Serial.print("\t"); Serial.print("errSum");
    Serial.print("\t");
    Serial.print(kd * dErr); Serial.print("\t"); Serial.println("dErr"); //*/

    if ((millis() - uptime) > 500) { //update tuning vars ON BLYNK only once per second (to save time
      uptime = millis();

      Blynk.virtualWrite(DPM_VIRTUAL_PIN, DPM);
      yield();
      Blynk.virtualWrite(DPM_avg_VIRTUAL_PIN, DPM_avg);
      yield();
      Blynk.virtualWrite(DROP_COUNT_VIRTUAL_PIN, count);
      yield();
      Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read());
      yield();
      Blynk.virtualWrite(SETPOINT_DPM_VIRT_PIN, set_DPM);
      yield();
      Blynk.virtualWrite(SERVO_UPDATE_SPEED_VPIN, Servo_update_Speed);

      if (debug == 1) {
        Serial.print("print_stats(");
        Serial.print("time it took to upload: ");
        Serial.print(millis() - uptime);
        Serial.print("-ms");
        Serial.println(")");
      }
    }
  }
}

void measure_DPM() {
  if (debug == 1) {
    Serial.println("measure_DPM()");
  }
  state = LOW;
  button = 0;

  digitalWrite(LED_PIN, LOW); // for some odd reason ... LED PIN to "LOW" means "on"

  delta = millis() - lastDrop; //get the difference in time between each drop
  lastDrop = millis(); // remember time of last drop to prevent bouncing.
  numDrops = count - lastDropCount;
  lastDropCount = count;
  DPM = 60000.0 / (delta / numDrops); // get drop per min

  //====================================================================== 3 reading smoothing
  total = total - readings[index_n];  // subtract the last reading:
  readings[index_n] = DPM;
  total = total + readings[index_n];   // add the reading to the total:
  index_n = index_n + 1;

  if (index_n >= numReadings) {
    start_avg = 1;
    index_n = 0;
  }
  if (start_avg == 1) {
    DPM = total / numReadings;
  }   // drop the instantaneou DPM and use the running ave after all initial readings taken
  else {
    DPM = total / (index_n + 1);
  }

  //======================================================================== 10 reading average
  total_avg = total_avg - readings_avg[index_n_avg];  // subtract the last reading:
  readings_avg[index_n_avg] = DPM;
  total_avg = total_avg + readings_avg[index_n_avg];   // add the reading to the total:
  index_n_avg = index_n_avg + 1;

  if (index_n_avg >= numReadings_avg) {
    start_avg_avg = 1;
    index_n_avg = 0;
  }

  if (start_avg_avg == 1) {
    DPM_avg = total_avg / numReadings_avg;
  }   // drop the instantaneou DPM and use the running ave after all initial readings taken
  else {
    DPM_avg = total_avg / (index_n_avg + 1);
  }
}

void Servo_angle_method() { //(NORMAL & AGRESSIVE MODE) The servo angle is adjusted after every drop to maintain set_DPM
  // This method sets the servo position according to the latest drop DPM (more open or more closed
  // This method is very lagging as it uses old data, including the last 10 point average to set the future servo position

  if (voltage == 0 && state == HIGH) {
    if (debug == 1) {
      Serial.println("Servo_angle_method()");
    }

    state = LOW;
    measure_DPM();
    voltage = 5.0;

    /*Compute all the working error variables*/
    error = set_DPM - DPM;
    errSum += (error * delta);
    dErr = (error - lastErr) / delta;

    if ( DPM_avg - set_DPM > DPM_buffer && first_drop == 0) { //to add forward or reverse bias Uncomment the elseif loop and the DPM condition
      Servo_Val = Servo_Val - ( kp * error + ki * errSum + kd * dErr); // Set servo change depending on how far away from set_DPM you are at
      if (Servo_Val < servo_min) Servo_Val = servo_min;
      if (Servo_Val > servo_max) Servo_Val = servo_max;

      if ( kick > 0) {
        myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
        delay(15);
        myservo.write(myservo.read() - kick);
        delay(kick_delay);
        myservo.write(Servo_Val);
      }

    }
    else if ( DPM_avg - set_DPM < DPM_buffer && first_drop == 0) { // This is to give a Forward or Reverse BIAS (commented out)
      Servo_Val = Servo_Val - open_bias * (kp * error + ki * errSum + kd * dErr); // Set servo change depending on how far away from set_DPM you are at
      if (Servo_Val < servo_min) Servo_Val = servo_min;
      if (Servo_Val > servo_max) Servo_Val = servo_max;

      if ( kick > 0) {
        myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
        delay(15);
        myservo.write(myservo.read() + kick);
        delay(kick_delay);
        myservo.write(Servo_Val);
      }

    }

    print_stats();
    first_drop = 0;
  }

  if (voltage == 0 && state == LOW ) { // end of pulse, now we may expect a new one, DEBOUNCE
    if (millis() - lastDrop > 50) state = HIGH; // only go back to state high if some time has passed.
  }

  lastErr = error;
}

void Speed_to_open_method() { //(FORCED MODE) The speed at which the servo is opened to let out a drop is adjusted after every drop to maintain set_DPM
  //The difference with this method is that it always opens the servo. So there is no possibility to "stall" like the Servo angle method
  //which waits for a drop then potentially sets the angle to a "bad" angle and can wait indefinitely, until the openup kicks in
  //this should be more acurate but also cause more servo noise and it will constantly be running and possibly closing quickly
  //after each drop (TBD)*/

  if (voltage == 0) { //Drop
    if (debug == 1) {
      Serial.println("Speed_to_open_method()");
    }
    voltage = 5.0;
    measure_DPM();
    open_to_drop = Servo_Val; // record the opening angle that caused a drop
    Servo_Val = closed_to_stop; // Close the servo
    if (Servo_Val < servo_min) Servo_Val = servo_min;
    if (Servo_Val > servo_max) Servo_Val = servo_max;

    //Serial.print("Servo val: ");Serial.println(Servo_Val);

    while (myservo.read() < servo_max - close_increment && voltage == 5) {
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(15);
      if (myservo.read() + close_increment > servo_max) close_increment = 0;
      myservo.write(myservo.read() + close_increment);
      if (debug == 1) {
        Serial.println("Speed_to_open_method("); Serial.print("closing valve: "); Serial.println(myservo.read());
      }
    }
    close_increment = 3;

    /*Compute all the working error variables*/
    error = set_DPM - DPM;
    errSum += (error * delta);
    dErr = (error - lastErr) / delta;

    if (abs(DPM_avg - set_DPM) > DPM_buffer && first_drop == 0) { //to add forward or reverse bias Uncomment the elseif loop and the DPM condition
      Servo_update_Speed -= 10 * ( kp * error + ki * errSum + kd * dErr); // Set servo change depending on how far away from set_DPM you are at
      if (Servo_update_Speed < 0) {
        Servo_update_Speed = 0; //negatives don't make sence
      }
      if (Servo_update_Speed > 2000) {
        Servo_update_Speed = 2000; // prevent from going too slow
      }
    }

    print_stats();
    first_drop = 0;
  }

  if (voltage == 5 && state == LOW ) { // end of pulse, now we may expect a new one, DEBOUNCE
    if (millis() - lastDrop > 50) state = HIGH; // only go back to state high if some time has passed.
  }

  if ( (millis() - lastDrop) > time_to_stay_closed && myservo.read() > (open_to_drop + open_increment) && voltage == 5) {
    Servo_Val = open_to_drop;
  }

  if (myservo.read() == Servo_Val) { //has reached open, but still no drop
    Servo_Val -= 1;
  }
}

void run_blynk() {
  if (debug == 1) {
    Serial.println("run_blynk()");
  }
  yield();
  //if (Blynk.connected()) {
    Blynk.run();
  //}
}

BLYNK_CONNECTED() {
  if (isFirstConnect) {
    if (debug == 1) {
      Serial.println("BLYNK_CONNECTED()");
    }
    //Console.println("*** Sync All ? ***");
    //Blynk.syncAll();
    Blynk.syncVirtual(MODE_DROPDOWN); //sync Mode from last setting on APP
    Blynk.syncVirtual(DPM_SLIDER); //sync setDPM from last setting on APP
    Blynk.syncVirtual(DropBOB_DEBUG); //sync Debug Button from last setting on APP
    isFirstConnect = false;
  }
}

void reconnectBlynk() {
  if (!Blynk.connected()) {
    if (debug == 1) {
      Serial.println("reconnectBlynk()");
    }
    if (Blynk.connect()) {
      Serial.println("Reconnected");
    } else {
      Serial.println("Not reconnected");
    }
  }
}

void saveConfigCallback () {
  if (debug == 1) {
    Serial.println("saveConfigCallback ()");
  }
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setup() {
  Serial.begin(19200);
  delay(10);
  if (debug == 1) {
    Serial.println("setup()");
  }

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          strcpy(blynk_token, json["blynk_token"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_blynk_token("Blynk", "blynk token", blynk_token, 33);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //wifiManager.resetSettings();  //leave commented to save settings after connected
  //SPIFFS.format();              //clean FS, for testing only

  wifiManager.addParameter(&custom_blynk_token);

  if (!wifiManager.autoConnect("Drop-BOB")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  strcpy(blynk_token, custom_blynk_token.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["blynk_token"] = blynk_token;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save

    Serial.println("local ip");
    Serial.println(WiFi.localIP());
  }

  Blynk.config(blynk_token);

  while ( abs(myservo.read() - Servo_Val) < Servo_movements + 1) {

    if (myservo.read() > Servo_Val ) {
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(15);
      myservo.write(myservo.read() - Servo_movements);
    }
    else if (myservo.read() < Servo_Val) {
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(15);
      myservo.write(myservo.read() + Servo_movements);
    }

    delay(50);

  }

  topLine = "MODE: Normal :) "; //startup in Normal mode. Show this on LCD

  Serial.println("DropBOB v2.5");
  Serial.println();

  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0; // Initialize the array

  pinMode(photo_interuptor_PIN, INPUT);
  //Not going to be using the interupt for Drop counting ... it causes too many issues with Blynk connectivity
  //Maybe for a future update ...
  //attachInterrupt(digitalPinToInterrupt(photo_interuptor_PIN), drop, FALLING); //possibly also Mode: LOW(no good),FALLING(best),CHANGE(too many),RISING(no)
  pinMode(ServoPIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SleepPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SleepPin), sleep_switch, FALLING); //Set Sleep monitor pin as INTERUPT... HIGH by default
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

  //Blynk.tweet("Brewing a fresh pot of Cold Drip Coffee with my Drop-BOB v1.0: Check it out at www.bobbobblogs.blogspot.com");

  timer.setInterval(500L, run_blynk); 
  timer.setInterval(5000L, open_up); // open up the servo every 5 seconds if no drops come ... not a Blynk update
  //timer.setInterval(60000L, reconnectBlynk); //only atempt reconnection once per minute

  tune(); //start by tuning the system
}//================================================================================END SETUP========================

void loop() {
  if (debug == 1) {
    Serial.println("loop()");
  }
  pause_requests(); //accept pause requests
  //if (Blynk.connected()) {
    Blynk.run();
  //}
  timer.run(); // Initiates SimpleTimer
  digitalWrite(LED_PIN, HIGH); // for some odd reason ... LED PIN to "HIGH" means "off"

  //--->Remouve this for Interupt driven sensing
  drop_no_interupt();

  if ((millis() - uptime) > 1000) { //update uptime LCD & uptime variable
    if (debug == 1) {
      Serial.print("loop(");
      Serial.print("uptime: ");
      Serial.print(millis());
      Serial.print("-s");
      Serial.println(")");
    }
    uptime = millis();
    botLine = "";
    seconds = (float)millis() / 1000;
    minutes = seconds / 60;
    hours = minutes / 60;
    days = hours / 24;
    seconds = (int)seconds % 60;
    minutes = (int)minutes % 60;
    hours = (int)hours % 24;
    // Construct a string indicating run time
    if (days < 10) botLine += "0"; // Add the leading 0
    botLine += String((int)days) + "-";
    if (hours < 10) botLine += "0"; // Add the leading 0
    botLine += String((int)hours) + ":";
    if (minutes < 10) botLine += "0"; // Add the leading 0
    botLine += String((int)minutes) + ":";
    if (seconds < 10) botLine += "0"; // Add the leading 0
    botLine += String((int)seconds);
    botLine += "     ";
    thLCD.print(0, 0, topLine); // Print top line
    thLCD.print(0, 1, botLine); // Print bottom line
    if (debug == 1) {
      Serial.print("loop(");
      Serial.print("time it took to upload: ");
      Serial.print(millis() - uptime);
      Serial.print("-ms");
      Serial.println(")");
    }
  }

  if ( (Mode == 1) | (Mode == 2) ) { //Only if Mode = Normal (1) or Agressive (2) do this
    Servo_angle_method();
  }

  if (Mode == 3) { //Only if Mode = Forced (3) do this
    Speed_to_open_method();
  }

  if ( (millis() - Servo_adjust) > Servo_update_Speed) {
    Servo_adjust = millis();
    if (myservo.read() < Servo_Val) {
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(15);
      myservo.write(myservo.read() + Servo_movements);

      if (debug == 1) {
        Serial.print("loop(");
        Serial.print("Smooth-closing valve: ");
        Serial.print(myservo.read());
        Serial.println(")");
      }
    }

    if (myservo.read() > Servo_Val) {
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(25);
      myservo.write(myservo.read() - Servo_movements);

      if (debug == 1) {
        Serial.print("loop(");
        Serial.print("Smooth-opening valve: ");
        Serial.print(myservo.read());
        Serial.println(")");
      }
    }
  }

  //=============if open to the max for 6 min and nothing comes out ... your done!
  if ( (millis() - lastDrop) > 360000 && Servo_Val < (servo_min + 2)) {
    Serial.println(); Serial.println("FINISHED!!!");
    //Blynk.tweet("Brew DONE!!: www.bobbobblogs.blogspot.com");
    myservo.detach();
    thLCD.clear(); // Clear the LCD
    thLCD.print(0, 0, "Finished in:"); // Print top line
    thLCD.print(0, 1, botLine); // Print bottom line
    while (restart == 0)
    {
      //if (Blynk.connected()) {
        Blynk.run();
      //}
      pause_requests(); //accept pause requests

      if (!digitalRead(SleepPin)) {
        myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
        delay(15);
        myservo.write(servo_max);
        Serial.println("...Going to Sleep...");
        delay(1000);
        ESP.deepSleep(0, WAKE_RF_DEFAULT); // Sleep forever, until Pin#16 is un-grounded (button un-pushed)
      }

    } // when finished do nothing but listen for Blynk app
    myservo.attach(ServoPIN);
    restart = 0;
    lastDrop = millis();
    Servo_Val = 140;
    Serial.println(); Serial.print("Restarting"); delay(700); Serial.print(" ."); delay(700); Serial.print("."); delay(700); Serial.print("."); delay(10); Serial.println(" waiting for drops"); Serial.println();
  }
}

/* this is the interrupt driven drop counting ... It causes problems
  void drop() {
  // If interrupts come faster than X-ms, assume it's a bounce and ignore
  Serial.println("8");
  if (millis() - last_interrupt_time > 70)
  {
    voltage = 0;
    count++;
    last_interrupt_time = millis();
  }
  }
*/

void drop_no_interupt() {
  raw = analogRead(photo_interuptor_PIN); // read the drop sensor
  if (raw == 0 && (millis() - last_interrupt_time) > 70) {
    voltage = 5.0 * raw / 1023; // convert it to voltage
    if (debug == 1) {
      Serial.println("drop_no_interupt() ... ... ... ");
    }
    count++;
    last_interrupt_time = millis();
  }
}

void sleep_switch() { //Interupt
  if (debug == 1) {
    Serial.println("sleep_switch()");
  }
  
  myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
  delay(25);
  myservo.write(servo_max);
  
  Serial.println("...Going to Sleep...");
  thLCD.clear(); // Clear the LCD
  thLCD.print(0, 0, "Going to Sleep"); // Print top line
  thLCD.print(0, 1, "zzzzzzzzzzzzzZ"); // Print bottom line
  delay(1000);
  ESP.deepSleep(0, WAKE_RF_DEFAULT); // Sleep forever, until Pin#16 is un-grounded (button un-pushed)
}

