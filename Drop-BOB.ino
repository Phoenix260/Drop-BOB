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
           - On the Blynk App make sure all widgets are "PUSH". Setting manual intervals slows down this code. If you are getting many APP disconnects (CHECK)
           - If a Virtual pin does not exist in the APP on your phone, the data will not be logged by BLYNK
           -
************************************************************************************************/

////////////////////////////////////
// Included libraries............ //
////////////////////////////////////

#include <FS.h>                   //Include File System (to save parameter data). this needs to be first
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
#include <math.h>                 //used for the complex math fuctions like LOG(X) for the PID

////////////////////////////////////
// Blynk Virtual Variable Mapping //
////////////////////////////////////

#define SERVO_ANGLE_VIRTUAL_PIN V0
#define SIM_DROP                V1
#define SETPOINT_DPM_VIRT_PIN   V2
#define DPM_SLIDER              V3
#define DROP_COUNT_VIRTUAL_PIN  V4
#define UPTIME_VIRTUAL_PIN      V5
#define RESTART_BTN             V6 // Should change this to a "BREAK" Button ... and Add official reset Button ESP.restart();
#define DPM_avg_VIRTUAL_PIN     V7
#define PAUSE_BTN               V8
#define SERVO_SLIDER            V9
#define DPM_VIRTUAL_PIN         V10
#define RE_TUNE_BTN             V11
#define DropBOB_DEBUG           V12
#define LCD_VIRTUAL             V13 // attach LCD to Virtual 13 complex mode
#define DPM_INSTANT_PIN         V14
#define TERMINAL_PIN            V15
#define SERVO_MIN_PIN           V16
#define SERVO_MAX_PIN           V17
#define MODE_DROPDOWN           V18
#define MENU_DROPDOWN           V19
#define SERVO_UPDATE_SPEED_VPIN V20
#define CLEAR_SAVED_SETTINGS    V21
#define IS_DONE_VPIN            V22
#define BOTTLE_SELECT_PIN       V23
#define ALLTIME_DROP_AVG_PIN    V24

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
// Wiget & Object declarations    //
////////////////////////////////////

WidgetLCD       thLCD(LCD_VIRTUAL);       // LCD widget. Global intialization.
WiFiManager     wifiManager;              //WiFiManager. Global intialization.
WidgetTerminal  terminal(TERMINAL_PIN);   //Terminal widger. Global initialization
Servo           myservo;                  // create servo object to control a servo
SimpleTimer     timer;
#define BLYNK_PRINT Serial

////////////////////////////////////
// Variables .................... //
////////////////////////////////////

// User values ==========================================================
char blynk_token[33]; //"Token to be entered in browser";

// Default OPTIONS ===============================================================
int debug = 0;          //Default mode: 0 none, 1 regular, 2 verbose
int isInterrupt = 0;    //set to 1 if wanting to use interrupts (NOT STABLE. code may crash)
int Mode = 2;           //Default mode: Normal=1, Agressive = 2, Forced = 3
int bottle_size = 300;  //Default mode: small=300, medium=500, large=1000

//================================\/ PID \/=============
unsigned long lastTime = 0;
double errSum = 0, lastErr = 0, error = 0, dErr = 0;
double kp = 0.7, ki = 0.4 / 60000.0, kd = 0; //kp tunes to 1.3 alone

//====================general variables==================================
int raw = 1024; //sensor reads High (1024) when no drop
volatile long count = 0;
long lastDropCount = 0;
int numDrops = 0;
long delta = 0;
int state = HIGH; // HIGH is high signal, not broken light beam.
double lastDrop = millis();
float DPM = 0;
float DPM_Instant = 0;
int first_drop = 1; // Avoid the first drop in servo settings
volatile int voltage = 5;
int button = 0;
int restart = 0;
int pause = 0;
bool led_gp5;
float A = 0;
float B = 0;
long uptime = 0;
int update_Sync = 0;
float Low_LED = millis();
float High_LED = millis();
float Servo_adjust = millis(); //servo adjustment time keeper
double time_now = 0;
bool isFirstConnect = true;
int first_tune = 1;
bool shouldSaveConfig = false;
float seconds, minutes, hours, days;
unsigned long interrupt_time = millis();
int tuned = 0;
int is_servo_min_tuned = 0;
long servo_min_tune_time = millis();
int is_close_success = 0;
int close_factor = 15;
int isDONE = 0;
int isNearDrop = 0;
int do_once = 0;
float ml_per_drop = 0.05; // should be stored in memory to increase precision per run
int total_time = 0;
int record_count = 0;

// Time until sleep (in seconds):
const int sleepTimeS = 0;

volatile unsigned long last_interrupt_time = 0;
long last_interrupt_drop = 0;
long sync_time = 0;

// SERVO SETUP VALUES ======================================================
float set_DPM = 6;
float Servo_Val = 100; // starting servo value (0 is full open 180 is full close)
int servo_min = 0;
int servo_max = 180; //Servo won't go higher than 180, you can check with "myservo.read()"
int Servo_update_Speed = 500; // update every X milliseconds
int Servo_movements = 1;      // how much to update servo position by
float open_factor = 0.5;
float kick_close = 0; //This makes the servo kick_close to remouve play in system (***LOUDER)
float kick_open = 0;
int kick_close_delay = 0; // delay to wait after the kick_close before going to true value
float DPM_buffer = 1; // leave the servo alone if DPM withing the buffer (abs() + or - DPM_buffer)
float DPM_compare = DPM; //this lets the agressive mode use instantaneous DPM, and regular mode use DPM_avg
float open_bias = 1; // no open or close bias for the valve with 1x
float open_delay = 1.25; // tollerance to letting the opening of the valve wait untill DPM is 1.25x what it should be if drop dropped now
int open_to_drop = 100; // forced mode servo value open_to_drop (this is the default ... it get overwriten by Forced Mode)
int closed_to_stop = 180; // forced mode servo value close_to_stop (this is the default ... It get overwriten with Tune function)
int time_to_stay_closed = 200; //forced mode time_to_stay_open in milliseconds
float hold_value = 0;
int sync_once_aft_tune = 0;
int kick_close_tuned = 0;
int kick_open_tuned = 0;
int temp = 0;
float alltimeAVG_DPM = 0;

// RUNNING AVERAGE SHORTER VARIABLES =========================================
const int numReadings = 3;             // minimum 2
double readings[numReadings];      // the readings from the analog input (subtract 1 since it starts at zero)
double total = 0;                  // the running total
int index_n = 0;                  // the index_n of the current reading
int start_avg = 0;
// RUNNING AVERAGE LONGER VARIABLES =========================================
const int numReadings_avg = 10;             // minimum 2
double readings_avg[numReadings_avg];      // the readings from the analog input (subtract 1 since it starts at zero)
double total_avg = 0;                  // the running total
int index_n_avg = 0;                  // the index_n of the current reading
int start_avg_avg = 0;
float DPM_avg = 0;

// TUNING VARIABLES =================================================
int tuning_drops = 3; // how many drops before closing the servo a little in the tuning mode
int max_DPM_to_tune = 30; //how high to get DPM to consider full open

// LCD Text =========================================================
// note that the LCD has 2 lines and 16 characters per line max use space wisely
String mode2 = "Standard"; //8 char show at LCD position 10
String mode1 = "Passive "; //8 char show at LCD position 10
String mode3 = "FoRcEd  "; //8 char show at LCD position 10

String eta = "eta:"; //4 char

String DropNEAR = ".drop"; //character indicating that drop is near ... so you know why display not updating

String botLine = ""; //empty strings used to compose the LCD - size 17 since you need space for the "\0" null char
String topLine = ""; //empty strings used to compose the LCD - size 17 since you need space for the "\0" null char


void pause_requests() {
  while (pause == 1) {
    if (debug >= 1) {
      Serial.println("pause_requests()");
    }

    //if (Blynk.connected()) {
    Blynk.run();
    //}

    thLCD.clear(); // Clear the LCD
    thLCD.print(0, 0, "Paused ..."); // Print top line
    thLCD.print(0, 1, "Valve Closed"); // Print bottom line

    if (myservo.read() != servo_max) {      //Close the valve when paused (True pause)
      myservo.attach(ServoPIN);
      delay(15);
      myservo.write(servo_max);
      Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, servo_max);
    }

    if (pause == 0) {                       // exit clause to put the valve back after pausing
      myservo.attach(ServoPIN);
      delay(15);
      myservo.write(Servo_Val);
      Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read());
    }
  }
}

BLYNK_WRITE(TERMINAL_PIN) { //this is the terminal input pin catching all the text
  if (String("clean slate") == param.asStr()) {
    wifiManager.resetSettings();  //clear settings
    SPIFFS.format();              //clear File System
    terminal.println("settings cleared ... RESETTING");
    delay(3000);
    ESP.restart();
    delay(5000);
  }
  if (String("clear terminal") == param.asStr()) {
    terminal.println("not yet supported");
  }
  terminal.flush(); //good practice
}

BLYNK_WRITE(SIM_DROP) { //pushbutton in Blynk app that simulates a drop of water (for testing) - INPUT
  voltage = 0;
  button = 1;
}

BLYNK_WRITE(DropBOB_DEBUG) {
  debug = param.asInt() - 1;
}

BLYNK_WRITE(DPM_SLIDER) { //"Blynk slider for DPM setting" - INPUT
  set_DPM = param[0].asInt();
  total_time = bottle_size / ml_per_drop / set_DPM / 60; //in hours
}

BLYNK_WRITE(RESTART_BTN) { //is "Blynk restart button" - INPUT
  if (param.asInt() == 1) {
    Serial.println("... RESETTING ...");
    delay(3000);
    ESP.restart();
    delay(5000);
  }
}

BLYNK_WRITE(PAUSE_BTN) { //is "Blynk Pause button" - INPUT
  pause = param.asInt();
}

BLYNK_WRITE(SERVO_SLIDER) { //is "Blynk Manual servo control" - INPUT
  Servo_Val = param.asInt();
  if (Servo_Val < servo_min) Servo_Val = servo_min;
  if (Servo_Val > servo_max) Servo_Val = servo_max;
  myservo.attach(ServoPIN);
  delay(15);
  myservo.write(Servo_Val);
  Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read());
  int slider_time = millis();
  while (millis() - slider_time < 5000) {
    pause_requests(); //accept pause requests
    //if (Blynk.connected()) {
    Blynk.run();
    //}
  }
}

BLYNK_WRITE(CLEAR_SAVED_SETTINGS) { //is "Blynk's clear ALL Settings config Pin" - INPUT
  wifiManager.resetSettings();  //clear settings
  SPIFFS.format();              //clear File System
  Serial.println("settings cleared ... RESETTING");
  delay(3000);
  ESP.restart();
  delay(5000);
}

void open_up() {
  if ((millis() - lastDrop) > (open_delay * (60000.0 / set_DPM)) && (Mode != 3)) { //don't use this on forced Mode
    if (debug >= 1) {
      Serial.println("open_up()");
    }

    uint32_t temp_delta = millis() - lastDrop;
    float temp_DPM = 60000.0 / temp_delta;

    Servo_Val = Servo_Val - open_factor * (set_DPM - temp_DPM); // if the servo closed and no drops are comming for too long open it up a little.

    if (Servo_Val < servo_min) {
      if ((millis() - lastDrop) > (5 * open_delay * (60000.0 / set_DPM)) && (tuned == 1)) { //if no drops for 5x wait time, change min val
        servo_min--;
        Servo_Val--;
      }
      if (Servo_Val < 0) {
        Servo_Val = 0;
      }
      if (servo_min < 0) {
        servo_min = 0;
      }
    }

    myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
    delay(15);
    myservo.write(Servo_Val);
    Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read());
  }

  if ((millis() - lastDrop) > (10 * (60000.0 / set_DPM)) && (tuned == 0)) { // stricktly for tuning mode to stop in too closed
    if (debug >= 1) {
      Serial.println("open_up() - *** END TUNE ***");
    }
    closed_to_stop = Servo_Val + 3;
    servo_max = Servo_Val + 5;
    if (closed_to_stop > 180) {
      closed_to_stop = 180;
    }
    if (servo_max > 180) {
      servo_max = 180;
    }
    Blynk.virtualWrite(SERVO_MAX_PIN, servo_max);
    Blynk.virtualWrite(SERVO_MIN_PIN, servo_min);
    tuned = 1;
    if (Servo_Val < servo_min) {
      Servo_Val = servo_min;
    }
    if (Servo_Val > servo_max) {
      Servo_Val = servo_max;
    }
  }
}

void blue_LED_blink_on_off(){
  if (millis() - High_LED > 250) { // add LED indicator for tuning
    digitalWrite(LED_PIN, LOW);
    Low_LED = millis();
    High_LED = INFINITY;
  }
  if (millis() - Low_LED > 250) { // add LED indicator for tuning
    digitalWrite(LED_PIN, HIGH);
    High_LED = millis();
    Low_LED = INFINITY;
  }
}

void tune() {
  Blynk.run();
  if (debug >= 1) {
    Serial.println("tune()");
  }

  tuned = 0;
  for (/*nothing needed here*/; Servo_Val < servo_max; Servo_Val++) {

    myservo.attach(ServoPIN);
    delay(15);
    myservo.write(Servo_Val);

    for (int run_tune = 0; run_tune < tuning_drops; run_tune++) {

      while (voltage > 0) {
        Blynk.run();
        drop_no_interupt();
        pause_requests();
        timer.run();

        if (first_tune == 1 && (millis() - servo_min_tune_time) > 1000) {
          Servo_Val -= 1;
          myservo.attach(ServoPIN);
          myservo.write(Servo_Val);
          open_to_drop = Servo_Val - 10;
          if (Servo_Val < servo_min) Servo_Val = servo_min;
          if (open_to_drop < servo_min) open_to_drop = servo_min;
          servo_min_tune_time = millis();
        }

        blue_LED_blink_on_off();

        if (tuned == 1) {
          break;
        }
      }

      if (tuned == 0) {
        voltage = 5.0;
        first_tune = 0;

        measure_DPM();
        print_stats();
      }

      if (DPM_avg < max_DPM_to_tune && is_servo_min_tuned == 0 && tuned == 0) { //not done yet .. need to fully open until 100-DPM to get servo_min value

        while (DPM_avg < max_DPM_to_tune || count < 20) {
          Blynk.run();
          drop_no_interupt();
          pause_requests();
          timer.run();

          if ((millis() - servo_min_tune_time) > 1000) {
            Servo_Val -= 1;
            if (Servo_Val < 0) Servo_Val = 0;
            myservo.attach(ServoPIN);
            delay(15);
            myservo.write(Servo_Val);
            servo_min = Servo_Val;
            servo_min_tune_time = millis();
          }

          if (voltage == 0) {
            measure_DPM();
            print_stats();
            voltage = 5.0;
          }

          if (tuned == 1) {
            break;
          }
        }

      } else if (DPM_avg > max_DPM_to_tune && is_servo_min_tuned == 0 && tuned == 0) {
        Serial.println();Serial.print("minimum servo value: ");Serial.println(servo_min);Serial.println();
        is_servo_min_tuned = 1;
        temp = tuning_drops; // hold value until the next block
        tuning_drops = 1; //minimize the drops after MAX-DPM back until to MAX-DPM
      }

      if (DPM_avg < max_DPM_to_tune && kick_close_tuned == 0) {
        kick_close = (Servo_Val - servo_min) / max_DPM_to_tune; //this is the factor of % of DPM to kick_close
        Serial.println();Serial.print("the Closed slack is: ");Serial.print(kick_close);Serial.println(" xDPM");Serial.println();
        kick_close_tuned = 1;
        tuning_drops = temp; // restore value
      }

      if (DPM <= set_DPM && count > 10 && tuned == 0 && is_servo_min_tuned == 1) { //if DPM less than of equal to 6 DPM. EXIT, this gives closed value by adding 10

        closed_to_stop = Servo_Val + 3;
        servo_max = Servo_Val + 5;
        if (closed_to_stop > 180) {
          closed_to_stop = 180;
        }
        if (servo_max > 180) {
          servo_max = 180;
        }

        Serial.println();Serial.print("MAXIMUIM servo value: ");Serial.println(servo_max);
        Serial.print("Closed to stop servo value: ");Serial.println(closed_to_stop);Serial.println();

        while (DPM <= set_DPM) { //open back up to find the open slack
          Blynk.run();
          drop_no_interupt();
          pause_requests();
          timer.run();

          blue_LED_blink_on_off();

          if ((count - record_count) >= tuning_drops) {
            Servo_Val -= 1;
            if (Servo_Val < 0){ Servo_Val = 0;}
            myservo.attach(ServoPIN);
            delay(15);
            myservo.write(Servo_Val);
            record_count = count;

            Blynk.syncVirtual(MODE_DROPDOWN); //sync Mode from last setting on APP
            Blynk.syncVirtual(DPM_SLIDER); //sync setDPM from last setting on APP
            Blynk.syncVirtual(DropBOB_DEBUG); //sync Debug Button from last setting on APP
            Blynk.syncVirtual(BOTTLE_SELECT_PIN); //sync bottle size from last selection on APP
            sync_once_aft_tune++; //keep track number of times, so If greater than 10, don't do more in reconnectBlynk()
            
          }

          if (voltage == 0) {
            measure_DPM();
            print_stats();
            voltage = 5.0;
          }

          if (tuned == 1) {
            break;
          }
        }

        if (DPM > set_DPM && kick_open_tuned == 0) {
          kick_open = ((closed_to_stop + 3) - Servo_Val) / set_DPM; //this is the factor of % of DPM to kick_close
          Serial.println();Serial.print("the Open slack is: ");Serial.print(kick_open);Serial.println(" xDPM");Serial.println();
          kick_open_tuned = 1;
        }

        tuned = 1;
        Blynk.virtualWrite(SERVO_MAX_PIN, servo_max);
        Blynk.virtualWrite(SERVO_MIN_PIN, servo_min);
        
        if (Servo_Val < servo_min) {
          Servo_Val = servo_min;
        }
        if (Servo_Val > servo_max) {
          Servo_Val = servo_max;
        }
      }

      if (restart == 1 && tuned != 1) {
        restart = 0;
        tuned = 1;
      }

      if (tuned == 1) {
        break;
      }
    }
    if (tuned == 1) {
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
    Servo_update_Speed = 500; //std movement
    open_factor = 0.5; //std open factor
    kick_close = 0; // no kick_close
    kick_close_delay = 0; // no kick_close delay
    DPM_buffer = 0.5; // 1-DPM buffer
    open_bias = 0.8; // no open bias (0 to 1) ... 1 is no bias, 0 is the valve never opens if too closed (good for keeping the valve biased towards les DPM
    open_delay = 1.25; // back to normal tollerance
    Servo_movements = 1;      // how much to update servo position by
  }
  if (param.asInt() == 2) {
    Mode = 2; //Standard mode flag
    kp = 0.6, ki = 0.2 / 60000.0, kd = 900; //derivative term predicts output bu more jitter
    Servo_update_Speed = 1000; //reduce the movement speed (mostly when opening up)
    open_factor = 0.25; //let the system equalize longer by halfing the opening up factor.
    kick_close_delay = 200; // wait 200 milliseconds before returning
    DPM_buffer = 0; // no buffer, always update
    open_bias = 1; // open half as fast as you close
    open_delay = 1.5; // add some opening tollerance
    Servo_movements = 1;      // how much to update servo position by
  }
  if (param.asInt() == 3) {
    Mode = 3; //Forced mode flag
    kp = 0.9, ki = 0.1 / 60000.0, kd = 700; //std PI(D)
    time_to_stay_closed = 100; //in milliseconds
    open_delay = 5; //let it go up to 5x off course before correcting
    Servo_movements = 1;      // how much to update servo position by
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
}

BLYNK_WRITE(BOTTLE_SELECT_PIN) { //Menu SELECT BOTTLE size
  if (param.asInt() == 1) { // Small 300ml
    //per google 1 drop has 0.05ml (use this as baseline, should be updated each full run)
    bottle_size = 300;
    total_time = round(float(bottle_size) / float(ml_per_drop) / float(set_DPM) / 60.0); //in hours
  }
  if (param.asInt() == 2) { //Medium 500ml
    //per google 1 drop has 0.05ml (use this as baseline, should be updated each full run)
    bottle_size = 500;
    total_time = round(float(bottle_size) / float(ml_per_drop) / float(set_DPM) / 60.0); //in hours
  }
  if (param.asInt() == 3) { //Large 1L
    //per google 1 drop has 0.05ml (use this as baseline, should be updated each full run)
    bottle_size = 1000;
    total_time = round(float(bottle_size) / float(ml_per_drop) / float(set_DPM) / 60.0); //in hours
  }

}

void print_stats() {
  if (debug >= 1) {
    Serial.println("print_stats()");
  }
  
  if (DPM < 1000) {
    Serial.print(count);
    Serial.print("\t");
    if (DPM_Instant >= 100){
      printFloat(DPM_Instant,0); Serial.print(". -"); Serial.print("DPMi");
    } else if(DPM_Instant < 10){
      printFloat(DPM_Instant,2); Serial.print(" -"); Serial.print("DPMi");
    } else {
      printFloat(DPM_Instant,1); Serial.print(" -"); Serial.print("DPMi");
    }
    if(abs(DPM_Instant - set_DPM) > (set_DPM/2)){
      Serial.print("*");
    }
    Serial.print("\t");
    if (DPM >= 100){
      printFloat(DPM,0); Serial.print(". -"); Serial.print("dpm3");
    } else if (DPM < 10){
      printFloat(DPM,2); Serial.print(" -"); Serial.print("dpm3");
    } else {
      printFloat(DPM,1); Serial.print(" -"); Serial.print("dpm3");
    }
    if(abs(DPM - set_DPM) > (set_DPM/2)){
      Serial.print("*");
    }
    Serial.print("\t");
    if (DPM_avg >= 100){
      printFloat(DPM_avg,0); Serial.print(". -"); Serial.print("dpm10");
    } else if (DPM_avg < 10){
      printFloat(DPM_avg,2); Serial.print(" -"); Serial.print("dpm10");
    } else {
      printFloat(DPM_avg,1); Serial.print(" -"); Serial.print("dpm10");
    }
    if(abs(DPM_avg - set_DPM) > (set_DPM/2)){
      Serial.print("*");
    }
    Serial.print("\t");
    if(alltimeAVG_DPM >= 100){
      printFloat(alltimeAVG_DPM,0); Serial.print(". -"); Serial.print("dpmALL");
    } else if (alltimeAVG_DPM < 10){
      printFloat(alltimeAVG_DPM,2); Serial.print(" -"); Serial.print("dpmALL");
    } else {
      printFloat(alltimeAVG_DPM,1); Serial.print(" -"); Serial.print("dpmALL");
    }
    
    Serial.print("\t");
    Serial.print(set_DPM); Serial.print(" -"); Serial.print("SetDPM");
    Serial.print("\t");

    if ( Mode == 1 || Mode == 2 || tuned == 0 ) {
      Serial.print(myservo.read());
      Serial.print("->");
      printFloat(Servo_Val,0);
      Serial.print(" -");
      Serial.print("Ser_Ang ");
      if (tuned == 0) {
        Serial.println();
      }
    }
    if ( (Mode == 3) && (tuned == 1) ) {
      Serial.print(Servo_update_Speed);
      Serial.print("/");
      Serial.print(Servo_movements);
      Serial.print("/");
      Serial.print(close_factor);
      Serial.print(" -");
      Serial.print("u_Spd/mov/clos");
    }
    if (tuned == 1) {
      Serial.print("\t");
      if (error > 0) {
        Serial.print(" "); //to line up with negatives
      }
      printFloat(kp * error,1); Serial.print("'"); Serial.print("P");
      Serial.print("  ");
      if (errSum > 0) {
        Serial.print(" "); //to line up with negatives
      }
      printFloat(ki * errSum,1); Serial.print("'"); Serial.print("I");
      Serial.print("  ");
      if (dErr > 0) {
        Serial.print(" "); //to line up with negatives
      }
      printFloat(kd * dErr,1); Serial.print("'"); Serial.println("D");
    }

    //Print to terminal all the less important stats needed for Debugging
    if (debug >= 1) {
      if ( Mode == 1 || Mode == 2 || tuned == 0 ) {
        terminal.print(myservo.read()); terminal.print("->"); terminal.print(Servo_Val); terminal.print("svr_ang ");
        if (tuned == 0) {
          terminal.println();
        }
      }
      if ( (Mode == 3) && (tuned == 1) ) {
        terminal.print(Servo_update_Speed); terminal.print("/"); terminal.print(Servo_movements); terminal.print("/"); terminal.print(close_factor); terminal.print("u_spd/move/cls_f ");
      }
      if (tuned == 1) {
        terminal.print(kp * error); terminal.print("P "); terminal.print(ki * errSum); terminal.print("I "); terminal.print(kd * dErr); terminal.println("D");
      }
      terminal.flush(); //good practice, otherwise it doesn't fully print until the next go around
    }

    if (tuned == 1 || DPM_avg < 20) {
      yield();
      Blynk.virtualWrite(DPM_VIRTUAL_PIN, DPM);
      Blynk.virtualWrite(DPM_avg_VIRTUAL_PIN, DPM_avg);
      Blynk.virtualWrite(DPM_INSTANT_PIN, DPM_Instant);
      Blynk.virtualWrite(DROP_COUNT_VIRTUAL_PIN, count);
      Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read());
      Blynk.virtualWrite(SETPOINT_DPM_VIRT_PIN, set_DPM);
      Blynk.virtualWrite(SERVO_UPDATE_SPEED_VPIN, Servo_update_Speed);
      Blynk.virtualWrite(IS_DONE_VPIN, isDONE);
      
      Blynk.virtualWrite(ALLTIME_DROP_AVG_PIN, alltimeAVG_DPM);

      //use it either here or in the loop to print once per second ... Save about 100ms per sec if you use it here
      //but you won't see the uptime update once per second ... which doesn't feel right
      UpTime();

      if (debug >= 1) {
        Serial.print("print_stats(");
        Serial.print("time it took to upload: ");
        Serial.print(millis() - uptime);
        Serial.print("-ms");
        Serial.println(")");
      }
    }

    if (DPM_avg > 20 && do_once == 0) {
      do_once = 1;
      thLCD.print(0, 1, "DPM HIGH appHOLD");
    }
  }
}

void measure_DPM() {
  if (debug >= 1) {
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
  DPM_Instant = DPM;

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
  
  alltimeAVG_DPM = float(count)/(float(millis())/float(60000.0));
  
  isNearDrop = 0; // drop had droped. no longer near
}

void Servo_angle_method() { //(NORMAL & AGRESSIVE MODE) The servo angle is adjusted after every drop to maintain set_DPM
  // This method sets the servo position according to the latest drop DPM (more open or more closed
  // This method is very lagging as it uses old data, including the last 10 point average to set the future servo position

  if (voltage == 0 && state == HIGH) {
    if (debug >= 1) {
      Serial.println("Servo_angle_method()");
      Serial.print("###old servo value:"); Serial.println(Servo_Val);
    }

    if (Servo_Val >= servo_max) { //if drops still keep coming after servo max, increase max
      servo_max++;
      if (servo_max > 180) {
        servo_max = 180;
      }
      Blynk.virtualWrite(SERVO_MAX_PIN, servo_max);
    }

    state = LOW;
    measure_DPM();
    voltage = 5.0;

    /*Compute all the working error variables*/
    error = set_DPM - DPM;
    errSum += (error * delta);
    dErr = (error - lastErr) / delta;

    if ( abs(myservo.read() - Servo_Val) <= Servo_movements) { //only change servo value if reached new angle
      if ( (DPM_avg - set_DPM) > DPM_buffer && first_drop == 0) { //to add forward or reverse bias Uncomment the elseif loop and the DPM condition
        
        if (abs( kp * error + ki * errSum + kd * dErr) <= 1 || DPM_avg >= ((2)*set_DPM)) {
          Servo_Val = Servo_Val - ( kp * error + ki * errSum + kd * dErr); // Set servo change depending on how far away from set_DPM you are at
        } else {
          Servo_Val++; //if only small movements needed, one tick at a time
        }
        
        if (debug >= 1) {
          Serial.print("CHANGE--->");
          Serial.println(- ( kp * error + ki * errSum + kd * dErr));
        }
        if (Servo_Val < servo_min) {Servo_Val = servo_min;}
        if (Servo_Val > servo_max) {Servo_Val = servo_max;}

        if (debug >= 1) {
          Serial.println("Servo_angle_method(CLOSING)");
        }

        if (Mode == 2) {
          myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
          delay(15);
          if (DPM_Instant >= (2 * set_DPM)) {
            myservo.write(myservo.read() + (DPM_avg * kick_close));
            Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read());
            delay(kick_close_delay);
            if (debug >= 1) {
              Serial.println();
              Serial.println("kick(CLOSING):");
              Serial.println(DPM_avg);
              Serial.println(kick_close);
              Serial.println(DPM_avg * kick_close);
              Serial.println();
            }
          }
          myservo.write(Servo_Val);
          Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read());
        }
      } else if ( (DPM_avg - set_DPM) < -DPM_buffer && first_drop == 0) { // This is to give a Forward or Reverse BIAS (commented out)
        
        if (abs( kp * error + ki * errSum + kd * dErr) <= 1 || DPM_avg <= ((0.5)*set_DPM)) {
          Servo_Val = Servo_Val - open_bias * (kp * error + ki * errSum + kd * dErr); // Set servo change depending on how far away from set_DPM you are at
        } else {
          Servo_Val--; //if only small movements needed, one tick at a time
        }
        
        if (debug >= 1) {
          Serial.print("CHANGE--->");
          Serial.println(- open_bias * (kp * error + ki * errSum + kd * dErr));
        }
        if (Servo_Val < servo_min) {Servo_Val = servo_min;}
        if (Servo_Val > servo_max) {Servo_Val = servo_max;}

        if (debug >= 1) {
          Serial.println("Servo_angle_method(OPENING)");
        }

        if (Mode == 2) {
          myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
          delay(15);
          if (DPM_Instant <= ((0.5)*set_DPM)) {
            myservo.write(myservo.read() - (DPM_avg * kick_open));
            Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read());
            delay(kick_close_delay);
            if (debug >= 1) {
              Serial.println();
              Serial.println("kick(Opening)");
              Serial.println(DPM_avg);
              Serial.println(kick_close);
              Serial.println(DPM_avg * kick_open);
              Serial.println();
            }
          }
          myservo.write(Servo_Val);
          Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read());
        }
      }
    }

    if (debug >= 1) {
      Serial.print("*** NEW servo value:"); Serial.println(Servo_Val);
    }

    print_stats();
    first_drop = 0;
  }

  if (voltage == 0 && state == LOW ) { // end of pulse, now we may expect a new one, DEBOUNCE
    if ((millis() - lastDrop) > 50) state = HIGH; // only go back to state high if some time has passed.
  }

  lastErr = error;
}

void Speed_to_open_method() { //(FORCED MODE) The speed at which the servo is opened to let out a drop is adjusted after every drop to maintain set_DPM
  //The difference with this method is that it always opens the servo. So there is no possibility to "stall" like the Servo angle method
  //which waits for a drop then potentially sets the angle to a "bad" angle and can wait indefinitely, until the openup kick_closes in
  //this should be more acurate but also cause more servo noise and it will constantly be running and possibly closing quickly
  //after each drop (TBD)*/

  if (voltage == 0) { //Drop
    if (debug >= 1) {
      Serial.println("Speed_to_open_method()");
    }

    //this can be easily captured with interrupts, but must do it this way without interrupts
    if ( isNearDrop == 0 && isInterrupt == 0) { //why did you come drop? premature dropulation ...
      servo_max++;
      closed_to_stop++;
      Blynk.virtualWrite(SERVO_MAX_PIN, servo_max);
      if (closed_to_stop > 180) {
        closed_to_stop = 180;
      }
      if (servo_max > 180) {
        servo_max = 180;
      }
    } else if (isNearDrop == 1 && isInterrupt == 0) { //successful drop
      if (record_count == 0) {
        record_count = count;
      }
      if ((count - record_count) >= 10) { //10 succesful closes and re-opens in a row
        servo_max--;
        closed_to_stop--;
        Blynk.virtualWrite(SERVO_MAX_PIN, servo_max);
        if (closed_to_stop < 0) {
          closed_to_stop = 0;
        }
        if (servo_max < 0) {
          servo_max = 0;
        }
        record_count = 0; //clear recording
      }
    }

    voltage = 5.0;
    measure_DPM();
    open_to_drop = Servo_Val; // record the opening angle that caused a drop
    Servo_Val = closed_to_stop; // Close the servo
    if (Servo_Val < servo_min) Servo_Val = servo_min;
    if (Servo_Val > servo_max) Servo_Val = servo_max;

    //Serial.print("Servo val: ");Serial.println(Servo_Val);

    if (isInterrupt == 1) { //only do this if interrupts are on ... doesn't make sence without
      record_count = count;
    }

    while (myservo.read() < (servo_max - Servo_movements) && myservo.read() < 180) {
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      if (Servo_update_Speed > (close_factor * 5)) {
        delay(Servo_update_Speed / close_factor); //don't go less than 5ms
      } else {
        delay(5);
      }
      if (myservo.read() + Servo_movements < servo_max) {
        myservo.write(myservo.read() + Servo_movements);
      }
      //Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read()); // this makes it close too slow
      if (debug >= 1) {
        Serial.print("Speed_to_open_method("); Serial.print("closing valve: "); Serial.print(myservo.read()); Serial.println(")");
      }
    }

    if (isInterrupt == 1) { //only do this if interrupts are on ... doesn't make sence without
      if (count > record_count) {
        if (close_factor < 100) {
          close_factor += 1;
          if (debug >= 1) {
            Serial.println("%%close_factor ++%%");
          }
        }
      } else {
        is_close_success += 1;
      }

      if (is_close_success > 10) {
        if (close_factor > 1) {
          close_factor -= 1;
          if (debug >= 1) {
            Serial.println("%%close_factor --%%");
          }
        } is_close_success = 0;
      }
    }

    /*Compute all the working error variables*/
    error = set_DPM - DPM;
    errSum += (error * delta);
    dErr = (error - lastErr) / delta;

    if (abs(DPM_avg - set_DPM) > DPM_buffer && first_drop == 0) { //to add forward or reverse bias Uncomment the elseif loop and the DPM condition
      Servo_update_Speed -= 10 * ( kp * error + ki * errSum + kd * dErr); // Set servo change depending on how far away from set_DPM you are at
      if (Servo_update_Speed < 0) {
        Servo_update_Speed += 10 * ( kp * error + ki * errSum + kd * dErr); // put it back if negative
        Servo_movements += 1; //but if im less than zero, I need to make it faster ... increase step size
        if (Servo_movements > 10) {
          Servo_movements = 10; // max
          Servo_update_Speed = 0; //if maxed out, then keep maxed out!
        }
      }
      if (Servo_update_Speed > 500) {
        Servo_movements -= 1; //but if im too slow, I need to make it slower ... decrease step size
        Servo_update_Speed += 10 * ( kp * error + ki * errSum + kd * dErr); // put it back if too high
        if (Servo_movements < 1) {
          Servo_movements = 1; // min
          Servo_update_Speed = 500; //if maxed out, then keep maxed out!
        }
      }
    }

    print_stats();
    first_drop = 0;
  }

  if (voltage == 5 && state == LOW ) { // end of pulse, now we may expect a new one, DEBOUNCE
    if (millis() - lastDrop > 50) state = HIGH; // only go back to state high if some time has passed.
  }

  if ( (millis() - lastDrop) > time_to_stay_closed && myservo.read() > (open_to_drop + Servo_movements) && voltage == 5) {
    Servo_Val = open_to_drop;
  }

  if (abs(myservo.read() - Servo_Val) <= Servo_movements) { //has reached open, but still no drop
    Servo_Val -= 1;
  }
}

void run_blynk() {
  if (debug >= 2) {
    Serial.println("run_blynk()");
  }
  yield();
  //if (Blynk.connected()) {
  Blynk.run();
  //}
}

BLYNK_CONNECTED() {
  if (isFirstConnect) {
    if (debug >= 1) {
      Serial.println("BLYNK_CONNECTED()");
    }
    //Console.println("*** Sync All ? ***");
    //Blynk.syncAll();
    Blynk.syncVirtual(MODE_DROPDOWN); //sync Mode from last setting on APP
    Blynk.syncVirtual(DPM_SLIDER); //sync setDPM from last setting on APP
    Blynk.syncVirtual(DropBOB_DEBUG); //sync Debug Button from last setting on APP
    Blynk.syncVirtual(BOTTLE_SELECT_PIN); //sync bottle size from last selection on APP
    isFirstConnect = false;
  }
}

void reconnectBlynk() {
  if (!Blynk.connected()) {
    if (debug >= 1) {
      Serial.println("reconnectBlynk()");
    }
    if (Blynk.connect()) {
      Serial.println("Reconnected");
    } else {
      Serial.println("Not reconnected");
    }
  }

  if(tuned == 1 && sync_once_aft_tune < 10){ //do once per timer call for 10 iterations to make sure it takes
    //also once per interval, syncronyse these:
    Blynk.syncVirtual(MODE_DROPDOWN); //sync Mode from last setting on APP
    Blynk.syncVirtual(DPM_SLIDER); //sync setDPM from last setting on APP
    Blynk.syncVirtual(DropBOB_DEBUG); //sync Debug Button from last setting on APP
    Blynk.syncVirtual(BOTTLE_SELECT_PIN); //sync bottle size from last selection on APP
    sync_once_aft_tune++;
  }
}

void saveConfigCallback () {
  if (debug >= 1) {
    Serial.println("saveConfigCallback ()");
  }
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void UpTime() {
  if ((millis() - uptime) > 1000 && isNearDrop == 0) { //update uptime LCD & uptime variable
    if (debug >= 1) {
      Serial.print("uptime: ");
      Serial.print(millis());
      Serial.println("-s");
    }
    uptime = millis();
    Blynk.virtualWrite(UPTIME_VIRTUAL_PIN, (int)millis() / 1000); //print uptime in seconds, not ms
    Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read());
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
    if (debug >= 1) {
      Serial.print("time it took to upload: ");
      Serial.print(millis() - uptime);
      Serial.println("-ms");
    }
  }
}

void topline_function() { // used to create the topline of the LCD during standard operation

  topLine = ""; //clear

  topLine = eta;
  if (total_time < 10) {
    topLine += "0"; // add leading zero to make constant
  }
  topLine += total_time - round(float(millis()) / 3600000.0); //estimate the total time left by remouving elapsed
  topLine += "H ";

  // 7 char used for ETA above

  switch (Mode) {
    case 1:
      topLine += mode1;
      break;
    case 2:
      topLine += mode2;
      break;
    case 3:
      topLine += mode3;
      break;
    default:
      topLine += "error!";
      break;
  }

  // 6 characters used for showing MODE ... leaving 1 char at the end for future.

}

void setup() {
  Serial.begin(19200);
  delay(10);
  if (debug >= 1) {
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
    ESP.restart();
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
      delay(1000);
      myservo.write(myservo.read() - Servo_movements);
      Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read());
    }
    else if (myservo.read() < Servo_Val) {
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(1000);
      myservo.write(myservo.read() + Servo_movements);
      Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read());
    }

    delay(50);

  }

  Serial.println("DropBOB v2.5");
  Serial.println();

  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0; // Initialize the array

  pinMode(photo_interuptor_PIN, INPUT);

  if (isInterrupt == 1) {
    attachInterrupt(digitalPinToInterrupt(photo_interuptor_PIN), drop_interrupt, FALLING); //possibly also Mode: LOW(no good),FALLING(best),CHANGE(too many),RISING(no)
  }

  pinMode(ServoPIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SleepPin, INPUT_PULLUP);

  delay(25);
  Serial.println(!digitalRead(SleepPin));
  delay(25);

  if (!digitalRead(SleepPin)) {
    while (!digitalRead(SleepPin)) {
      digitalWrite(LED_PIN, LOW);  led_gp5 = LOW;
      delay(50);
      digitalWrite(LED_PIN, HIGH); led_gp5 = HIGH;
      delay(50);
    }
    wifiManager.resetSettings();  //clear settings
    SPIFFS.format();              //clear File System
    Serial.println("settings cleared ... RESETTING");
    delay(3000);
    ESP.restart();
    delay(5000);
  }

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

  timer.setInterval(10000L, open_up); // open up the servo every 30 seconds if no drops come ... not a Blynk update

  reconnectBlynk();

  while (!Blynk.connected()) {
    //wait for connection
    Blynk.run();
  }

  thLCD.clear(); // Clear the LCD
  thLCD.print(0, 0, "Tuning(~5-10min)"); // Print top line
  thLCD.print(0, 1, "please wait ... "); // Print bottom line

  isDONE = 0;
  Blynk.virtualWrite(IS_DONE_VPIN, isDONE);

  tune(); //start by tuning the system, No blynk update during tuning mode

  //these timers should run only after system is tuned (interferes with fast drop reading)
  timer.setInterval(500L, run_blynk);
  timer.setInterval(60000L, reconnectBlynk); //only atempt reconnection once per minute
}//================================================================================END SETUP========================

void loop() {

  if ((millis() - lastDrop) > ((0.75*set_DPM/DPM_Instant) * (60000.0 / set_DPM)) && isNearDrop == 0) {
    isNearDrop = 1;
    thLCD.print(11, 1, DropNEAR);
  }

  if (debug >= 2) {
    Serial.println("loop()");
  }

  pause_requests(); //accept pause requests
  Blynk.run();
  timer.run(); // Initiates SimpleTimer

  digitalWrite(LED_PIN, HIGH); // for some odd reason ... LED PIN to "HIGH" means "off"

  drop_no_interupt();

  if ( (Mode == 1) || (Mode == 2) ) { //Only if Mode = Normal (1) or Agressive (2) do this
    Servo_angle_method();
    topline_function();
    UpTime(); //Uptime takes away about 100ms
  }

  if (Mode == 3) { //Only if Mode = Forced (3) do this
    Speed_to_open_method();
    //NOTE: can't show uptime on Forced Mode with intterrupts ... Too much servo interference.
    topline_function();
    UpTime(); //Uptime takes away about 100ms
  }

  if ( (millis() - Servo_adjust) >= Servo_update_Speed) {
    Servo_adjust = millis();
    if ((myservo.read() - Servo_Val) <= -Servo_movements ) {
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(5);
      myservo.write(myservo.read() + Servo_movements);
      //Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read()); //this is nice but unnecessary, slows down code

      if (debug >= 1) {
        Serial.print("loop(");
        Serial.print("Smooth-closing valve: ");
        Serial.print(myservo.read());
        Serial.println(")");
        Serial.print(" going to: ");
        Serial.println(Servo_Val);
      }
    }
    else if ((myservo.read() - Servo_Val) >= Servo_movements) {
      myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
      delay(5);
      myservo.write(myservo.read() - Servo_movements);
      //Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read()); //this is nice but unnecessary, slows down code

      if (debug >= 1) {
        Serial.print("loop(");
        Serial.print("Smooth-opening valve: ");
        Serial.print(myservo.read());
        Serial.print(")");
        Serial.print(" going to: ");
        Serial.println(Servo_Val);
      }
    }
  }

  //=============if open to the max for 6 min and nothing comes out ... your done!
  if ( (millis() - lastDrop) > 360000 && Servo_Val < 5) { //Servo must be at least all the way down to 5deg open
    Serial.println(); Serial.println("FINISHED!!!");
    //Blynk.tweet("Brew DONE!!: www.bobbobblogs.blogspot.com");
    myservo.detach();
    thLCD.clear(); // Clear the LCD
    thLCD.print(0, 0, "Finished in:"); // Print top line
    thLCD.print(0, 1, botLine); // Print bottom line

    while (restart == 0)
    {
      isDONE = 1;
      if ((millis() - last_interrupt_time) > 10000) { //needed otherwise it dowsn't show the finished on the LCD
        thLCD.print(0, 0, "Finished in:"); // Print top line
        thLCD.print(0, 1, botLine); // Print bottom line
        last_interrupt_time = millis();
        Blynk.virtualWrite(IS_DONE_VPIN, isDONE);
      }
      Blynk.run();
      pause_requests(); //accept pause requests
    } // when finished do nothing but listen for Blynk app

    Serial.println("Restarting DropBOB ... please wait");
    delay(3000);
    ESP.restart();
    delay(5000);
  }
}

void drop_interrupt() {
  // If interrupts come faster than X-ms, assume it's a bounce and ignore
  if ((millis() - last_interrupt_drop) > 70)
  {
    if (debug >= 1) {
      Serial.println("");
      Serial.println("drop_interrupt() ... ... ...");
    }
    voltage = 0;
    count++;
    last_interrupt_drop = millis();
  }
}

void drop_no_interupt() {
  if (isInterrupt == 0) {
    raw = analogRead(photo_interuptor_PIN); // read the drop sensor
    if (raw == 0 && (millis() - last_interrupt_drop) > 70) {
      voltage = 5.0 * raw / 1023; // convert it to voltage
      if (debug >= 1) {
        Serial.println("drop_no_interupt() ... ... ... ");
      }
      count++;
      last_interrupt_drop = millis();
    }
  }
}

void sleep_switch() { //Interupt
  if (debug >= 1) {
    Serial.println("sleep_switch()");
  }

  myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
  delay(25);
  myservo.write(servo_max);

  Blynk.virtualWrite(SERVO_ANGLE_VIRTUAL_PIN, myservo.read());

  Serial.println("...Going to Sleep...");
  thLCD.clear(); // Clear the LCD
  thLCD.print(0, 0, "Going to Sleep"); // Print top line
  thLCD.print(0, 1, "zzzzzzzzzzzzzZ"); // Print bottom line
  delay(1000);
  ESP.deepSleep(0, WAKE_RF_DEFAULT); // Sleep forever, until Pin#16 is un-grounded (button un-pushed)
  delay(1000);
}

// printFloat prints out the float 'value' rounded to 'places' places after the decimal point
void printFloat(float value, int places) {
  // this is used to cast digits 
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  int i;
  float tempfloat = value;

    // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
  // if this rounding step isn't here, the value  54.321 prints as 54.3209

  // calculate rounding term d:   0.5/pow(10,places)  
  float d = 0.5;
  if (value < 0)
    d *= -1.0;
  // divide by ten for each decimal place
  for (i = 0; i < places; i++)
    d/= 10.0;    
  // this small addition, combined with truncation will round our values properly 
  tempfloat +=  d;

  // first get value tens to be the large power of ten less than value
  // tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

  if (value < 0)
    tempfloat *= -1.0;
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }


  // write out the negative if needed
  if (value < 0)
    Serial.print('-');

  if (tenscount == 0)
    Serial.print(0, DEC);

  for (i=0; i< tenscount; i++) {
    digit = (int) (tempfloat/tens);
    Serial.print(digit, DEC);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }

  // if no places after decimal, stop now and return
  if (places <= 0)
    return;

  // otherwise, write the point and continue on
  Serial.print('.');  

  // now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
  for (i = 0; i < places; i++) {
    tempfloat *= 10.0; 
    digit = (int) tempfloat;
    Serial.print(digit,DEC);  
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit; 
  }
}
