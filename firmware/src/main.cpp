#include <Arduino.h>
#include <U8g2lib.h>
#include <Rotary.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h> 

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// ### DISPLAY ###
// vanilla SSD1306 board without Reset pin on I2C SCL (GPIO22) and SDA (GPIO21)
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);  

// ### ROTARY ENCODER for user input ###
// 20 Position 360 Degree Rotary Encoder EC11 w Push Button 
#define ENCODER_PIN_A GPIO_NUM_33
#define ENCODER_PIN_B GPIO_NUM_32
Rotary r = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);  //rotary encoder on pins GPIO32 and GPIO33


int rpm = 10; //revolutions per Minute

// ### STEPPER MOTOR ###
// 40mm Nema17 Stepper Motor 42BYGH 1.7A (17HS4401) 
// Stepper Driver A4988

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 400  // Steps per Revolution, we use Half-Steps, therefore 400
#define RPM 120
// Acceleration and deceleration values are always in FULL steps / s^2
#define MOTOR_ACCEL 200  // in steps per second per second
#define MOTOR_DECEL 100
#define STEPPER_DIR GPIO_NUM_16
#define STEPPER_STEP GPIO_NUM_17
#define STEPPER_SLEEP GPIO_NUM_18
#define STEPPER_MS1 GPIO_NUM_25
#define STEPPER_MS2 GPIO_NUM_26
#define STEPPER_MS3 GPIO_NUM_27

// ### SERVO ###
// Hobbyking HK15148 Analog Servo 2.5kg / 0.14sec / 17

#define SERVO_PIN 13
Servo myservo;  // create servo object to control a servo

// ### OTA + WiFi ###

#ifndef WIFI_SSID
#define WIFI_SSID "You should not put your SSID here"
#endif
#ifndef WIFI_PASSWD
#define WIFI_PASSWD "Define your WLAN AP password in platformio_overide.ini"
#endif
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWD;

bool OTA_in_Progress = false;

// max 600 rpm = 1 r/sec = 400 steps/sec --> 1 step = 2500 uS
// timer clock divison = 80 --> 1 tick per uS
// 1 rpm = 400 ticks/min --> 1 tick per 150000 hs
volatile uint64_t stepCounter = 0;
volatile uint64_t tPhaseUs = 15000;
volatile uint8_t stepLevel = LOW;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 


void draw(int degree, int steps)
{
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_logisoso16_tf);
    u8g2.setCursor(16+3, 42);
    u8g2.print(degree);
    u8g2.setCursor(16+3, 61);
    u8g2.print(steps);
  } while ( u8g2.nextPage() );
}

void IRAM_ATTR rotaryChanged() {
    unsigned char result = r.process();
    if (result) {
      Serial.println(result == DIR_CW ? "Right" : "Left");
      rpm+= result == DIR_CW ? -1 : 1;
      if (rpm<1) rpm=1;
    }
}

void IRAM_ATTR onStepUpTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  if (stepLevel==LOW) {
    stepLevel = HIGH;
    stepCounter++;
    timerAlarmWrite(timer, tPhaseUs, true);  
  } else {
    stepLevel = LOW;
    timerAlarmWrite(timer, 10, true);  
  }
  portEXIT_CRITICAL_ISR(&timerMux);
  digitalWrite(STEPPER_STEP, stepLevel);  
}



void setup(void) {
// configure stepper driver A4988
  pinMode(STEPPER_SLEEP, OUTPUT);
  digitalWrite(STEPPER_SLEEP, LOW);
  pinMode(STEPPER_DIR, OUTPUT);
  digitalWrite(STEPPER_DIR, HIGH);
  pinMode(STEPPER_STEP, OUTPUT);
  digitalWrite(STEPPER_STEP, LOW);
  // 400 Half-Steps
  pinMode(STEPPER_MS1, OUTPUT);
  digitalWrite(STEPPER_MS1, HIGH);
  pinMode(STEPPER_MS2, OUTPUT);
  digitalWrite(STEPPER_MS2, LOW);
  pinMode(STEPPER_MS3, OUTPUT);
  digitalWrite(STEPPER_MS3, LOW);
  Serial.begin(115200);
  Serial.println("Booting");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA
      .onStart([]() {
        OTA_in_Progress = true;
        myservo.detach();
        timerDetachInterrupt(timer);
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
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

  myservo.setPeriodHertz(50);// Standard 50hz servo
  myservo.attach(SERVO_PIN, 500, 2400); 
  
  u8g2.begin();   
  u8g2.enableUTF8Print();
  r.begin(true);
  attachInterrupt(digitalPinToInterrupt(GPIO_NUM_32), rotaryChanged, CHANGE);
  attachInterrupt(digitalPinToInterrupt(GPIO_NUM_33), rotaryChanged, CHANGE);
  tPhaseUs= 60000000/(400*rpm)-10;
  timer = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &onStepUpTimer, true);
  timerAlarmWrite(timer, 100, true);  ////
  digitalWrite(STEPPER_SLEEP, HIGH); ////
  timerAlarmEnable(timer);

 
}

void loop(void) {

  
  
  delay(1000);
  long steps = 0;
  for(;;){
    draw(rpm, steps);
    //myservo.write(position);
    ArduinoOTA.handle();
    delay(50);
    portENTER_CRITICAL(&timerMux);
    tPhaseUs= 60000000/(400*rpm)-100;
    steps = stepCounter;    
    portEXIT_CRITICAL(&timerMux);
    
  }
    //Serial.println("foo"); 
    //position++;
  
}


//https://github.com/laurb9/StepperDriver?utm_source=platformio&utm_medium=piohome



// https://randomnerdtutorials.com/esp32-pinout-reference-gpios/


// https://github.com/laurb9/StepperDriver = StepperDriver@1.2.0
// is blocking while waiting for the next pulse and can not handle
// speed changes during move

// Maybe https://github.com/Stan-Reifel/FlexySteppe = FlexyStepper@1.0.0
// is a better solution?



 