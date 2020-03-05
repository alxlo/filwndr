#include <Arduino.h>
#include <U8g2lib.h>
#include <Rotary.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

/*
  U8g2lib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.
    
  This is a page buffer example.    
*/


U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display


#define SUN	0
#define SUN_CLOUD  1
#define CLOUD 2
#define RAIN 3
#define THUNDER 4

Rotary r = Rotary(GPIO_NUM_32, GPIO_NUM_33);  //rotary encoder on pins GPIO32 and GPIO33
#define GPIO_BIT_MASK  ((1ULL<<GPIO_NUM_32) | (1ULL<<GPIO_NUM_33)) 

int position = 0;

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120
// Acceleration and deceleration values are always in FULL steps / s^2
#define MOTOR_ACCEL 200
#define MOTOR_DECEL 100
#define DIR GPIO_NUM_16
#define STEP GPIO_NUM_17
#define SLEEP GPIO_NUM_18

#include "A4988.h"
#define MS1 GPIO_NUM_25
#define MS2 GPIO_NUM_26
#define MS3 GPIO_NUM_27

A4988 stepper(MOTOR_STEPS, DIR, STEP, SLEEP, MS1, MS2, MS3);


#ifndef WIFI_SSID
#define WIFI_SSID "You should not put your SSID here"
#endif
#ifndef WIFI_PASSWD
#define WIFI_PASSWD "Define your WLAN AP password in platformio_overide.ini"
#endif
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWD;


void drawWeatherSymbol(u8g2_uint_t x, u8g2_uint_t y, uint8_t symbol)
{
  // fonts used:
  // u8g2_font_open_iconic_embedded_6x_t
  // u8g2_font_open_iconic_weather_6x_t
  // encoding values, see: https://github.com/olikraus/u8g2/wiki/fntgrpiconic
  
  switch(symbol)
  {
    case SUN:
      u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
      u8g2.drawGlyph(x, y, 69);	
      break;
    case SUN_CLOUD:
      u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
      u8g2.drawGlyph(x, y, 65);	
      break;
    case CLOUD:
      u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
      u8g2.drawGlyph(x, y, 64);	
      break;
    case RAIN:
      u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
      u8g2.drawGlyph(x, y, 67);	
      break;
    case THUNDER:
      u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
      u8g2.drawGlyph(x, y, 79);
      break;      
  }
}

void drawWeather(uint8_t symbol, int degree)
{
  drawWeatherSymbol(0, 42, symbol);
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.setCursor(16+3, 42);
  u8g2.print(degree);
  //u8g2.print("°C");		// requires enableUTF8Print()
}


void draw(uint8_t symbol, int degree)
{
  u8g2.firstPage();
  do {
    drawWeather(symbol, degree);
  } while ( u8g2.nextPage() );
}

void IRAM_ATTR rotaryChanged() {
    unsigned char result = r.process();
    if (result) {
      Serial.println(result == DIR_CW ? "Right" : "Left");
      position+= result == DIR_CW ? -1 : 1;
    }
}

void setup(void) {
  Serial.begin(115200);

  u8g2.begin();  
  u8g2.enableUTF8Print();
  r.begin(true);
  attachInterrupt(digitalPinToInterrupt(GPIO_NUM_32), rotaryChanged, CHANGE);
  attachInterrupt(digitalPinToInterrupt(GPIO_NUM_33), rotaryChanged, CHANGE);
  stepper.begin(RPM);
  stepper.setSpeedProfile(stepper.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepper.enable();

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
}

void loop(void) {

  draw(THUNDER, position);
  //draw("The sun's come out!", SUN_CLOUD, 19);
  //draw("It's raining cats and dogs.", RAIN, 8);
  //draw("That sounds like thunder.", THUNDER, 12);
  //draw("It's stopped raining", CLOUD, 15);

    delay(1000);

    /*
     * Moving motor in full step mode is simple:
     */
    stepper.setMicrostep(1);  // Set microstep mode to 1:1

    // One complete revolution is 360°
    stepper.rotate(360);     // forward revolution
    delay(1000);
    stepper.rotate(-360);    // reverse revolution
    delay(1000);
    // One complete revolution is also MOTOR_STEPS steps in full step mode
    stepper.move(MOTOR_STEPS*3);    // forward revolution
    delay(1000);
    stepper.move(-MOTOR_STEPS);   // reverse revolution
    delay(1000);
    /*
     * Microstepping mode: 1, 2, 4, 8, 16 or 32 (where supported by driver)
     * Mode 1 is full speed.
     * Mode 32 is 32 microsteps per step.
     * The motor should rotate just as fast (at the set RPM),
     * but movement precision is increased, which may become visually apparent at lower RPMs.
     */
    stepper.setMicrostep(8);   // Set microstep mode to 1:8

    // In 1:8 microstepping mode, one revolution takes 8 times as many microsteps
    
    stepper.move(8 * MOTOR_STEPS*3);    // forward revolution
    delay(1000);
    stepper.move(-8 * MOTOR_STEPS);   // reverse revolution
    delay(1000);
    // One complete revolution is still 360° regardless of microstepping mode
    // rotate() is easier to use than move() when no need to land on precise microstep position
    stepper.rotate(360);
    delay(1000);
    stepper.rotate(-360);


  for(;;){

      draw(CLOUD, position);
      ArduinoOTA.handle();
  }
    //Serial.println("foo"); 
    //position++;
  
}


//https://github.com/laurb9/StepperDriver?utm_source=platformio&utm_medium=piohome





