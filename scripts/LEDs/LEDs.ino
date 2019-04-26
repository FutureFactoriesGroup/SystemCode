#include <FastLED.h>

#define NUM_LEDS 3
#define DATA_PIN 7

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

CRGB leds[NUM_LEDS];

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
}

void loop() {
  // print the string when a newline arrives:
  if (stringComplete) {
    //Serial.println(inputString);
    // clear the string:
    if(inputString == "on\n"){
      FastLED.setBrightness(255);
      FastLED.show();
      leds[0] = CRGB::Red;
      FastLED.show();
      delay(1000);
      leds[1] = CRGB::Blue;
      FastLED.show();
      delay(1000);
      leds[2] = CRGB::Green;
      FastLED.show();
      delay(1000);
      FastLED.setBrightness(0);
      FastLED.show();
      delay(500);
      FastLED.setBrightness(255);
      FastLED.show();
      delay(500);
       FastLED.setBrightness(0);
      FastLED.show();
      delay(500);
      FastLED.setBrightness(255);
      FastLED.show();
      delay(500);
    }
    else if(inputString == "off\n"){
      FastLED.setBrightness(0);
      FastLED.show();
    }
    inputString = "";
    stringComplete = false;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
