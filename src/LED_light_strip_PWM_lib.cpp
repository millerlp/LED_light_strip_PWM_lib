// LED_light_strip_PWM.cpp
// Luke Miller 2020
// Functions for use with a MusselBedHeater RevF board that will 
// drive PWM LED light strips (Govee brand) in a common anode arrangement

#include "LED_light_strip_PWM_lib.h"

//-------------------------------------------------------------------
// Initialize the onboard RGB LED on the MusselBedHeater circuit board
// Default intialization using pins defined in LED_light_strip_PWM.h

ONBOARDLED::ONBOARDLED(){};
ONBOARDLED::~ONBOARDLED(){};

void ONBOARDLED::begin(){
  m_redled = 9;
  m_greenled = 5;
  m_blueled = 6;
  pinMode(m_redled, OUTPUT);
  pinMode(m_greenled, OUTPUT);
  pinMode(m_blueled, OUTPUT);
  if (COMMON_ANODE){
    digitalWrite(m_redled, HIGH); // for common anode LED, set high to shut off
    digitalWrite(m_greenled, HIGH);
    digitalWrite(m_blueled, HIGH);  
  } else {
    digitalWrite(m_redled, LOW); // for common cathode LED, set low to shut off
    digitalWrite(m_greenled, LOW);
    digitalWrite(m_blueled, LOW);     
  }
  
}

// User-specified pins for red, green, blue channels of LED
void ONBOARDLED::begin(uint8_t redpin, uint8_t greenpin, uint8_t bluepin){
  m_redled = redpin;
  m_greenled = greenpin;
  m_blueled = bluepin;
  pinMode(m_redled, OUTPUT);
  pinMode(m_greenled, OUTPUT);
  pinMode(m_blueled, OUTPUT);
  if (COMMON_ANODE){
    digitalWrite(m_redled, HIGH); // for common anode LED, set high to shut off
    digitalWrite(m_greenled, HIGH);
    digitalWrite(m_blueled, HIGH);  
  } else {
    digitalWrite(m_redled, LOW); // for common cathode LED, set low to shut off
    digitalWrite(m_greenled, LOW);
    digitalWrite(m_blueled, LOW);     
  }
}
//-----------------setColor---------------------
// Enter a set of values 0-255 for the red, green, and blue LED channels
void ONBOARDLED::setColor(uint8_t red, uint8_t green, uint8_t blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(m_redled, red);
  analogWrite(m_greenled, green);
  analogWrite(m_blueled, blue);  
}



// Convert a given HSV (Hue Saturation Value) to RGB(Red Green Blue) and set the led to the color
//   h is hue value, integer between 0 and 360
//   s is saturation value, double between 0 and 1
//   v is value, double between 0 and 1
// Stolen from: http://eduardofv.com/read_post/179-Arduino-RGB-LED-HSV-Color-Wheel-
// Based on: http://www.splinter.com.au/converting-hsv-to-rgb-colour-using-c/
void ONBOARDLED::writeHSV(int h, double s, double v, double R255) {
  //this is the algorithm to convert from RGB to HSV
  double r=0; 
  double g=0; 
  double b=0;

//  double hf=h/60.0;

  int i=(int)floor(h/60.0);
  double f = h/60.0 - i;
  double pv = v * (1 - s);
  double qv = v * (1 - s*f);
  double tv = v * (1 - s * (1 - f));

  switch (i)
  {
  case 0: //red dominant
    r = v;
    g = tv;
    b = pv;
    break;
  case 1: //green dominant
    r = qv;
    g = v;
    b = pv;
    break;
  case 2: 
    r = pv;
    g = v;
    b = tv;
    break;
  case 3: //blue dominant
    r = pv;
    g = qv;
    b = v;
    break;
  case 4:
    r = tv;
    g = pv;
    b = v;
    break;
  case 5: //red dominant
    r = v;
    g = pv;
    b = qv;
    break;
  }

  //set each component to a integer value between 0 and 255
  int red=constrain((int)255*r,0,255);
  int green=constrain((int)255*g,0,127); // Trying to balance green's brightness
  int blue=constrain((int)255*b,0,255);
#ifdef LOGFADE  
  // Map values into non-linear space
   red = pow(2,(red/R255)) - 1;
   green = pow(2,(green/R255)) - 1;
   blue = pow(2,(blue/R255)) - 1;
#endif

  setColor(red,green,blue);
}


//------------------------------------------------------------------------
// Version for the LED light strips controlled by the PCA9685 PWM chip
// rscale is a scaling factor calculated as 
// rscale = (pwmIntervals * log10(2))/(log10(4096));
// where pwmIntervals is the number of steps you want in the PWM scale, and
// 4096 is the maximum PWM value (for a 12-bit resolution PWM chip)

void convertHSV(int h, double s, double v, double rscale, uint16_t &rOut, uint16_t &gOut, uint16_t &bOut) {
  //this is the algorithm to convert from RGB to HSV
  // See https://github.com/FastLED/FastLED/wiki/Pixel-reference#chsv for info on the
  // HSV set
  double r=0; 
  double g=0; 
  double b=0;

//  double hf=h/60.0;

  int i=(int)floor(h/60.0);
  double f = h/60.0 - i;
  double pv = v * (1 - s);
  double qv = v * (1 - s*f);
  double tv = v * (1 - s * (1 - f));

  switch (i)
  {
  case 0: //red dominant
    r = v;
    g = tv;
    b = pv;
    break;
  case 1: //green dominant
    r = qv;
    g = v;
    b = pv;
    break;
  case 2: 
    r = pv;
    g = v;
    b = tv;
    break;
  case 3: //blue dominant
    r = pv;
    g = qv;
    b = v;
    break;
  case 4:
    r = tv;
    g = pv;
    b = v;
    break;
  case 5: //red dominant
    r = v;
    g = pv;
    b = qv;
    break;
  }
  //set each component to a integer value between 0 and 4096
  uint16_t red=constrain((int)4096*r,0,4096);
  uint16_t green=constrain((int)4096*g,0,4096);
  uint16_t blue=constrain((int)4096*b,0,4096);
#ifdef LOGFADE  
  // Map values into non-linear space
   rOut = pow(2,(red/rscale)) - 1;
   gOut = pow(2,(green/rscale)) - 1;
   bOut = pow(2,(blue/rscale)) - 1;
#else
  rOut = red;
  gOut = green;
  bOut = blue;
#endif
}


// Function to take care of updating all 3 color channels on a strip of LEDs
// via a PCA9685 PWM chip
void mysetpwm(Adafruit_PWMServoDriver &pwm, int RedChannel, int GreenChannel, int BlueChannel, uint16_t maxbrightnessRed, uint16_t maxbrightnessGreen, uint16_t maxbrightnessBlue) {
	pwm.setPWM(RedChannel, 0, maxbrightnessRed);
	pwm.setPWM(GreenChannel, 0, maxbrightnessGreen);
	pwm.setPWM(BlueChannel, 0, maxbrightnessBlue);
}

