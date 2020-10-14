// LED_light_strip_PWM.h
// Luke Miller 2020
// Functions for use with a MusselBedHeater RevF board that will 
// drive PWM LED light strips (Govee brand) in a common anode arrangement


#ifndef LED_light_strip_PWM_lib_H
#define LED_light_strip_PWM_lib_H


#include <Arduino.h> // to get access to pinMode, digitalRead etc functions
#include <Wire.h> // I2C library
#include "Adafruit_PWMServoDriver.h" // https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include "RTClib.h" // https://github.com/millerlp/RTClib Real time clock library

// Onboard RGB LED settings for MusselBedHeater board
#define COMMON_ANODE true // LED used on MusselBedHeater RevF is common anode style
#define REDLED 9 // red led cathode on digital pin 9
#define GRNLED 5 // green led cathode on digital pin 5
#define BLULED 6 // blue led cathode on digital pin 6

#define LOGFADE


class ONBOARDLED{
	public:
		ONBOARDLED();
		~ONBOARDLED();

		// Functions to output color on RGB led
		void begin(); // Use default pins defined above
		// Allow user to specify default pins
		void begin(uint8_t redpin, uint8_t greenpin, uint8_t bluepin);
		void setColor(uint8_t red, uint8_t green, uint8_t blue);
		void writeHSV(int h, double s, double v, double R255);

	private:
		uint8_t m_redled;
		uint8_t m_greenled; 
		uint8_t m_blueled;
};

// Other public functions

// Function to map HSV color specification to RGB output values, also with log 
// scaling if #define LOGFADE is specified in the program
void convertHSV(int h, double s, double v, double rscale, uint16_t &rOut, uint16_t &gOut, uint16_t &bOut);

void mysetpwm(Adafruit_PWMServoDriver &pwm, int RedChannel, int GreenChannel, int BlueChannel, uint16_t maxbrightnessRed, uint16_t maxbrightnessGreen, uint16_t maxbrightnessBlue, double rscale);

#endif /* LED_light_strip_PWM_H */