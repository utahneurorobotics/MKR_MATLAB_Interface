/*
   Written By Connor Olsen

   Controls the On-Board RGB LED
   for the Arduino MKR WiFi 1010
*/

#ifndef RGB_LED_H
#define RGB_LED_H

#include <WiFiNINA.h>
#include <utility/wifi_drv.h>   // Exposes MKR1010's underlying functions to control the RGB LED
#include "Arduino.h"

//These values are for the Arduino
#define RED 25
#define GREEN 26
#define BLUE 27

class rgb_led {
  private:

  public:
    rgb_led();
    void init();
    void white(int intensity);
    void red(int intensity);
    void green(int intensity);
    void blue(int intensity);
    void yellow(int intensity);
    void magenta(int intensity);
    void cyan(int intensity);
    void off();
    void setTo(int r_val, int g_val, int b_val);
};

#endif
