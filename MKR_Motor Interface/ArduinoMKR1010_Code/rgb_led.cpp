#include "Arduino.h"
#include "rgb_led.h"

rgb_led::rgb_led() {
}

void rgb_led::init() {
  WiFiDrv::pinMode(RED, OUTPUT);
  WiFiDrv::pinMode(GREEN, OUTPUT);
  WiFiDrv::pinMode(BLUE, OUTPUT);
}

void rgb_led::white(int intensity) {
  WiFiDrv::analogWrite(RED, intensity);
  WiFiDrv::analogWrite(GREEN, intensity);
  WiFiDrv::analogWrite(BLUE, intensity);
}
void rgb_led::red(int intensity) {
  WiFiDrv::analogWrite(RED, intensity);
  WiFiDrv::analogWrite(GREEN, 0);
  WiFiDrv::analogWrite(BLUE, 0);
}
void rgb_led::green(int intensity) {
  WiFiDrv::analogWrite(RED, 0);
  WiFiDrv::analogWrite(GREEN, intensity);
  WiFiDrv::analogWrite(BLUE, 0);
}
void rgb_led::blue(int intensity) {
  WiFiDrv::analogWrite(RED, 0);
  WiFiDrv::analogWrite(GREEN, 0);
  WiFiDrv::analogWrite(BLUE, intensity);
}
void rgb_led::yellow(int intensity) {
  WiFiDrv::analogWrite(RED, intensity);
  WiFiDrv::analogWrite(GREEN, intensity);
  WiFiDrv::analogWrite(BLUE, 0);
}
void rgb_led::magenta(int intensity) {
  WiFiDrv::analogWrite(RED, intensity);
  WiFiDrv::analogWrite(GREEN, 0);
  WiFiDrv::analogWrite(BLUE, intensity);
}
void rgb_led::cyan(int intensity) {
  WiFiDrv::analogWrite(RED, 0);
  WiFiDrv::analogWrite(GREEN, intensity);
  WiFiDrv::analogWrite(BLUE, intensity);
}

void rgb_led::off() {
  WiFiDrv::analogWrite(RED, 0);
  WiFiDrv::analogWrite(GREEN, 0);
  WiFiDrv::analogWrite(BLUE, 0);
}

void rgb_led::setTo(int r_val, int g_val, int b_val) {
  WiFiDrv::analogWrite(RED, r_val);
  WiFiDrv::analogWrite(GREEN, g_val);
  WiFiDrv::analogWrite(BLUE, b_val);
}
