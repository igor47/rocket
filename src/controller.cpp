// Main controller code running on the Arduino built into each penguin
// (c) Moomers, Inc. 2012

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#include "config.h"

/*********** Pin Assignments ***********************/
//joystick pins
const byte VerticalStickPin = 1;
const byte HorizontalStickPin = 0;

// these aren't used in the code, they're just here
// as a reminder that those pins are not available
const byte ServerRXPin = 0;
const byte ServerTXPin = 1;

// buttons on the shield
const byte ButtonEPin = 3;
const byte ButtonDPin = 4;
const byte ButtonCPin = 5;
const byte ButtonBPin = 6;
const byte ButtonAPin = 7;
const byte ButtonFPin = 8;
const byte ButtonGPin = 9;

// talking to the Sabertooth driver
const byte DriverTXPin = 10;
const byte DriverRXPin = 11;

// just show that things are going
const byte RunLEDPin = 13;

/*************** Globals *********************/

// Sabertooth serial interface is unidirectional, so only TX is really needed
SoftwareSerial sabertoothSerial(DriverRXPin, DriverTXPin);

/*************** Data Types  *********************/

static struct State {
  State() :
    loop(0),

    // set by read_sticks
    vertical(0),
    horizontal(0),

    // set by set_desired_speeds
    desired_left(0),
    desired_right(0),
    desired_speed(0),
    desired_side(0)
  { }
  unsigned int loop;
  unsigned int vertical;
  unsigned int horizontal;
  int desired_left;
  int desired_right;
  int desired_speed;
  int desired_side;
} state;

static struct Drive {
  Drive() : forward(1),
    backward(-1),
    max_speed(100),
    h_center(450),
    v_center(450),
    h_gap(50),
    v_gap(50),
    h_range(400),
    v_range(400) { }
  short forward;
  short backward;
  short max_speed;
  short h_center;
  short v_center;
  short h_gap;
  short v_gap;
  short h_range;
  short v_range;
} drive;

/*************** Prototypes  *********************/

void send_velocity_to_computer();
void send_velocity_to_sabertooth();
void left_encoder_interrupt();
void right_encoder_interrupt();
void read_sticks();
void toggle_led();
void set_desired_speeds();

// begin code
void setup()
{
  // initialize the serial communication with the server
  Serial.begin(9600);

  // initialize communication with the sabertooth motor controller
  sabertoothSerial.begin(19200);
  sabertoothSerial.write(uint8_t(0));

  // initialize the led pin
  pinMode(RunLEDPin, OUTPUT);

  // turn on the warn led to indicate calibration
  int h_reads = 0,
      v_reads = 0,
      reads = 16;

  for (int i =0; i < reads; i++) {
    read_sticks();
    h_reads += state.horizontal;
    v_reads += state.vertical;
    toggle_led();
  }

  drive.h_center = h_reads / reads;
  drive.v_center = v_reads / reads;
}

void read_sticks() {
  state.vertical = analogRead(VerticalStickPin);
  state.horizontal = analogRead(HorizontalStickPin);
}

inline int clamp(int value, int min, int max)
{
  if (value < min) {
    return min;
  } else if (value > max) {
    return max;
  }
  return value;
}

void set_desired_speeds() {
  short direction = drive.forward;

  int speed = clamp(
          state.vertical,
          drive.v_center - drive.v_gap - drive.v_range,
          drive.v_center + drive.v_gap + drive.v_range);

  // speed is from 0 to 100
  if (speed > drive.v_center + drive.v_gap) {
    speed = (speed - (drive.v_center + drive.v_gap)) / 4;
    direction = drive.backward;
  } else if (speed < drive.v_center - drive.v_gap) {
    speed = (drive.v_center - drive.v_gap - speed) / 4;
  } else {
    speed = 0;
  }

  // side is between -100 and 100
  // -100 is all the way on the right
  int side = clamp(
          state.horizontal,
          drive.h_center - drive.h_gap - drive.h_range,
          drive.h_center + drive.h_gap + drive.h_range);

  if (side > drive.h_center + drive.h_gap) {
    side = (side - (drive.h_center + drive.h_gap)) / 4;
  } else if (side < (drive.h_center - drive.h_gap)) {
    side = (side - (drive.h_center - drive.h_gap)) / 4;
  } else {
    side = 0;
  }

   // biggest diff between left and right is at most 200
  int right = direction * (speed - side);
  int left = direction * (speed + side);

  // no motor goes faster than speed
  // now diff between left and right is at most 2 * speed
  right = clamp(right, -speed, speed);
  left = clamp(left, -speed, speed);

  // proportional to the max speed we want sabertooth to go
  left = drive.max_speed * left / 200;
  right = drive.max_speed * right / 200;

  // save the state
  state.desired_left = left;
  state.desired_right = right;
  state.desired_speed = speed;
  state.desired_side = side;
}

void loop()
{
  // increment loop counter and blink the run led
  toggle_led();

  // deal with actual driving
  read_sticks();
  set_desired_speeds();
  send_velocity_to_sabertooth();

  // talking to the computer takes a while
  if (state.loop % 16 == 0) {
    send_velocity_to_computer();
  }
}

// interrupt functions for logging encoder events
volatile int LEFT_PULSES = 0;
void left_encoder_interrupt() {
  LEFT_PULSES++;
}

volatile int RIGHT_PULSES = 0;
void right_encoder_interrupt() {
  RIGHT_PULSES++;
}

// code for talking to the sabertooth
void send_velocity_to_sabertooth()
{
  int left = clamp(state.desired_left, -63, 63);
  int right = clamp(state.desired_right, -63, 63);

  if (left == 0 && right == 0) {
    sabertoothSerial.write(uint8_t(0));
  } else {
    sabertoothSerial.write(uint8_t(64 + left));
    sabertoothSerial.write(uint8_t(192 + right));
  }
}

void send_velocity_to_computer() {
  Serial.print("V/H raw:");
  Serial.print(state.vertical);
  Serial.print("/");
  Serial.print(state.horizontal);
  Serial.print(";");

  Serial.print("-- h/v center:");
  Serial.print(drive.h_center);
  Serial.print("/");
  Serial.print(drive.v_center);
  Serial.print(";");


  Serial.print("-- speed/side:");
  Serial.print(state.desired_speed);
  Serial.print("/");
  Serial.print(state.desired_side);
  Serial.print(";");


  Serial.print(" -- left/right:");
  Serial.print(state.desired_left);
  Serial.print("/");
  Serial.print(state.desired_right);
  Serial.print("\r\n");
}

void toggle_led()
{
  state.loop++;

  if(state.loop % 2 == 0) {
    digitalWrite(RunLEDPin, LOW);
  } else {
    digitalWrite(RunLEDPin, HIGH);
  }
}
