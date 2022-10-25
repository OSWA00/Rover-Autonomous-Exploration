#include <Arduino.h>
// #include "kinematics.h"
#include "Encoder.h"
// #include "encoders/helper_functions.h"

#include <ros.h>
#include <std_msgs/String.h>

Encoder ENCODER_LEFT;
Encoder ENCODER_RIGHT;

void setup()
{
  init_encoder(ENCODER_RIGHT, 0x0, 0x1);
  init_encoder(ENCODER_LEFT, 0x2, 0x3);

  // encoders::init_encoder(&ENCODER_RIGHT);
  //   encoders::init_encoder(ENCODER_LEFT);
  //   attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT.channel_A), encoder_1_ISR_handler, RISING);
  //   encoders::init_encoder(ENCODER_RIGHT);
  //   attachInterrupt(digitalPinToInterrupt(Encoder_1.channel_A), encoder_1_ISR_handler, RISING);
  //
}

void loop()
{
}
