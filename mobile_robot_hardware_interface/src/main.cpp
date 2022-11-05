#include <Arduino.h>
#include "Encoder.h"
#include "kinematics.h"
#include "Motor.h"
#include "vel_controller.h"

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;
std_msgs::Float32 wl_msg;
std_msgs::Float32 wr_msg;

ros::Publisher wr("wr", &wr_msg);
ros::Publisher wl("wl", &wl_msg);

float REF_RIGHT_VEL = 0.0;
float REF_LEFT_VEL = 0.0;

Encoder ENCODER_LEFT;
Encoder ENCODER_RIGHT;
Motor MOTOR_RIGHT;
Motor MOTOR_LEFT;
Vel_controller CONTROLLER_MOTOR_RIGHT;
Vel_controller CONTROLLER_MOTOR_LEFT;

unsigned long int TIME_CURRENT;
float TIME_DELTA;
unsigned long int TIME_LAST;

void encoder_right_isr_handler();
void encoder_left_isr_handler();
void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel);

ros::Subscriber<geometry_msgs::Twist> cmd_vel("/cmd_vel", &cmd_vel_callback);

void setup()
{
    Serial.begin(115200);

    nh.initNode();
    nh.advertise(wl);
    nh.advertise(wr);
    nh.subscribe(cmd_vel);

    TIME_LAST = millis();

    init_encoder(ENCODER_RIGHT, 0xC);
    init_encoder(ENCODER_LEFT, 0x2);

    init_motor(MOTOR_RIGHT, 0x19, 0x0);
    init_motor(MOTOR_LEFT, 0x5, 0x1);

    init_controller(CONTROLLER_MOTOR_RIGHT, 10, 0.01);
    init_controller(CONTROLLER_MOTOR_LEFT, 10, 0.01);

    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT.channel_A_pin), encoder_right_isr_handler, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT.channel_A_pin), encoder_left_isr_handler, RISING);
}

void loop()
{
    TIME_CURRENT = millis();
    TIME_DELTA = (TIME_CURRENT - TIME_LAST) * 0.001;
    TIME_LAST = TIME_CURRENT;

    float omega_left = calculate_omega(ENCODER_LEFT, TIME_DELTA);
    float vel_left = convert_omega_to_vel(omega_left);

    float omega_right = calculate_omega(ENCODER_RIGHT, TIME_DELTA);
    float vel_right = convert_omega_to_vel(omega_right);

    float u_right = calculate_u(CONTROLLER_MOTOR_RIGHT, vel_right, REF_RIGHT_VEL, TIME_DELTA);
    float u_left = calculate_u(CONTROLLER_MOTOR_LEFT, vel_left, REF_LEFT_VEL, TIME_DELTA);

    wl_msg.data = omega_left;
    wl.publish(&wl_msg);

    wr_msg.data = omega_right;
    wr.publish(&wr_msg);

    send_pwm(MOTOR_RIGHT, u_right);
    send_pwm(MOTOR_LEFT, u_left);

    nh.spinOnce();
    delay(5);
}

void encoder_right_isr_handler()
{
    ENCODER_RIGHT.pulses++;
}

void encoder_left_isr_handler()
{
    ENCODER_LEFT.pulses++;
}

void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel)
{
    float v = cmd_vel.linear.x;
    float omega = cmd_vel.angular.z;
    REF_RIGHT_VEL = calculate_right_velocity(v, omega);
    REF_LEFT_VEL = calculate_left_velocity(v, omega);
}