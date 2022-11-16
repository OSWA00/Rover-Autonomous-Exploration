#include <Arduino.h>
#include "Encoder.h"
#include "kinematics.h"
#include "Motor.h"
#include "vel_controller.h"

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;
std_msgs::Float64 wl_msg;
std_msgs::Float64 wr_msg;

ros::Publisher wr("rover/wr", &wr_msg);
ros::Publisher wl("rover/wl", &wl_msg);

double REF_RIGHT_VEL = 0.0;
double REF_LEFT_VEL = 0.0;

encoder::Encoder ENCODER_LEFT;
encoder::Encoder ENCODER_RIGHT;

motor::Motor MOTOR_RIGHT;
motor::Motor MOTOR_LEFT;

Vel_controller CONTROLLER_MOTOR_RIGHT;
Vel_controller CONTROLLER_MOTOR_LEFT;

unsigned long int TIME_CURRENT;
double TIME_DELTA;
unsigned long int TIME_LAST;

void encoder_right_isr_handler();
void encoder_left_isr_handler();
void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel);

ros::Subscriber<geometry_msgs::Twist> cmd_vel("rover/cmd_vel", &cmd_vel_callback);

void setup()
{
    nh.initNode();
    nh.advertise(wl);
    nh.advertise(wr);
    nh.subscribe(cmd_vel);

    TIME_LAST = millis();

    encoder::init_encoder(ENCODER_RIGHT, 0x2);
    encoder::init_encoder(ENCODER_LEFT, 0xC);

    motor::init_motor(MOTOR_RIGHT, 0x5, 0x0);
    motor::init_motor(MOTOR_LEFT, 0x19, 0x1);

    init_controller(CONTROLLER_MOTOR_RIGHT, 10, 0.01);
    init_controller(CONTROLLER_MOTOR_LEFT, 10, 0.01);

    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT.channel_A_pin), encoder_right_isr_handler, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT.channel_A_pin), encoder_left_isr_handler, RISING);
}

void loop()
{
    TIME_CURRENT = millis();
    TIME_DELTA = (TIME_CURRENT - TIME_LAST) / 0x3E8;
    TIME_LAST = TIME_CURRENT;

    double omega_left = encoder::calculate_omega(ENCODER_LEFT, TIME_DELTA);
    float vel_left = convert_omega_to_vel(omega_left);

    double omega_right = encoder::calculate_omega(ENCODER_RIGHT, TIME_DELTA);
    float vel_right = convert_omega_to_vel(omega_right);

    float u_right = calculate_u(CONTROLLER_MOTOR_RIGHT, vel_right, REF_RIGHT_VEL, TIME_DELTA);
    float u_left = calculate_u(CONTROLLER_MOTOR_LEFT, vel_left, REF_LEFT_VEL, TIME_DELTA);

    wl_msg.data = omega_left;
    wl.publish(&wl_msg);

    wr_msg.data = omega_right;
    wr.publish(&wr_msg);

    motor::send_pwm(MOTOR_RIGHT, u_right);
    motor::send_pwm(MOTOR_LEFT, u_left);

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