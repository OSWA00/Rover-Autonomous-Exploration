#include <Arduino.h>
#include <geometry_msgs/Twist.h>
#include <ros.h>
#include <std_msgs/Float32.h>

#include "Encoder.h"
#include "Motor.h"
#include "kinematics.h"
#include "velocity_controller.h"

void encoder_right_isr_handler();
void encoder_left_isr_handler();
void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel);

ros::NodeHandle NODE_HANDLE;

std_msgs::Float32 WL_MSG;
ros::Publisher WL("rover/wl", &WL_MSG);

std_msgs::Float32 WR_MSG;
ros::Publisher WR("rover/wr", &WR_MSG);

ros::Subscriber<geometry_msgs::Twist> CMD_VEL("rover/cmd_vel", &cmd_vel_callback);

double REF_RIGHT_VEL = 0.0;
double REF_LEFT_VEL = 0.0;

encoder::Encoder ENCODER_LEFT;
encoder::Encoder ENCODER_RIGHT;

motor::Motor MOTOR_RIGHT;
motor::Motor MOTOR_LEFT;

velocity_controller::ControllerParameters CONTROLLER_RIGHT;
velocity_controller::ControllerParameters CONTROLLER_LEFT;

unsigned long int TIME_CURRENT;
double TIME_DELTA;
unsigned long int TIME_LAST;

void setup() {
    NODE_HANDLE.initNode();

    NODE_HANDLE.advertise(WL);
    NODE_HANDLE.advertise(WR);

    NODE_HANDLE.subscribe(CMD_VEL);

    TIME_LAST = millis();

    encoder::init_encoder(ENCODER_RIGHT, 0x2);
    encoder::init_encoder(ENCODER_LEFT, 0xC);

    motor::init_motor(MOTOR_RIGHT, 0x5, 0x0);
    motor::init_motor(MOTOR_LEFT, 0x19, 0x1);

    velocity_controller::init_controller(CONTROLLER_RIGHT, 10, 0.01);
    velocity_controller::init_controller(CONTROLLER_LEFT, 10, 0.01);

    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT.channel_A_pin), encoder_right_isr_handler, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT.channel_A_pin), encoder_left_isr_handler, RISING);
}

void loop() {
    TIME_CURRENT = millis();
    TIME_DELTA = (TIME_CURRENT - TIME_LAST) / 0x3E8;
    TIME_LAST = TIME_CURRENT;

    double omega_left = encoder::calculate_omega(ENCODER_LEFT, TIME_DELTA);
    double velocity_left = kinematics::convert_omega_to_vel(omega_left);

    double omega_right = encoder::calculate_omega(ENCODER_RIGHT, TIME_DELTA);
    double velocity_right = kinematics::convert_omega_to_vel(omega_right);

    double u_right = velocity_controller::calculate_u(CONTROLLER_RIGHT, velocity_right, REF_RIGHT_VEL, TIME_DELTA);
    double u_left = velocity_controller::calculate_u(CONTROLLER_LEFT, velocity_left, REF_LEFT_VEL, TIME_DELTA);

    double x = M_PI;
    WL_MSG.data = (float)x;
    WL.publish(&WL_MSG);

    WR_MSG.data = (float)x;
    WR.publish(&WR_MSG);

    motor::send_pwm(MOTOR_RIGHT, u_right);
    motor::send_pwm(MOTOR_LEFT, u_left);

    NODE_HANDLE.spinOnce();
    delay(5);
}

void encoder_right_isr_handler() {
    ENCODER_RIGHT.pulses++;
}

void encoder_left_isr_handler() {
    ENCODER_LEFT.pulses++;
}

void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel) {
    double velocity_linear_x = cmd_vel.linear.x;
    double velocity_angular_z = cmd_vel.angular.z;
    REF_RIGHT_VEL = kinematics::calculate_right_velocity(velocity_linear_x, velocity_angular_z);
    REF_LEFT_VEL = kinematics::calculate_left_velocity(velocity_linear_x, velocity_angular_z);
}
