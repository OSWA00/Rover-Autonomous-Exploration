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

ros::Subscriber<geometry_msgs::Twist> CMD_VEL("rover/cmd_vel",
                                              &cmd_vel_callback);

encoder::Encoder ENCODER_LEFT;
encoder::Encoder ENCODER_RIGHT;

motor::Motor MOTOR_RIGHT;
motor::Motor MOTOR_LEFT;

velocity_controller::ControllerParameters CONTROLLER_RIGHT;
velocity_controller::ControllerParameters CONTROLLER_LEFT;

double TIME_CURRENT;
double TIME_DELTA;
double TIME_LAST;

void setup() {
    NODE_HANDLE.initNode();

    NODE_HANDLE.advertise(WL);
    NODE_HANDLE.advertise(WR);

    NODE_HANDLE.subscribe(CMD_VEL);

    TIME_LAST = micros();

    encoder::init_encoder(ENCODER_RIGHT, 0x2);
    encoder::init_encoder(ENCODER_LEFT, 0xC);

    motor::init_motor(MOTOR_RIGHT, 0x5, 0x0);
    motor::init_motor(MOTOR_LEFT, 0x19, 0x1);

    velocity_controller::init_controller(CONTROLLER_RIGHT, 10, 0.01);
    velocity_controller::init_controller(CONTROLLER_LEFT, 10, 0.01);

    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT.channel_A_pin),
                    encoder_right_isr_handler, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT.channel_A_pin),
                    encoder_left_isr_handler, RISING);
}

void loop() {
    TIME_CURRENT = micros();
    TIME_DELTA = (TIME_CURRENT - TIME_LAST) / 1000000;
    TIME_LAST = TIME_CURRENT;

    double omega_left = encoder::calculate_omega(ENCODER_LEFT, TIME_DELTA);
    double velocity_left = kinematics::convert_omega_to_vel(omega_left);

    double omega_right = encoder::calculate_omega(ENCODER_RIGHT, TIME_DELTA);
    double velocity_right = kinematics::convert_omega_to_vel(omega_right);

    double u_right = velocity_controller::calculate_u(
        CONTROLLER_RIGHT, velocity_right, CONTROLLER_RIGHT.velocity_reference,
        TIME_DELTA);
    double u_left = velocity_controller::calculate_u(
        CONTROLLER_LEFT, velocity_left, CONTROLLER_LEFT.velocity_reference,
        TIME_DELTA);

    WL_MSG.data = omega_left;
    WL.publish(&WL_MSG);

    WR_MSG.data = omega_right;
    WR.publish(&WR_MSG);

    motor::send_pwm(MOTOR_RIGHT, u_right);
    motor::send_pwm(MOTOR_LEFT, u_left);

    NODE_HANDLE.spinOnce();
    delay(5);
}

void encoder_right_isr_handler() { ENCODER_RIGHT.pulses++; }

void encoder_left_isr_handler() { ENCODER_LEFT.pulses++; }

void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel) {
    double velocity_linear_x = cmd_vel.linear.x;
    double velocity_angular_z = cmd_vel.angular.z;
    CONTROLLER_RIGHT.velocity_reference = kinematics::calculate_right_velocity(
        velocity_linear_x, velocity_angular_z);
    CONTROLLER_LEFT.velocity_reference = kinematics::calculate_left_velocity(
        velocity_linear_x, velocity_angular_z);
}
