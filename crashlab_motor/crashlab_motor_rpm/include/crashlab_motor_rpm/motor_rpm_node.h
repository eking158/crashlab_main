#ifndef MOTOR_NODE_H
#define MOTOR_NODE_H
#include <pigpiod_if2.h>
#include <crashlab_motor_msgs/control_motor.h>

#define motor1_DIR 19
#define motor1_PWM 26
#define motor2_ENA 23
#define motor2_ENB 24

#define motor2_DIR 6
#define motor2_PWM 13
#define motor1_ENA 27
#define motor1_ENB 17

#define PI 3.141592

//Text_Input
void Text_Input(void);
int PWM_range;
int PWM_frequency;
int PWM_limit;
double Control_cycle;
int Acceleration_ratio;
double Wheel_radius;
double Robot_radius;
int Encoder_resolution;
double Wheel_round;
double Robot_round;

//Motor_Setup
int Motor_Setup(void);
int pinum;
int current_PWM1;
int current_PWM2;
bool current_Direction1;
bool current_Direction2;
int acceleration;

//Interrupt_Setting
void Interrupt_Setiing(void);
volatile int EncoderCounter1;
volatile int EncoderCounter2;
volatile int EncoderCounter1A;
volatile int EncoderCounter1B;
volatile int EncoderCounter2A;
volatile int EncoderCounter2B;
volatile int EncoderSpeedCounter1;
volatile int EncoderSpeedCounter2;
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
int Motor1_Encoder_Sum();
int Motor2_Encoder_Sum();
void Init_Encoder(void);
////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void Initialize(void);

//Utiliy
double RPM_Value1;
double RPM_Value2;
void RPM_Calculator();
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ros::Publisher pub_rpm;

///////////////////////////////////////////////////////////////////////////////////////////////
crashlab_motor_msgs::control_motor rpm_msgs;

#endif // MOTOR_NODE_H
