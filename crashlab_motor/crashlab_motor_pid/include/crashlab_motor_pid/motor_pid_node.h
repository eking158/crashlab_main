#ifndef MOTOR_NODE_PID_H
#define MOTOR_NODE_PID_H
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

////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void Initialize(void);

//Motor_Controller
void Motor_Controller(int motor_num, bool direction, int pwm);

//Utiliy
int Limit_Function(int pwm);
void Motor_View();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct pid_param
{
  double kP=0;
  double kI=0;
  double kD=0;
  double Imax=15;
  double Dmax=15;
} pid_param;

typedef struct pid
{
  double p_out=0;
  double integrator=0;
  double derivative=0;
  double last_input=0;
  double lastderivative=0;
  double error=0;

  double output=0;
} pid;
///////////////////////////////////////////////////////////////////////////////////////////

pid_param crash_pid_param1, crash_pid_param2;  //kp, ki. kd. Imax, Dmax
pid crash_pid1, crash_pid2;  //p_out, integrator, derivative, last_input, error, output

int desire_rpm1;
int desire_rpm2;

double cur_rpm1;
double cur_rpm2;

int pwm1;
int pwm2;

///////////////////////////////////////////////////////////////////////////////////////////////

ros::Publisher pub_pwm;
ros::Subscriber sub_rpm;

///////////////////////////////////////////////////////////////////////////////////////////////

crashlab_motor_msgs::control_motor pwm_msgs;
crashlab_motor_msgs::control_motor rpm_msgs;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GetRPMCallback(const crashlab_motor_msgs::control_motor& msg);
//double PidContoller(double goal, double error, double dt, pid *pid_data, pid_param *pid_paramdata, int error_rat);

#endif // MOTOR_NODE_H
