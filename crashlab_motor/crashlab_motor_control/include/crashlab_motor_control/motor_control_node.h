#ifndef MOTOR_NODE_H
#define MOTOR_NODE_H
#include <pigpiod_if2.h>
#include <geometry_msgs/Twist.h>
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

//Motor_Controller
void Motor_Controller(int motor_num, bool direction, int pwm);

//Utiliy
int Limit_Function(int pwm);
double RPM_Value1;
double RPM_Value2;
void RPM_Calculator();
void Motor_View();
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
  double error_ratio=1;
  double lasterror=0;

  double output=0;
} pid;
///////////////////////////////////////////////////////////////////////////////////////////

pid_param crash_pid_param1, crash_pid_param2;  //kp, ki. kd. Imax, Dmax
pid crash_pid1, crash_pid2;  //p_out, integrator, derivative, last_input, error, output

int pwm1;
int pwm2;

void PIDGain_Input(void);
double PidContoller(double goal, double curr, double cycle, pid *pid_data, pid_param *pid_paramdata);
double simplePID(double goal, double curr, double cycle, pid *pid_data, pid_param *pid_paramdata);

void Motor_Control_RPM(double rpm1, double rpm2);

///////////////////////////////////////////////////////////////////////////////////////////////
ros::Publisher pub_rpm;
ros::Subscriber sub_cmd_vel;
///////////////////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Twist vel_msgs;
crashlab_motor_msgs::control_motor rpm_msgs;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GetVelCallback(const geometry_msgs::Twist& msg);

void Motor_robot_vel(double linear_x, double angular_z);


#endif // MOTOR_NODE_H
