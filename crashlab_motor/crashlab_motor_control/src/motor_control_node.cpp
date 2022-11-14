/*
 * motor_control_node.cpp
 *
 *      Author: Chis Chun
 */
#include <ros/ros.h>
#include <crashlab_motor_control/motor_control_node.h>
#include <fstream>
#include <cmath>
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

void Text_Input(void)
{
  int i = 0;
  std::size_t found;
  std::ifstream inFile;
  inFile.open("/home/ubuntu/catkin_ws/src/crashlab_main/crashlab_settings/motor_input.txt");
  for(std::string line; std::getline(inFile,line);)
  {
      found=line.find("=");

      switch(i)
      {
      case 0: PWM_range = atof(line.substr(found+2).c_str()); break;
      case 1: PWM_frequency = atof(line.substr(found+2).c_str()); break;
      case 2: PWM_limit = atof(line.substr(found+2).c_str()); break;
      case 3: Control_cycle = atof(line.substr(found+2).c_str()); break;
      case 4: Wheel_base = atof(line.substr(found+2).c_str()); break;
      case 5: Wheel_radius = atof(line.substr(found+2).c_str()); break;
      case 6: Robot_radius = atof(line.substr(found+2).c_str()); break;
      case 7: Encoder_resolution = atof(line.substr(found+2).c_str()); break;
          //case :  = atof(line.substr(found+2).c_str()); break;
      }
      i +=1;
  }
  inFile.close();
}
int Motor_Setup(void)
{
  pinum=pigpio_start(NULL, NULL);
  
  if(pinum<0)
  {
    ROS_INFO("Setup failed");
    ROS_INFO("pinum is %d", pinum);
    return 1;
  }

  set_mode(pinum, motor1_DIR, PI_OUTPUT);
  set_mode(pinum, motor2_DIR, PI_OUTPUT);
  set_mode(pinum, motor1_PWM, PI_OUTPUT);
  set_mode(pinum, motor2_PWM, PI_OUTPUT);
  set_mode(pinum, motor1_ENA, PI_INPUT);
  set_mode(pinum, motor1_ENB, PI_INPUT);
  set_mode(pinum, motor2_ENA, PI_INPUT);
  set_mode(pinum, motor2_ENB, PI_INPUT);

  gpio_write(pinum, motor1_DIR, PI_LOW);
  gpio_write(pinum, motor2_DIR, PI_LOW);

  set_PWM_range(pinum, motor1_PWM, PWM_range);
  set_PWM_range(pinum, motor2_PWM, PWM_range);
  set_PWM_frequency(pinum, motor1_PWM, PWM_frequency);
  set_PWM_frequency(pinum, motor2_PWM, PWM_frequency);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);

  set_pull_up_down(pinum, motor1_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor1_ENB, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENB, PI_PUD_DOWN);

  current_PWM1 = 0;
  current_PWM2 = 0;

  current_Direction1 = true;
  current_Direction2 = true;

  ROS_INFO("Setup Fin");
  return 0;
}
void Interrupt_Setting(void)
{
    callback(pinum, motor1_ENA, EITHER_EDGE, Interrupt1A);
    callback(pinum, motor1_ENB, EITHER_EDGE, Interrupt1B);
    callback(pinum, motor2_ENA, EITHER_EDGE, Interrupt2A);
    callback(pinum, motor2_ENB, EITHER_EDGE, Interrupt2B);
}
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor1_DIR) == true)EncoderCounter1A ++;
  else EncoderCounter1A --;
  EncoderSpeedCounter1 ++;
}
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor1_DIR) == true)EncoderCounter1B ++;
  else EncoderCounter1B --;
  EncoderSpeedCounter1 ++;
}
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor2_DIR) == true)EncoderCounter2A --;
  else EncoderCounter2A ++;
  EncoderSpeedCounter2 ++;
}
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor2_DIR) == true)EncoderCounter2B --;
  else EncoderCounter2B ++;
  EncoderSpeedCounter2 ++;
}
int Motor1_Encoder_Sum()
{
  EncoderCounter1 = EncoderCounter1A + EncoderCounter1B;
  return EncoderCounter1;
}
int Motor2_Encoder_Sum()
{
  EncoderCounter2 = EncoderCounter2A + EncoderCounter2B;
  return EncoderCounter2;
}
void Init_Encoder(void)
{
  EncoderCounter1 = 0;
  EncoderCounter2 = 0;
  EncoderCounter1A = 0;
  EncoderCounter1B = 0;
  EncoderCounter2A = 0;
  EncoderCounter2B = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void Initialize(void)
{
  Text_Input();
  Motor_Setup();
  Init_Encoder();
  Interrupt_Setting();
  PIDGain_Input();
  
  pwm1=0;
  pwm2=0;

  Wheel_round = 2*PI*Wheel_radius;
  Robot_round = 2*PI*Robot_radius;

  ROS_INFO("PWM_range %d", PWM_range);
  ROS_INFO("PWM_frequency %d", PWM_frequency);
  ROS_INFO("PWM_limit %d", PWM_limit);
  ROS_INFO("Control_cycle %f", Control_cycle);
  ROS_INFO("Initialize Complete");

  printf("\033[2J");  
}

///////////////////////////////////////////////////////////////////////////////////////////

void Motor_Controller(int motor_num, bool direction, int pwm)
{
  int local_PWM = Limit_Function(pwm);

  if(motor_num == 1)
  {
    if(direction == true)
    {
      gpio_write(pinum, motor1_DIR, PI_LOW);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      current_Direction1 = 1;
    }
    else if(direction == false)
    {
      gpio_write(pinum, motor1_DIR, PI_HIGH);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      current_Direction1 = 0;
    }
  }
  
  else if(motor_num == 2)
  {
   if(direction == true)
   {
     gpio_write(pinum, motor2_DIR, PI_LOW);
     set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
     current_PWM2 = local_PWM;
     current_Direction2 = 1;
   }
   else if(direction == false)
   {
     gpio_write(pinum, motor2_DIR, PI_HIGH);
     set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
     current_PWM2 = local_PWM;
     current_Direction2 = 0;
   }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////

int Limit_Function(int pwm)
{
  int output;
  if (pwm > PWM_limit*2)
  {
    output = PWM_limit;
    ROS_WARN("PWM too fast!!!");
  }
  else if(pwm > PWM_limit)output = PWM_limit;
  else if(pwm < 0)
  {
	output = 0;
    ROS_WARN("trash value!!!");
  }
  else output = pwm;
  return output; 
}
void RPM_Calculator()
{
  RPM_Value1 = (EncoderSpeedCounter1*(60*Control_cycle))/(Encoder_resolution*4);
  EncoderSpeedCounter1 = 0;
  RPM_Value2 = (EncoderSpeedCounter2*(60*Control_cycle))/(Encoder_resolution*4);
  EncoderSpeedCounter2 = 0;
}
void Motor_View()
{
	RPM_Calculator();
	printf("\033[2J");
	printf("\033[1;1H");
	printf("Encoder1A : %5d  ||  Encoder2A : %5d\n", EncoderCounter1A, EncoderCounter2A);
	printf("Encoder1B : %5d  ||  Encoder2B : %5d\n", EncoderCounter1B, EncoderCounter2B);
	printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", RPM_Value1, RPM_Value2);
	printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_PWM1, current_PWM2);
	printf("DIR1 :%10.0d     ||  DIR2 :%10.0d\n", current_Direction1, current_Direction2);
	printf("kP_1 :%f     ||  kP_2 :%f\n", crash_pid_param1.kP, crash_pid_param2.kP);
	printf("kI_1 :%f     ||  kI_2 :%f\n", crash_pid_param1.kI, crash_pid_param2.kI);
	printf("kD_1 :%f     ||  kD_2 :%f\n", crash_pid_param1.kD, crash_pid_param2.kD);
	printf("P out_1 :%f     ||  P out_2 :%f\n", crash_pid1.p_out, crash_pid2.p_out);
	printf("I out_1 :%f     ||  I out_2 :%f\n", crash_pid1.integrator, crash_pid2.integrator);
	printf("D out_1 :%f     ||  D out_2 :%f\n", crash_pid1.derivative, crash_pid2.derivative);
	printf("Error_1 :%f     ||  Error_2 :%f\n", crash_pid1.error, crash_pid2.error);
	printf("\n");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PIDGain_Input(void)
{
  int i = 0;
  std::size_t found;
  std::ifstream inFile;
  inFile.open("/home/ubuntu/catkin_ws/src/crashlab_main/crashlab_settings/pid_gain.txt");
  for(std::string line; std::getline(inFile,line);)
  {
      found=line.find("=");

      switch(i)
      {
      case 0: crash_pid_param1.kP = atof(line.substr(found+2).c_str()); break;
      case 1: crash_pid_param1.kI = atof(line.substr(found+2).c_str()); break;
      case 2: crash_pid_param1.kD = atof(line.substr(found+2).c_str()); break;
      case 3: crash_pid_param2.kP = atof(line.substr(found+2).c_str()); break;
      case 4: crash_pid_param2.kI = atof(line.substr(found+2).c_str()); break;
      case 5: crash_pid_param2.kD = atof(line.substr(found+2).c_str()); break;
      case 6: crash_pid1.error_ratio = atof(line.substr(found+2).c_str()); break;
      case 7: crash_pid2.error_ratio = atof(line.substr(found+2).c_str()); break;
          //case :  = atof(line.substr(found+2).c_str()); break;
      }
      i +=1;
  }
  inFile.close();
}

//--------------------------------------------------------------------------------------
double PidContoller(double goal, double curr, double cycle, pid *pid_data, pid_param *pid_paramdata)
{
  // ROS_INFO(" goal : %f, curr: %f, dt: %f", goal,curr,dt);
  // double error = goal - curr;
  // ROS_INFO(" error : %f", error);
  double dt = 1/cycle;
  double error_rat = pid_data -> error_ratio;
  pid_data -> error = goal - curr;
  
  if (fabs(pid_data -> error) < error_rat)
    pid_data -> error = 0;

  pid_data->p_out = pid_paramdata->kP * pid_data -> error;
  double p_data = pid_data->p_out ;

  pid_data->integrator += (pid_data -> error * pid_paramdata->kI) * dt;
  double i_data = pid_data->integrator;
  i_data = constrain(i_data, -pid_paramdata->Imax, pid_paramdata->Imax);

  double filter = 15.9155e-3; // Set to  "1 / ( 2 * PI * f_cut )";
  // Examples for _filter:
  // f_cut = 10 Hz -> _filter = 15.9155e-3
  // f_cut = 15 Hz -> _filter = 10.6103e-3
  // f_cut = 20 Hz -> _filter =  7.9577e-3
  // f_cut = 25 Hz -> _filter =  6.3662e-3
  // f_cut = 30 Hz -> _filter =  5.3052e-3

  //pid_data->derivative = (goal - pid_data->last_input) / dt;
  //pid_data->derivative = pid_data->lastderivative + (dt / (filter + dt)) * (pid_data->derivative - pid_data->lastderivative);
  //pid_data->last_input = goal;
  //pid_data->lastderivative = pid_data->derivative;
  pid_data -> derivative = pid_paramdata->kD * (pid_data -> error - pid_data -> lasterror)/dt;
  double d_data = pid_data->derivative;
  //d_data = constrain(d_data, -pid_paramdata->Dmax, pid_paramdata->Dmax);

  double output = p_data + i_data + d_data;
  pid_data->output = output;
  
  pid_data -> lasterror = pid_data -> error;

  return pid_data->output;
}
//----------------------------------------------------------------------------------
double simplePID(double goal, double curr, double cycle, pid *pid_data, pid_param *pid_paramdata)
{
  double dt = 1/cycle;
  double error_rat = pid_data -> error_ratio;
  pid_data -> error = goal - curr;

  pid_data->p_out = pid_paramdata->kP * pid_data -> error;
  double p_data = pid_data->p_out ;

  pid_data->integrator += (pid_data -> error * pid_paramdata->kI) * dt;
  double i_data = pid_data->integrator;

  pid_data -> derivative = pid_paramdata->kD * (pid_data -> error - pid_data -> lasterror)/dt;
  double d_data = pid_data->derivative;

  pid_data->output = p_data + i_data + d_data;
  double output = pid_data->output;
  
  
  pid_data -> lasterror = pid_data -> error;

  return pid_data->output;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Motor_Control_RPM(double rpm1, double rpm2){  //robot motor control by robot velocity(linear x, linear y, angular z)

  pwm1 = PidContoller(rpm1, RPM_Value1, Control_cycle, &crash_pid1, &crash_pid_param1);
  //pwm2 = PidContoller(rpm2, RPM_Value2, Control_cycle, &crash_pid2, &crash_pid_param2);
  Motor_Controller(1, true, pwm1);
  //Motor_Controller(2, true, 80+pwm2);
  //Motor_Controller(1, true, 83);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void GetVelCallback(const geometry_msgs::Twist& msg){
  vel_msgs.linear.x = msg.linear.x;
  vel_msgs.angular.z = msg.angular.z;
}

void Motor_robot_vel(double linear_x, double angular_z){  
	//Wheel_radius : 바퀴 반지름(cm)
	//rpm: 분장 회전수
	//linear_x, angular_z : 속도(m/s)
	//PI : 원주율
	//rpm * (2 * pi * r) : 속도(cm/m)
	//(rpm/60) * (2 * pi* (r/100)) : 속도(m/s)
  double rpm_1 = 60 * (linear_x - angular_z*2*Robot_radius/100) / (2 * PI * Wheel_radius / 100);
  double rpm_2 = 60 * (linear_x + angular_z*2*Robot_radius/100) / (2 * PI * Wheel_radius / 100);
  
  if(rpm_1 >= 0){
    pwm1 = PidContoller(rpm_1, RPM_Value1, Control_cycle, &crash_pid1, &crash_pid_param1);
    Motor_Controller(1, true, pwm1);
  }
  else{
    pwm1 = PidContoller(-rpm_1, RPM_Value1, Control_cycle, &crash_pid1, &crash_pid_param1);
    Motor_Controller(1, false, pwm1);
  }
  
  if(rpm_2 >= 0){
    pwm2 = PidContoller(rpm_2, RPM_Value2, Control_cycle, &crash_pid2, &crash_pid_param2);
    Motor_Controller(2, false, pwm2);
  }
  else{
    pwm2 = PidContoller(-rpm_2, RPM_Value2, Control_cycle, &crash_pid2, &crash_pid_param2);
    Motor_Controller(2, true, pwm2);
  }
  
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_control_node");
  ros::NodeHandle nh;
  Initialize();
  ros::Rate loop_rate(Control_cycle);
  
  sub_cmd_vel = nh.subscribe("crashlab/cmd_vel", 10, GetVelCallback);
  pub_rpm = nh.advertise<crashlab_motor_msgs::control_motor>("crashlab/rpm", 10);
  
  while(ros::ok())
  {
    rpm_msgs.motor1 = RPM_Value1;
    rpm_msgs.motor2 = RPM_Value2;
    pub_rpm.publish(rpm_msgs);
    
    Motor_Control_RPM(80, 80);
    Motor_View();
    //Motor_Controller(1, true, 83);
    ros::spinOnce();
    loop_rate.sleep();
  }
  Motor_Controller(1, true, 0);
  Motor_Controller(2, true, 0);
  return 0;
}
