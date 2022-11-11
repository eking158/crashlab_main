/*
 * motor_control_node.cpp
 *
 *      Author: Chis Chun
 */
#include <ros/ros.h>
#include <crashlab_motor_pid/motor_pid_node.h>
#include <fstream>
#include <cmath>
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

void Text_Input(void)
{
  int i = 0;
  std::size_t found;
  std::ifstream inFile;
  inFile.open("/home/ubuntu/catkin_ws/src/crashlab_main/crashlab_motor/crashlab_motor_control/motor_input.txt");
  for(std::string line; std::getline(inFile,line);)
  {
      found=line.find("=");

      switch(i)
      {
      case 0: PWM_range = atof(line.substr(found+2).c_str()); break;
      case 1: PWM_frequency = atof(line.substr(found+2).c_str()); break;
      case 2: PWM_limit = atof(line.substr(found+2).c_str()); break;
      case 3: Control_cycle = atof(line.substr(found+2).c_str()); break;
      case 4: Acceleration_ratio = atof(line.substr(found+2).c_str()); break;
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

  acceleration = PWM_limit/(Acceleration_ratio);

  ROS_INFO("Setup Fin");
  return 0;
}

void Initialize(void)
{
  Text_Input();
  Motor_Setup();
  
  crash_pid_param1.kP=1;
  crash_pid_param1.kI=0.3;
  crash_pid_param1.kD=0.15;
  //////////////////////////////
  //crash_pid_param1.kP=3;
  //crash_pid_param1.kI=0.04;
  //crash_pid_param1.kD=20;
  ////////////////////////////////////////
  //crash_pid_param2.kP=10;
  //crash_pid_param2.kI=1;
  //crash_pid_param2.kD=10;

  Wheel_round = 2*PI*Wheel_radius;
  Robot_round = 2*PI*Robot_radius;

  switch_direction = true;
  Theta_Distance_Flag = 0;
  
  pwm_msgs.motor1 = 0;
  pwm_msgs.motor2 = 0;

  ROS_INFO("PWM_range %d", PWM_range);
  ROS_INFO("PWM_frequency %d", PWM_frequency);
  ROS_INFO("PWM_limit %d", PWM_limit);
  ROS_INFO("Control_cycle %f", Control_cycle);
  ROS_INFO("Acceleration_ratio %d", Acceleration_ratio);
  ROS_INFO("Initialize Complete");

  printf("\033[2J");  
}

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

void Motor_View()
{
	printf("\033[2J");
	printf("\033[1;1H");
	printf("Encoder1A : %5d  ||  Encoder2A : %5d\n", EncoderCounter1A, EncoderCounter2A);
	printf("Encoder1B : %5d  ||  Encoder2B : %5d\n", EncoderCounter1B, EncoderCounter2B);
	printf("RPM1 : %f    ||  RPM2 : %f\n", cur_rpm1, cur_rpm2);
	printf("Error1 : %f    ||  Error2 : %f\n", crash_pid1.error, crash_pid2.error);
	printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_PWM1, current_PWM2);
	printf("DIR1 :%10.0d     ||  DIR2 :%10.0d\n", current_Direction1, current_Direction2);
	printf("Acc  :%10.0d\n", acceleration);
	printf("\n");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double PidContoller(double goal, double curr, double dt, pid *pid_data, pid_param *pid_paramdata, int error_rat)
{
  // ROS_INFO(" goal : %f, curr: %f, dt: %f", goal,curr,dt);
  // double error = goal - curr;
  // ROS_INFO(" error : %f", error);
  pid_data -> error = goal - curr;
  if (fabs(pid_data -> error) < error_rat)
    pid_data -> error = 0;

  pid_data->p_out = pid_paramdata->kP * pid_data -> error;
  double p_data = pid_data->p_out ;

  pid_data->integrator += (pid_data -> error * pid_paramdata->kI) * dt;
  pid_data->integrator = constrain(pid_data->integrator, -pid_paramdata->Imax, pid_paramdata->Imax);
  double i_data = pid_data->integrator;

  double filter = 15.9155e-3; // Set to  "1 / ( 2 * PI * f_cut )";
  // Examples for _filter:
  // f_cut = 10 Hz -> _filter = 15.9155e-3
  // f_cut = 15 Hz -> _filter = 10.6103e-3
  // f_cut = 20 Hz -> _filter =  7.9577e-3
  // f_cut = 25 Hz -> _filter =  6.3662e-3
  // f_cut = 30 Hz -> _filter =  5.3052e-3

  pid_data->derivative = (goal - pid_data->last_input) / dt;
  pid_data->derivative = pid_data->lastderivative + (dt / (filter + dt)) * (pid_data->derivative - pid_data->lastderivative);
  pid_data->last_input = goal;
  pid_data->lastderivative = pid_data->derivative;
  double d_data = pid_paramdata->kD * pid_data->derivative;
  d_data = constrain(d_data, -pid_paramdata->Dmax, pid_paramdata->Dmax);

  double output = p_data + i_data + d_data;
  pid_data->output = output;

  return pid_data->output;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////

void GetRPMCallback(const crashlab_motor_msgs::control_motor& msg){
  cur_rpm1 = msg.motor1;
  cur_rpm2 = msg.motor2;
}

void Motor_robot_vel(double rpm1, double rpm2){  //robot motor control by robot velocity(linear x, linear y, angular z)

  pwm_msgs.motor1 = PidContoller(80+rpm1, cur_rpm1, 0.1, &crash_pid1, &crash_pid_param1, 0.1);
  pwm_msgs.motor2 = PidContoller(80+rpm2, cur_rpm2, 0.1, &crash_pid2, &crash_pid_param2, 1);
  Motor_Controller(1, true, pwm_msgs.motor1);
  //Motor_Controller(2, true, 80+pwm_msgs.motor2);
  //Motor_Controller(1, true, 83);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_pid_node");
  ros::NodeHandle nh;
  Initialize();
  ros::Rate loop_rate(Control_cycle);
  //ros::Rate loop_rate(50);
  
  sub_rpm = nh.subscribe("crashlab/rpm", 10, GetRPMCallback);
 
  
  while(ros::ok())
  {
    Motor_robot_vel(60, 50);
    //Motor_Controller(1, true, 100);
    Motor_View();
    loop_rate.sleep();
    ros::spinOnce();
  }
  Motor_Controller(1, true, 0);
  Motor_Controller(2, true, 0);
  return 0;
}
