# crashlab_main

<변수 단위>

-선속도: m/s

-각속도: rad/s

-Wheel_radius(바퀴 반지름): cm

-Wheel_base(바퀴 사이의 거리): cm

-Robot_radius(로봇 반지름): cm

////////////////////////////////////////////////////////////////////////////////////

<패키지 설명>

-crashlab_motor_control: cmd_vel(속도)값을 rpm값으로 변환하고, pid제어를 통해 최종적으로 모터를 제어해주는 패키지

-crashlab_get_odometry: rpm값을 받아와서 odom을 계산해주는 패키지

-crashlab_teleop_key: 키보드 입력을 받아와서 속도값을 만들고, 이를 crashlab/cmd_vel 토픽으로 pub해주는 패키지

-crashlab_motor_msgs: 모터의 방향과 rpm을 저장하는 custom message

-crashlab_motor_pid: 모터 pid 제어 테스트 패키지(사용 x)

-crashlab_motor_rpm: 모터 rpm 측정 테스트 패키지(사용 x)


//////////////////////////////////////////////////////////////////////////////////////
<노드 설명>

-motor_control_node: cmd_vel(속도)값을 rpm값으로 변환하고, pid제어를 통해 최종적으로 모터를 제어해주는 노드

-get_odometry_node: rpm값을 받아와서 odom을 계산해주는 노드

-teleop_key_node: 키보드 입력을 받아와서 속도값을 만들고, 이를 crashlab/cmd_vel 토픽으로 pub해주는 노드

-motor_pid_node: 모터 pid 제어 테스트 노드(사용 x)

-motor_rpm_node: 모터 rpm 측정 테스트 노드(사용 x)

////////////////////////////////////////////////////////////////////////////////////////

<토픽 설명>

-/crashlab/rpm: rpm값을 주고받는 토픽

-/crashlab/cmd_vel: 모터 제어를 위한 선속도, 각속도값을 주고받는 토픽

-/odom: odom값을 반환하는 토픽
