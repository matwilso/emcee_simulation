#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>

// JoyTeleop takes a joy message and converts it into robot commands.
// The joysticks are converted to a Twist message that is sent to a
// diff drive controller.  The buttons are converted to commands to
// position controllers for arm, bucket, sled.
class JoyTeleop
{
public:
  JoyTeleop();


private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void armStateCallback(const control_msgs::JointControllerState::ConstPtr& arm_state);
  void bucketStateCallback(const control_msgs::JointControllerState::ConstPtr& bucket_state);
  void sledStateCallback(const control_msgs::JointControllerState::ConstPtr& sled_state);
  void turnOnDebugging(void);

  ros::NodeHandle nh;

  double max_velocity, max_angular; //limits value we can send on Twist message
  //#HACK that allows you to set how fast the positions moved when being controlled by joystick
  double arm_max_vel;
  double bucket_max_vel;
  double sled_max_vel;
  int left_hoz, left_ver, right_hoz, right_ver; //joystick axes
  int lt, rt; //button axes
  int y_button, a_button, lb, rb; //buttons
  // positions
  double arm_min; double arm_max;
  double bucket_min; double bucket_max;
  double sled_min; double sled_max;
  // Set points from the PID controller. Tells us where the things are commanded to go
  double arm_set_point;
  double bucket_set_point;
  double sled_set_point;
  // Previous position of joints
  double prev_arm;
  double prev_bucket;
  double prev_sled;
  // ROS stuff
  ros::Subscriber joy_sub; //joystick
  ros::Publisher vel_pub; //diff drive velocity

// Subscibe to the states of the position controllers to see where they are,
// and send a differential command based on that. This is only used because we are teleop-ing.
  ros::Subscriber arm_state_sub; //arm state
  ros::Publisher arm_pub; //arm command pub
  ros::Subscriber bucket_state_sub; //bucket state
  ros::Publisher bucket_pub; //bucket command pub
  ros::Subscriber sled_state_sub; //sled state
  ros::Publisher sled_pub; //sled command pub
};


JoyTeleop::JoyTeleop() {
    int debug;
    nh.getParam("debug", debug);
    if (debug == 1) {
        ROS_INFO("joy_teleop: DEBUGGING ON");
        turnOnDebugging();
    }

    //these determine how quickly joy commands change the values
    //TODO: fine tune these more, in conjunction with the joint vals in the urdf
    arm_max_vel = 0.005;
    bucket_max_vel = 0.01;
    sled_max_vel = 0.0008;

    // positions
    arm_min = -0.253177;
    arm_max = 0.309183;

    bucket_min = -0.4;
    bucket_max = 1.570795;

    sled_min = -0.204975;
    sled_max = 0.103375;

    //get parameters from the Parameter Server.
    //These are loaded from the emcee_control.yaml by the control.launch file.
    nh.getParam("/emcee_velocity_controller/linear/x/max_velocity", max_velocity);
    nh.getParam("/emcee_velocity_controller/angular/z/max_velocity", max_angular);

    //joystick parameters. different for xbox and ps3
    nh.getParam("/joy_config/axes/left_hoz", left_hoz);
    nh.getParam("/joy_config/axes/left_ver", left_ver);
    nh.getParam("/joy_config/axes/right_hoz", right_hoz);
    nh.getParam("/joy_config/axes/right_ver", right_ver);
    nh.getParam("/joy_config/axes/rt", rt);
    nh.getParam("/joy_config/axes/lt", lt);

    nh.getParam("/joy_config/buttons/a", a_button);
    nh.getParam("/joy_config/buttons/y", y_button);
    nh.getParam("/joy_config/buttons/lb", lb);
    nh.getParam("/joy_config/buttons/rb", rb);

    //pubs and subs
    joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &JoyTeleop::joyCallback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    arm_pub = nh.advertise<std_msgs::Float64>("/emcee_arm_position_controller/command", 10);
    bucket_pub = nh.advertise<std_msgs::Float64>("/emcee_bucket_position_controller/command", 10);
    sled_pub = nh.advertise<std_msgs::Float64>("/emcee_sled_position_controller/command", 10);

    arm_state_sub = nh.subscribe<control_msgs::JointControllerState>("/emcee_arm_position_controller/state", 10, &JoyTeleop::armStateCallback, this);
    bucket_state_sub = nh.subscribe<control_msgs::JointControllerState>("/emcee_bucket_position_controller/state", 10, &JoyTeleop::bucketStateCallback, this);
    sled_state_sub = nh.subscribe<control_msgs::JointControllerState>("/emcee_sled_position_controller/state", 10, &JoyTeleop::sledStateCallback, this);

    //wait for most recent set_point
    ros::topic::waitForMessage<control_msgs::JointControllerState>("/emcee_arm_position_controller/state");
    ros::topic::waitForMessage<control_msgs::JointControllerState>("/emcee_bucket_position_controller/state");
    ros::topic::waitForMessage<control_msgs::JointControllerState>("/emcee_sled_position_controller/state");

    prev_arm = arm_set_point;
    prev_bucket = bucket_set_point;
    prev_sled = sled_set_point;

    ROS_DEBUG_STREAM("armStateCallback: "<< prev_arm);
    ROS_DEBUG_STREAM("bucketStateCallback: "<< prev_bucket);
    ROS_DEBUG_STREAM("sledStateCallback: "<< prev_sled);
}

void JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // velocity message expected by ros_control
  geometry_msgs::Twist vel;

  // Diff drive control (classic URMP)
  vel.linear.x = (max_velocity/2) * (joy->axes[right_ver] + joy->axes[left_ver]);
  vel.angular.z = (max_angular/2) * (joy->axes[right_ver] - joy->axes[left_ver]);

// Single stick drive (seems harder to control)
  // vel.linear.x = max_velocity * joy->axes[left_ver];
  // vel.angular.z = max_angular * joy->axes[left_hoz];

  // Position controllers
  std_msgs::Float64 arm_command;
  std_msgs::Float64 bucket_command;
  std_msgs::Float64 sled_command;

  float arm_diff = arm_max_vel * (((joy->axes[rt]+1)/2.) - ((joy->axes[lt]+1)/2.));
  float sled_diff = (joy->buttons[y_button] - joy->buttons[a_button]) * sled_max_vel;
  float bucket_diff = (joy->buttons[rb] - joy->buttons[lb]) * bucket_max_vel;


  //Add the new command from the joystick and make sure it is in the moveable range
  arm_command.data = arm_diff + prev_arm;
  arm_command.data = std::max(arm_min, arm_command.data);
  arm_command.data = std::min(arm_max, arm_command.data);
  prev_arm = arm_command.data;

  bucket_command.data = bucket_diff + prev_bucket;
  bucket_command.data = std::max(bucket_min, bucket_command.data);
  bucket_command.data = std::min(bucket_max, bucket_command.data);
  prev_bucket = bucket_command.data;

  sled_command.data = sled_diff + prev_sled;
  sled_command.data = std::max(sled_min, sled_command.data);
  sled_command.data = std::min(sled_max, sled_command.data);
  prev_sled = sled_command.data;


  ROS_DEBUG_STREAM("arm set_point = " << arm_set_point);
  ROS_DEBUG_STREAM("bucket set_point = " << bucket_set_point);
  ROS_DEBUG_STREAM("sled set_point = " << sled_set_point);

  ROS_DEBUG_STREAM("arm_command = " << arm_command.data);
  ROS_DEBUG_STREAM("bucket_command = " << bucket_command.data);
  ROS_DEBUG_STREAM("sled_command = " << sled_command.data);



  arm_pub.publish(arm_command);
  bucket_pub.publish(bucket_command);
  sled_pub.publish(sled_command);
  vel_pub.publish(vel);
}

void JoyTeleop::armStateCallback(const control_msgs::JointControllerState::ConstPtr& arm_state) {
    arm_set_point = arm_state->set_point;
}

void JoyTeleop::bucketStateCallback(const control_msgs::JointControllerState::ConstPtr& bucket_state) {
    bucket_set_point = bucket_state->set_point;
}

void JoyTeleop::sledStateCallback(const control_msgs::JointControllerState::ConstPtr& sled_state) {
    sled_set_point = sled_state->set_point;
}

//turn on debugging in the terminal. I don't think this is the best way.
//If somebody finds a better way to do it, that would be cool
void JoyTeleop::turnOnDebugging() {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  JoyTeleop joy_teleop;

  ros::spin();
}
