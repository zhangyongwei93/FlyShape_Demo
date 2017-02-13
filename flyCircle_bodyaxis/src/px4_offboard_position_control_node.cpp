#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <array>
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <keyboard/Key.h>
#include <math.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

geometry_msgs::PoseStamped oriPos,ps,currentPos,nextPos,initCirclePos;
geometry_msgs::PoseStamped uavCurrentLocalPose,uavCurrentViconPose;
geometry_msgs::TransformStamped vicon_currentPos;
geometry_msgs::PoseStamped vicon_mocaptf;

ros::Publisher setPositionPublisher;
ros::Publisher currentViconPositionPublisher;

double tmpRoll,tmpPitch,tmpYaw;
float pos_value;

int angle,angleStep;
int radius;

bool flag_offboard_mode = false;
bool isFlyCircle,hasInitFlyCircle;
bool isLockLocalPos;
bool hasSet;

// 判断飞行模式
mavros_msgs::State current_state;
void stateReceived(const mavros_msgs::State::ConstPtr& msg) 
{
    static bool init_flag = false;
    current_state = *msg;

    if (current_state.mode == "OFFBOARD")
    {
        if(!init_flag)
        {
            ROS_INFO_STREAM("Offboard Mode"); 
            init_flag = true;         
        }

        flag_offboard_mode = true;
    }
    else
    {
        flag_offboard_mode = false;
        init_flag = false;  
    }
}

// Vicon位姿数据转换为Mavros位姿数据(转换原因：消息类型不匹配)
void PosetopicTransport(geometry_msgs::TransformStamped& viconPos, geometry_msgs::PoseStamped& rosPos)
{
    rosPos.pose.position.x = viconPos.transform.translation.x;
    rosPos.pose.position.y = viconPos.transform.translation.y;
    rosPos.pose.position.z = viconPos.transform.translation.z;
    rosPos.pose.orientation.x = viconPos.transform.rotation.x;
    rosPos.pose.orientation.y = viconPos.transform.rotation.y;
    rosPos.pose.orientation.z = viconPos.transform.rotation.z;
    rosPos.pose.orientation.w = viconPos.transform.rotation.w;
}

// 飞 圆形 demo(机头沿切线方向)
void flyCircle(double r)
{
    // 更新当前位置
    currentPos = uavCurrentLocalPose;

    if(!hasInitFlyCircle)
    {
        initCirclePos = currentPos;
        nextPos = initCirclePos;
        angle = angle + angleStep;

        nextPos.pose.position.x = r * sin(angles::from_degrees(angle)) + initCirclePos.pose.position.x;
        nextPos.pose.position.y = r - r * cos(angles::from_degrees(angle)) + initCirclePos.pose.position.y;

        bodyangle = angle;
        flyCircleYaw = bodyangle/180.0*3.14159;
        nextPos.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,flyCircleYaw);
        ps.pose = nextPos.pose;
        ps.header.stamp = ros::Time::now();
        setPositionPublisher.publish(ps);
        ROS_INFO_STREAM("next angle:" << angle);
        hasInitFlyCircle = true;
    } 
    
    // judge whether is reached
    bool isReached = false;
    double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  + 
                         (currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
    double threshold = 0.2;
    if ((distance < threshold) && (fabs(tmpYaw - flyCircleYaw) < 0.1))
    {
        isReached = true;
    }

    if (isReached)
    {
        // send next pos
        angle = angle + angleStep;
        if(angle > 360) angle = angle - 360;
        nextPos = initCirclePos;

        nextPos.pose.position.x = r * sin(angles::from_degrees(angle)) + initCirclePos.pose.position.x;
        nextPos.pose.position.y = r - r * cos(angles::from_degrees(angle)) + initCirclePos.pose.position.y;

        bodyangle = angle;
        if (bodyangle > 180)
        {
           bodyangle = bodyangle - 360;
        }
        flyCircleYaw = bodyangle/180.0*3.14159;
        nextPos.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,flyCircleYaw);
        ps.pose = nextPos.pose;
        ps.header.stamp = ros::Time::now();
        setPositionPublisher.publish(ps);     
    } 
    else 
    {
        ps.header.stamp = ros::Time::now();
        setPositionPublisher.publish(ps);
        ROS_INFO_STREAM("next angle:" << angle);
    }
}

// 无人机当前位置和姿态(Pixhawk LPE 滤波结果)
void localPositionReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{
    uavCurrentLocalPose.pose.position.x = msg->pose.position.x;
    uavCurrentLocalPose.pose.position.y = msg->pose.position.y;
    uavCurrentLocalPose.pose.position.z = msg->pose.position.z;
    uavCurrentLocalPose.pose.orientation.x = msg->pose.orientation.x;
    uavCurrentLocalPose.pose.orientation.y = msg->pose.orientation.y;
    uavCurrentLocalPose.pose.orientation.z = msg->pose.orientation.z;
    uavCurrentLocalPose.pose.orientation.w = msg->pose.orientation.w;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(uavCurrentLocalPose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(tmpRoll, tmpPitch, tmpYaw);

    //ROS_INFO("Roll,Pitch,Yaw:[%0.3f,%0.3f,%0.3f]",tmpRoll/3.14*180,tmpPitch/3.14*180,tmpYaw/3.14*180);

    if (!isLockLocalPos)
    {
        ps = uavCurrentLocalPose;
        hasSet = true;
    }
}

// Vicon 数据接收处理
void viconPositionReceived(const geometry_msgs::TransformStampedConstPtr& vicon_msg)
{
    vicon_currentPos = *vicon_msg;
    PosetopicTransport(vicon_currentPos, uavCurrentViconPose);
}

/***************** 键盘控制 ********************/
void sendCommand(const keyboard::Key &key)
{
  switch(key.code)
  {
      case 'i':
      {
          // Forward
          ps.pose.position.x = ps.pose.position.x + pos_value * cos(tmpYaw);
          ps.pose.position.y = ps.pose.position.y + pos_value * sin(tmpYaw);
          ROS_INFO_STREAM("Forward: " << pos_value);        
          break;
      }
      case 'k':
      {
          // Backward
          ps.pose.position.x = ps.pose.position.x - pos_value * cos(tmpYaw);
          ps.pose.position.y = ps.pose.position.y - pos_value * sin(tmpYaw);
          ROS_INFO_STREAM("Backward: " << pos_value);       
          break;
      }
      case 'j':
      {
          // left
          ps.pose.position.x = ps.pose.position.x - pos_value * sin(tmpYaw);
          ps.pose.position.y = ps.pose.position.y + pos_value * cos(tmpYaw);
          ROS_INFO_STREAM("Left: " << pos_value);     
          break;
      }
      case 'l':
      {
          // right
          ps.pose.position.x = ps.pose.position.x + pos_value * sin(tmpYaw);
          ps.pose.position.y = ps.pose.position.y - pos_value * cos(tmpYaw);       
          ROS_INFO_STREAM("Right: " << pos_value);
          break;
      }
      case 'w':
      {
          // Up
          ps.pose.position.z += pos_value;      
          ROS_INFO_STREAM("Up: " << pos_value);
          break;
      }
      case 's':
      {
          // Down
          ps.pose.position.z -= pos_value;
          ROS_INFO_STREAM("Down: "<< pos_value);
          break;
      }
      case 'a':
      {
          // turn left
          tmpYaw += 0.05;
          if (tmpYaw > 3.14159)
          {
              tmpYaw = tmpYaw - 2*3.14159;
          }
          ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,tmpYaw);
          ROS_INFO("Yaw:[%0.3f]",tmpYaw/3.14159*180);
          break;
      }
      case 'd':
      {
          // turn right
          tmpYaw -= 0.05;
          if (tmpYaw < -3.14159)
          {
              tmpYaw = tmpYaw + 2*3.14159;
          }
          ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,tmpYaw);
          ROS_INFO("Yaw:[%0.3f]",tmpYaw/3.14159*180);
          break;
      }
      case 'y':
      {
          // fly shape demo
          isFlyCircle = true;
          ROS_INFO_STREAM("Fly circle shape Mode");
          break;
      }
      case 'h':
      {
          // turn to manual mode
          isFlyCircle = false;
          hasInitFlyCircle = false;
          ROS_INFO_STREAM("Manual Mode");
          break;
      }
      case 'b':
      {
          // unlock local position
          isLockLocalPos = false;
          break;
      }
      case 'n':
      {
          // lock local position
          isLockLocalPos = true;
          break;
      }
      default:
      {

      }
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_offboard_position_control_node");
    ros::NodeHandle nodeHandle;

    ROS_INFO_STREAM("start");

    setPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros04/setpoint_position/local",10);
    currentViconPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros04/mocap/pose",10);
    ros::Subscriber localPositionSubsciber = nodeHandle.subscribe("/mavros04/local_position/pose", 10, localPositionReceived);
    ros::Subscriber viconPositionSubsciber = nodeHandle.subscribe("/vicon/nuflie04/nuflie04", 10, viconPositionReceived);
    ros::Subscriber commandSubscriber = nodeHandle.subscribe("/keyboard/keydown",1,sendCommand);
    ros::Subscriber stateSubscriber = nodeHandle.subscribe("mavros04/state", 10, stateReceived);
    
    pos_value = 0.2f;
    hasSet = false;

    // fly circle parameters
    isFlyCircle = false;
    hasInitFlyCircle = false;
    angle = 0.0;
    bodyangle = 0.0;
    //radius = 1.75;
    //angleStep = 7.0;
    
    // 飞行半径和机头方向累加角度
    ros::param::param("~radius",radius,1.0);
    ROS_INFO("radius:%0.3f",radius);
    ros::param::param("~angle",angleStep,5);
    ROS_INFO("angleStep:%d",angleStep);

    isLockLocalPos = false;

    tmpRoll = 0.0;
    tmpPitch = 0.0;
    tmpYaw= 0.0;
    flyCircleYaw = 0.0;

    ros::Rate loopRate(50.0);
    while(ros::ok())
    {
      if(hasSet) 
      {
          ps.header.seq++;

          if(!isFlyCircle)
          {
              ps.header.stamp = ros::Time::now();
              setPositionPublisher.publish(ps); 
          } 
          else 
          {
              flyCircle(radius);
          }

          // 向 mocap/pose 话题发送 Vicon 提供的位姿数据
          uavCurrentViconPose.header.stamp = ros::Time::now();
          uavCurrentViconPose.header.frame_id = "/world";
          currentViconPositionPublisher.publish(uavCurrentViconPose);  
      }

      ros::spinOnce();
      loopRate.sleep();
    }
}
