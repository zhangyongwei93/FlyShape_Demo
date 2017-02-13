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

#define PI 3.14159

geometry_msgs::PoseStamped oriPos,ps,currentPos,nextPos,initCirclePos;
geometry_msgs::PoseStamped uavCurrentLocalPose,uavCurrentViconPose;
geometry_msgs::TransformStamped vicon_currentPos;
geometry_msgs::PoseStamped vicon_mocaptf;

ros::Publisher setPositionPublisher;
ros::Publisher currentViconPositionPublisher;

double radius;
float pos_value;
float bodyangle;
int angle,angleStep;

double tmpRoll,tmpPitch,tmpYaw;
double flyCircleYaw;

float last_pos_x,last_pos_y;
float curr_pos_x,curr_pos_y;
float next_pos_x,next_pos_y;
float next_body_angle,curr_body_angle,acc_body_angle;

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

void flyNum8(double r)
{
    // 更新当前位置
    currentPos = uavCurrentLocalPose;

    // InitFly
    if(!hasInitFlyCircle)
    {
        initCirclePos = currentPos;
        nextPos = initCirclePos;
        angle = angle + angleStep;

        nextPos.pose.position.x = r * sin(2*angles::from_degrees(angle)) / 2.0 + initCirclePos.pose.position.x;
        nextPos.pose.position.y = r * cos(angles::from_degrees(angle)) - r + initCirclePos.pose.position.y;
        bodyangle = -6.0; // 初始估计值
        flyCircleYaw = bodyangle/180.0*PI;
        nextPos.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,flyCircleYaw);
        ps.pose = nextPos.pose;
        ps.header.stamp = ros::Time::now();
        setPositionPublisher.publish(ps);
        ROS_INFO_STREAM("next angle:" << angle);
        hasInitFlyCircle = true;

        last_pos_x = initCirclePos.pose.position.x;
        last_pos_y = initCirclePos.pose.position.y;
        //当前位置是上一个nextPos
        curr_pos_x = nextPos.pose.position.x;
        curr_pos_y = nextPos.pose.position.y;
    } 

    // judge whether is reached
    bool isReached = false;
    double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  + 
                         (currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
    double threshold = 0.3;

    ROS_INFO_STREAM("tmpYaw:" << tmpYaw);
    ROS_INFO_STREAM("flyCircleYaw:" << flyCircleYaw);
    if ((distance < threshold) && (fabs(tmpYaw - flyCircleYaw) < 0.22))
    {
        isReached = true;
    }

    if (isReached)
    {
        // send next pos
        angle = angle + angleStep;
        if(angle > 360) angle = angle - 360;
        nextPos = initCirclePos;
        nextPos.pose.position.x = r * sin(2*angles::from_degrees(angle)) / 2.0 + initCirclePos.pose.position.x;
        nextPos.pose.position.y = r * cos(angles::from_degrees(angle)) - r + initCirclePos.pose.position.y;

        //下一个目标位置
        next_pos_x = nextPos.pose.position.x;
        next_pos_y = nextPos.pose.position.y;

        //计算上一个位置与现在位置，现在位置与下一个位置的夹角差，为飞机累积转动角度（弧度）
        //防止除0操作
        if ((fabs(next_pos_x - curr_pos_x) > 0.001) && (fabs(curr_pos_x - last_pos_x) > 0.001))
        {
            next_body_angle = atan((next_pos_y - curr_pos_y) / (next_pos_x - curr_pos_x));
            curr_body_angle = atan((curr_pos_y - last_pos_y) / (curr_pos_x - last_pos_x));

            //考虑跨越 正负90°情况
            if ((fabs(curr_body_angle - (PI/2)) < 0.35) && (fabs(next_body_angle - (-PI/2)) < 0.35))
            {
                acc_body_angle = next_body_angle - curr_body_angle + PI;
            }
            else if ((fabs(curr_body_angle - (-PI/2)) < 0.35) && (fabs(next_body_angle - (PI/2)) < 0.35))
            {
                acc_body_angle = next_body_angle - curr_body_angle - PI;
            }
            else
            {
                acc_body_angle = next_body_angle - curr_body_angle;            
            }
        }
        else //出现除0，进入定点悬停
        {
            isFlyCircle = false;
            hasInitFlyCircle = false;
            ROS_INFO_STREAM("Divisor is 0, Enter Manual Mode");
        }

        flyCircleYaw = flyCircleYaw + acc_body_angle;
        nextPos.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,flyCircleYaw);
        ps.pose = nextPos.pose;
        ps.header.stamp = ros::Time::now();
        setPositionPublisher.publish(ps);
        
        // 更新位置
        last_pos_x = curr_pos_x;
        last_pos_y = curr_pos_y;
        curr_pos_x = next_pos_x;
        curr_pos_y = next_pos_y;          
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

    // ROS_INFO("Roll,Pitch,Yaw:[%0.3f,%0.3f,%0.3f]",tmpRoll/PI*180,tmpPitch/PI*180,tmpYaw/PI*180);

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
          if (tmpYaw > PI)
          {
              tmpYaw = tmpYaw - 2*PI;
          }
          ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,tmpYaw);
          ROS_INFO("Yaw:[%0.3f]",tmpYaw/PI*180);
          break;
      }
      case 'd':
      {
          // turn right
          tmpYaw -= 0.05;
          if (tmpYaw < -PI)
          {
              tmpYaw = tmpYaw + 2*PI;
          }
          ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,tmpYaw);
          ROS_INFO("Yaw:[%0.3f]",tmpYaw/PI*180);
          break;
      }
      case 'y':
      {
          // fly shape demo
          isFlyCircle = true;
          ROS_INFO_STREAM("Fly Circle Mode");
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

    // fly shape demo parameters
    isFlyCircle = false;
    hasInitFlyCircle = false;
    angle = 0.0;
    bodyangle = 0.0;
    //radius = 1.0;
    angleStep = 5.0;
    // 飞行半径
    ros::param::param("~radius",radius,1.0);
    ROS_INFO("radius:%0.3f",radius);

    isLockLocalPos = false;

    tmpRoll = 0.0;
    tmpPitch = 0.0;
    tmpYaw= 0.0;
    flyCircleYaw = 0.0;
    next_body_angle = 0.0;
    curr_body_angle = 0.0;
    acc_body_angle = 0.0;

    last_pos_x = 0.0;
    last_pos_y = 0.0;
    curr_pos_x = 0.0;
    curr_pos_y = 0.0;
    next_pos_x = 0.0;
    next_pos_y = 0.0;

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
                flyNum8(radius);
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
