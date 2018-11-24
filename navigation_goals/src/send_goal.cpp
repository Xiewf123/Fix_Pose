#include <ros/ros.h>  
#include <move_base_msgs/MoveBaseAction.h>  
#include <actionlib/client/simple_action_client.h> 
#include <iostream>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Int8.h"
#include "actionlib_msgs/GoalID.h"

using namespace std;
int tag;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 

typedef struct _POSE
{
  double X;
  double Y;
  double Z;
  double or_x;
  double or_y;
  double or_z;
  double or_w;
} POSE;

//设置导航的定位点
POSE pose1 = {1.200, 0.000, 0.000, 0.000, 0.000, 0.000, 1.000};//识别到6时发送
POSE pose2 = {2.400, 0.000, 0.000, 0.000, 0.000, 0.000, 1.000};//识别到7时发送
POSE pose3 = {3.600, 0.600, 0.000, 0.000, 0.000, 0.000, 1.000};//识别到8、9时发送
POSE pose4 = {4.800, 1.200, 0.000, 0.000, 0.000, 0.000, 1.000};
POSE pose5 = {4.800, 1.200, 0.000, 0.000, 0.000, 0.000, 1.000};


void setGoal(POSE pose)
{
     // 定义客户端 
    MoveBaseClient ac("move_base", true);  
      
    // 等待服务器
    while(!ac.waitForServer(ros::Duration(5.0))){  
        ROS_WARN("Waiting for the move_base action server to come up");  
    }  
    
    // 创建action的goal
    move_base_msgs::MoveBaseGoal goal;  
      
    // 发送位置坐标 
    goal.target_pose.header.frame_id = "map";  
    goal.target_pose.header.stamp = ros::Time::now();  
   
    goal.target_pose.pose.position.x = pose.X; 
    goal.target_pose.pose.position.y = pose.Y;  
    goal.target_pose.pose.position.z = pose.Z;   
    goal.target_pose.pose.orientation.x = pose.or_x;
    goal.target_pose.pose.orientation.y = pose.or_y;
    goal.target_pose.pose.orientation.z = pose.or_z;
    goal.target_pose.pose.orientation.w = pose.or_w;   
    
    ROS_INFO("Sending goal");  
    
    // 发送action的goal给服务器端，并且设置回调函数
    ac.sendGoal(goal);

    // 等待结果 
    ac.waitForResult();  
      
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("it is successful"); 
    } 
       
     else  
     {
         ROS_ERROR("The base failed  move to goal!!!");
     }
       
}

void tagCallback(const std_msgs::Int8::ConstPtr& msg)
{
  tag=msg->data;
  //std::cout<<tag<<endl;
  //Callback_flag = true;
}

void spinThread()
{
    ros::NodeHandle nh;
    ros::Subscriber sub_tag = nh.subscribe("/tag", 20, tagCallback);
    ros::Publisher cancel_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 2);
    ros::Rate loop_rate(10);  // 自循环频率
    int count=0;
    int countz=0;
    int countx=0;
    int countc=0;
    int countv=0;
    while (ros::ok())
    {
        if(tag==7)
        {
            if(countz<1)
            {
                actionlib_msgs::GoalID empty_goal;
                cout<<"next goal!"<<endl;
                //ros::Duration(2).sleep();
                cancel_pub.publish(empty_goal);
                countz+=1;
            }
        }            
        if(tag==8)
        {
            if(countx<1)
            {
                actionlib_msgs::GoalID empty_goal;
                cout<<"next goal!"<<endl;
                //ros::Duration(2).sleep();
                cancel_pub.publish(empty_goal);
                countx+=1;
            }
        }     
        if(tag==4)
        {
            if(countc<1)
            {
                actionlib_msgs::GoalID empty_goal;
                cout<<"next goal!"<<endl;
                //ros::Duration(2).sleep();
                cancel_pub.publish(empty_goal);
                countc+=1;
            }

        }  
        if(tag==3)
        {
            if(countv<1)
            {
                actionlib_msgs::GoalID empty_goal;
                cout<<"next goal!"<<endl;
                //ros::Duration(2).sleep();
                cancel_pub.publish(empty_goal);
                countv+=1;
            }

        }
    ros::spinOnce();
    // 休眠，来使发布频率为10Hz
    //ros::Duration(2).sleep();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "base_pose_control");  
  ros::NodeHandle n;
  //ros::Publisher pub_initialpose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
  
  //setInitial_pose(pub_initialpose);//发送初始位置
  MoveBaseClient ac("move_base", true);  

  int locations_cnt = 1; //设定巡航圈数
  int loop_cnt=0;//当前巡航圈数
  
  while(ros::ok())
  {
    boost::thread spin_thread(&spinThread);
    if(loop_cnt<locations_cnt)
    {
        loop_cnt += 1;
        ROS_INFO("Left loop cnt:%d",locations_cnt-loop_cnt);
        setGoal(pose1);
        setGoal(pose2);
        setGoal(pose3);
        setGoal(pose4);
    }
    // 巡航结束后返回原点
    else
    {
        //setGoal(pose5);
        ROS_INFO("Navigation test finished");
        ros::shutdown();
    }
    
    //boost::thread spin_thread(&spinThread); // the thread is created and the ros node is started spinning in the background
    spin_thread.join();
    ros::spinOnce();
    //rate_loop.sleep();
  }
  return 0;  
}  
 
