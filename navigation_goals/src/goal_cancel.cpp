#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "actionlib_msgs/GoalID.h"
#include <iostream>

using namespace std;
int tag;
// 回调函数
void tagCallback(const std_msgs::Int8::ConstPtr& msg)
{
  tag=msg->data;
  //std::cout<<tag<<endl;
}


int main(int argc, char **argv)
{
 
    ros::init(argc, argv, "goal_cancel");
    ros::NodeHandle nh;
    ros::Subscriber sub_tag = nh.subscribe("/tag", 1, tagCallback);
    ros::Publisher cancel_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    ros::Rate loop_rate(10);  // 自循环频率
    int count=0;
    int countz=0;
    int countx=0;
    int countc=0;
    while (ros::ok())
    {
        /*if(tag==6)
        {
            if(count<1)
            {
                actionlib_msgs::GoalID empty_goal;
                cout<<"next goal!"<<endl;
                //ros::Duration(2).sleep();
                cancel_pub.publish(empty_goal);
                count+=1;
            }
        }*/
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
    ros::spinOnce();
    // 休眠，来使发布频率为10Hz
    //ros::Duration(2).sleep();
    loop_rate.sleep();
  }
  //count=0;
  return 0;
}
