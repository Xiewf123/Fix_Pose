#include "ros/ros.h" 
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "math.h" 
#include <sstream> 
#include <iostream> 
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include "std_msgs/Int8.h"
using namespace std;

double X_POS,Y_POS;
double	roll,pitch,yaw;
double angular_speed,goal_angle,angular_duration;
double linear_speed,goal_distance,linear_duration;
double Forward_goal_distance,Backward_goal_distance;


int ticks;
int tag;
//int count=0;//记录循环次数
int rate=50;//定义更新频率
bool Callback_flag = false;
//ros::Rate loop_rate(rate);//更新频率50Hz，它会追踪记录自上一次调用Rate::sleep()后时间的流逝，并休眠直到一个频率周期的时间 
ros::Publisher cmd_vel_pub;

/***********tagCallback回调，获取二维码标签值***********/
void tagCallback(const std_msgs::Int8::ConstPtr& msg)
{
  tag=msg->data;
  std::cout<<tag<<endl;
  /*if(tag==3)
	  Callback_flag = true;*/
}


/***********odom_revise回调，获取偏差值***********/
void revise_pose_callback(const nav_msgs::OdometryConstPtr& pose_msg)
{

    X_POS = pose_msg->pose.pose.position.x-4.8;
    Y_POS = pose_msg->pose.pose.position.y-1.2;
	//X_POS = pose_msg->pose.pose.position.x;
    //Y_POS = pose_msg->pose.pose.position.y;
    //cout<<"X_POS:"<<X_POS<<endl;
    tf::Quaternion	RQ2;		
    tf::quaternionMsgToTF(pose_msg->pose.pose.orientation,RQ2);
    tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);
	//cout<<"Yaw:"<<yaw<<endl;
}

int main(int argc, char **argv) 
{ 	
    ros::init(argc,argv,"fix_pose");//指定节点
	ros::NodeHandle n;//创造一个节点句柄 
	ros::Subscriber sub_tag = n.subscribe("/tag", 20, tagCallback);
    ros::Subscriber sub_revise = n.subscribe("odom_revise", 50, revise_pose_callback); //订阅/odom_revise主题
	cmd_vel_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);//将在/cmd_vel话题上发布一个geometry_msgs::Twist消息
	rate=50;//定义更新频率
	ros::Rate loop_rate(rate);//更新频率50Hz，它会追踪记录自上一次调用Rate::sleep()后时间的流逝，并休眠直到一个频率周期的时间 
         
	int count =0;

	while(ros::ok())//等待键盘ctrl+C操作则停止 
	{ 
			ros::spinOnce();
			double linear_speed=0.008;//向前的线速度0.005m/s  
			double angular_speed=0.05;//角速度0.05rad/s 
			double pgv_distance=0.132;//两轮中点到pgv探头的距离
			double distance_tolerance=0.002;//距离的误差容忍在0.002m以内
			double angular_tolerance=1;//误差容忍为1度
			double goal_angle=yaw;//旋转角度
			int angle = int(yaw*180/M_PI);
			//cout<<"angle:"<<angle<<endl;
			geometry_msgs::Twist move_cmd;//定义消息对象 
			move_cmd.linear.x=move_cmd.linear.y=move_cmd.linear.z=0; 
			move_cmd.angular.x=move_cmd.angular.y=move_cmd.angular.z=0;	
		if(count<3&&tag==3)
		{
			cout<<"count:"<<count<<endl;
			/**************************先直行再旋转再后退（第一象限与第四象限）**************************/
			if(Y_POS<0&&angle>angular_tolerance)
			{
				/**************************初始化**************************/
				cout<<"AAA"<<endl;
				count+=1;
				Forward_goal_distance=fabs(Y_POS)/sin(yaw)+pgv_distance;//行进记距离
				if(X_POS<0)
					Backward_goal_distance=-fabs(X_POS)-(Y_POS)/tan(yaw)+pgv_distance;//后退距离
				else
					Backward_goal_distance=fabs(X_POS)-(Y_POS)/tan(yaw)+pgv_distance;
			}
			
			/**************************分情况考虑(第二象限与第三象限)**************************/
			if(Y_POS>=0&&angle>angular_tolerance)
			{
				cout<<"BBB"<<endl;
				count+=1;
				/**************************初始化**************************/
				Forward_goal_distance=pgv_distance-fabs(Y_POS/sin(yaw));//后退距离
				cout<<"Forward_goal_distance:"<<Forward_goal_distance<<endl;
				/**************************先直行再旋转再后退**************************/
				if(Forward_goal_distance>=0)
				{
					if(X_POS<0)
						Backward_goal_distance=pgv_distance-fabs(Y_POS/tan(yaw))-fabs(X_POS);//后退距离
					else
						Backward_goal_distance=pgv_distance-fabs(Y_POS/tan(yaw))+fabs(X_POS);//后退距离	
				}

				/**************************先后退再旋转再前进**************************/
				else
				{
					cout<<"CCC"<<endl;
					linear_speed=-linear_speed;//将线速度变为负
					Forward_goal_distance=fabs(Forward_goal_distance);//后退距离
					if(X_POS<0)
						Backward_goal_distance=fabs(Y_POS/tan(yaw))-fabs(X_POS)-pgv_distance;//前进距离
					else
						Backward_goal_distance=pgv_distance-fabs(Y_POS/tan(yaw))-fabs(X_POS);//前进距离						
					
				}

			}
			
			/**************************角度偏差在容忍度范围内时纠正X的值**************************/	
			if(fabs(X_POS)>distance_tolerance&&angle<=angular_tolerance)
			{
				linear_speed=fabs(linear_speed);
				count+=1;
				if(X_POS<0)
				{	
					Forward_goal_distance=-X_POS;//前进距离
					Backward_goal_distance=0;
				}
					
				else
				{
					Forward_goal_distance=0;
					Backward_goal_distance=X_POS;//后退距离
				}
					
				angular_speed=0;
			}
/***********************************运动过程***********************************/	
				/**************************直行**************************/				
				move_cmd.linear.x=linear_speed;//设置线速度
				//cout<<move_cmd.linear.x<<endl;
				linear_duration=fabs(Forward_goal_distance/linear_speed);//前进的时间
    			ticks=int(linear_duration*rate);
				//cout<<"前进时间："<<linear_duration<<endl;
				for(int i=0;i<ticks;i++) 
				{ 
	    			cmd_vel_pub.publish(move_cmd); 
	    			loop_rate.sleep(); 
    			}
				//旋转前先停止
    			move_cmd.linear.x=0; 
				cmd_vel_pub.publish(move_cmd); 
				ros::Duration(1).sleep(); //休眠1s 
				cmd_vel_pub.publish(move_cmd);

				/**************************旋转**************************/
 	 			move_cmd.angular.z=-angular_speed;//设置角速度 
				angular_duration=fabs(goal_angle/angular_speed);//旋转时间 
				ticks=int(angular_duration*rate);             
    			for(int i=0;i<ticks;i++) 
				{ 
					cmd_vel_pub.publish(move_cmd); 
					loop_rate.sleep(); 
    			}
   				 //停止 
				move_cmd.angular.z=0; 
				cmd_vel_pub.publish(move_cmd); 
				ros::Duration(1).sleep();

				/**************************后退**************************/
				move_cmd.linear.x=-linear_speed;//设置线速度
				double linear_duration=fabs(Backward_goal_distance/linear_speed);//后退的时间
				//cout<<"后退时间："<<linear_duration<<endl;
    			ticks=int(linear_duration*rate);
				for(int i=0;i<ticks;i++) 
				{ 
	    			cmd_vel_pub.publish(move_cmd); 
	    			loop_rate.sleep(); 
    			}
				//停止
    			move_cmd.linear.x=0; 
				cmd_vel_pub.publish(move_cmd); 
				ros::Duration(1).sleep(); //休眠1s 				 
		}
		loop_rate.sleep(); 
	}
	
//ros::Duration(1).sleep();  
	return 0; 
}

