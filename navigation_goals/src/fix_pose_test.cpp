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

class Fixpose{
    public:
        Fixpose();
        void Forward(double linear_speed, double goal_distance);
        void TurnRound(double angular_speed, double angular);
        void Backward(double linear_speed, double goal_distance);
        void Action();

    private:
        void tagCallback(const std_msgs::Int8::ConstPtr& msg);
        void revise_pose_callback(const nav_msgs::OdometryConstPtr& pose_msg);
        ros::NodeHandle n;//创造一个节点句柄
        ros::Publisher cmd_vel_pub;
        ros::Subscriber sub_tag;
        ros::Subscriber sub_revise;
        int rate;//发送频率
        int ticks;
        int angle;
        int tag;
        int count;
        double roll,pitch,yaw;
        double X_POS,Y_POS;
        double angular_speed,goal_angle,angular_duration;
        double linear_speed,goal_distance,linear_duration;
        double Forward_goal_distance,Backward_goal_distance;
        double pgv_distance,distance_tolerance,angular_tolerance;
};

Fixpose::Fixpose()
{
    cmd_vel_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);//将在/cmd_vel话题上发布一个geometry_msgs::Twist消息
    sub_tag = n.subscribe("/tag", 20, &Fixpose::tagCallback,this);
    sub_revise = n.subscribe("odom_revise", 50, &Fixpose::revise_pose_callback,this); //订阅/odom_revise主题
}

void Fixpose::tagCallback(const std_msgs::Int8::ConstPtr& msg)
{
    tag=msg->data;
    cout<<tag<<endl;
}

void Fixpose::revise_pose_callback(const nav_msgs::OdometryConstPtr& pose_msg)
{
    X_POS = pose_msg->pose.pose.position.x-4.8;
    Y_POS = pose_msg->pose.pose.position.y-1.2;
    //double	roll,pitch,yaw;
	//X_POS = pose_msg->pose.pose.position.x;
    //Y_POS = pose_msg->pose.pose.position.y;
    //cout<<"X_POS:"<<X_POS<<endl;
    tf::Quaternion	RQ2;		
    tf::quaternionMsgToTF(pose_msg->pose.pose.orientation,RQ2);
    tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);
	//cout<<"Yaw:"<<yaw<<endl;    
}

/****************************前进****************************/
void Fixpose::Forward(double linear_speed, double goal_distance)
{
	geometry_msgs::Twist move_cmd;//定义消息对象 
	move_cmd.linear.x=move_cmd.linear.y=move_cmd.linear.z=0; 
	move_cmd.angular.x=move_cmd.angular.y=move_cmd.angular.z=0;
	move_cmd.linear.x=linear_speed;//设置线速度
	linear_duration=goal_distance/linear_speed;//后退的时间
    rate=50;
    ros::Rate loop_rate(rate);
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

/****************************顺时针旋转****************************/
void Fixpose::TurnRound(double angular_speed, double goal_angle)
{
	geometry_msgs::Twist move_cmd;//定义消息对象 
	move_cmd.linear.x=move_cmd.linear.y=move_cmd.linear.z=0; 
	move_cmd.angular.x=move_cmd.angular.y=move_cmd.angular.z=0;
 	move_cmd.angular.z=-angular_speed;//设置角速度 
	angular_duration=fabs(goal_angle/angular_speed);//旋转时间 
    rate=50;
    ros::Rate loop_rate(rate);
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
}

/****************************后退****************************/
void Fixpose::Backward(double linear_speed, double goal_distance)
{
	geometry_msgs::Twist move_cmd;//定义消息对象
	move_cmd.linear.x=move_cmd.linear.y=move_cmd.linear.z=0; 
	move_cmd.angular.x=move_cmd.angular.y=move_cmd.angular.z=0;  
	move_cmd.linear.x=-linear_speed;//设置线速度
	double linear_duration=goal_distance/linear_speed;//后退的时间
    rate=50;
    ros::Rate loop_rate(rate);
    ticks=int(linear_duration*rate);
	for(int i=0;i<ticks;i++) 
	{ 
	    cmd_vel_pub.publish(move_cmd);
	    loop_rate.sleep(); 
    }
	//停止
    move_cmd.linear.x=0; 
	cmd_vel_pub.publish(move_cmd); 
	ros::Duration(1.0).sleep(); //休眠1s 	
}

void Fixpose::Action()
{
        /**************************初始化**************************/
		linear_speed=0.008;//向前的线速度0.005m/s  
		angular_speed=0.05;//角速度0.05rad/s 
		pgv_distance=0.132;//两轮中点到pgv探头的距离
		distance_tolerance=0.002;//距离的误差容忍在0.002m以内
		angular_tolerance=1;//误差容忍为1度
		goal_angle=yaw;//旋转角度
        ros::Rate r(50);
        //cout<<"ganle:"<<angle<<endl;
		angle = int(yaw*180/M_PI);
        if(tag==3)
		{
/**************************先直行再顺时针旋转再后退（第一象限与第四象限）**************************/
			if(Y_POS<0&&angle>angular_tolerance)
			{
				/**************************初始化**************************/
				cout<<"AAA"<<endl;
				count+=1;
				Forward_goal_distance=fabs(Y_POS)/sin(yaw)+pgv_distance;//前进距离
				if(X_POS<0)
					Backward_goal_distance=-fabs(X_POS)-(Y_POS)/tan(yaw)+pgv_distance;//后退距离
				else
					Backward_goal_distance=fabs(X_POS)-(Y_POS)/tan(yaw)+pgv_distance;
                Forward(linear_speed,Forward_goal_distance);
                TurnRound(angular_speed,goal_angle);
                Backward(linear_speed,Backward_goal_distance);
			}
			
/**************************分情况考虑(第二象限与第三象限)**************************/
			if(Y_POS>=0&&angle>angular_tolerance)
			{
				cout<<"BBB"<<endl;
				count+=1;
				/**************************初始化**************************/
				Forward_goal_distance=pgv_distance-fabs(Y_POS/sin(yaw));//前进距离
				cout<<"Forward_goal_distance:"<<Forward_goal_distance<<endl;
				/**************************先直行再旋转再后退**************************/
				if(Forward_goal_distance>=0)
				{
					if(X_POS<0)
						Backward_goal_distance=pgv_distance-fabs(Y_POS/tan(yaw))-fabs(X_POS);//后退距离
					else
						Backward_goal_distance=pgv_distance-fabs(Y_POS/tan(yaw))+fabs(X_POS);//后退距离
                    Forward(linear_speed,Forward_goal_distance);
                    cout<<"angular_speed:"<<angular_speed<<endl;
                    TurnRound(angular_speed,goal_angle);
                    Backward(linear_speed,Backward_goal_distance);	
				}
				/**************************先后退再旋转再前进**************************/
				else
				{
					cout<<"CCC"<<endl;
					//linear_speed=fabs(linear_speed);//将线速度变为负
					Backward_goal_distance=fabs(Forward_goal_distance);//后退距离
					if(X_POS<0)
						Forward_goal_distance=fabs(Y_POS/tan(yaw))-fabs(X_POS)-pgv_distance;//前进距离
					else
						Forward_goal_distance=pgv_distance-fabs(Y_POS/tan(yaw))-fabs(X_POS);//前进距离						
                    Backward(linear_speed,Backward_goal_distance);
                    TurnRound(angular_speed,goal_angle);
                    Forward(linear_speed,Forward_goal_distance);					
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
					Forward(linear_speed,Forward_goal_distance);
				}
					
				else
				{
					Backward_goal_distance=X_POS;//后退距离
                    Backward(linear_speed,Backward_goal_distance);
				}
			}
        }
        r.sleep();
}

/****************************主函数****************************/
int main(int argc, char **argv) 
{ 	
    ros::init(argc,argv,"fix_pose");//指定节点  
    Fixpose Pose;	
    while(ros::ok())
    {
		ros::spinOnce();
        Pose.Action();
        
    }
    //ros::Duration(1).sleep();  
	return 0; 
}

