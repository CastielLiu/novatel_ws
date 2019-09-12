#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<iostream>

using namespace std;

class TestImu
{
public:
	TestImu()
	{
		memset(&new_imu_msg_,0,sizeof(new_imu_msg_));
	}
	~TestImu(){}
	
	void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		new_imu_msg_.angular_velocity.x += msg->angular_velocity.x;
		new_imu_msg_.angular_velocity.y += msg->angular_velocity.y;
		new_imu_msg_.angular_velocity.z += msg->angular_velocity.z;
		
		new_imu_msg_.linear_acceleration.x += msg->linear_acceleration.x;
		new_imu_msg_.linear_acceleration.y += msg->linear_acceleration.y;
		new_imu_msg_.linear_acceleration.z += msg->linear_acceleration.z;
		
		pub_imu_.publish(new_imu_msg_);
	}
	
	void init(int argc,char** argv)
	{
		ros::init(argc,argv,"test_imu_node");
		ros::NodeHandle nh;
		sub_imu_ = nh.subscribe("/imu",0,&TestImu::imu_callback,this);
		pub_imu_ = nh.advertise<sensor_msgs::Imu>("/imu_new",0);
		
		ros::spin();
	}
private:
	ros::Subscriber sub_imu_;
	ros::Publisher pub_imu_;
	sensor_msgs::Imu new_imu_msg_;
	
};

int main(int argc,char** argv)
{
	TestImu test;
	test.init(argc,argv);
	return 0;
}
