#include<iostream>
#include<fstream>
#include<ros/ros.h>
#include<serial/serial.h>
#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include"gps_msgs/Inspvax.h"
#include"gps_msgs/Satellite.h"
#include"gps_msgs/Satellites.h"

using namespace std;

class Recorder
{
public:
	Recorder(){}
	~Recorder()
	{
		if(mOutFileInspvax.is_open())
			mOutFileInspvax.close();
		if(mOutFileSatellite.is_open())
			mOutFileSatellite.close();
	}
	
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg)
	{
		mInspvaxMsg = *msg;
	}
	
	void satellite_callback(const gps_msgs::Satellites::ConstPtr& msgs)
	{
		mSatellites = *msgs;
	}
	
	void timer_callback(const ros::TimerEvent&)
	{
		static size_t count = 0;
		mOutFileInspvax << fixed << setprecision(7)
						<< mInspvaxMsg.latitude << "\t" << mInspvaxMsg.longitude << "\t"
						<< setprecision(2)
						<< mInspvaxMsg.azimuth << "\t" << mInspvaxMsg.height <<"\r\n";
						
		for(auto& satellite:mSatellites.satellites)
		{
			mOutFileSatellite << setw(6)
			 				  << int(satellite.navigation_system) << "\t" << int(satellite.satellite_num) << "\t"
							  << int(satellite.satellite_frequency) << "\t" << int(satellite.elevation) << "\t"
							  << int(satellite.azimuth) << "\t" << int(satellite.snr) << "\r\n";
		}
		mOutFileSatellite << "--------------------------\r\n";
		ROS_INFO("%5ld: recording...",++count);
						
	}
	
	bool init(int argc,char** argv)
	{
		ros::init(argc,argv,"record_data_node");
		ros::NodeHandle nh;
		ros::NodeHandle nh_private("~");
		
		mSubGpsMsg = nh.subscribe(nh_private.param<string>("gps_topic","/gps"),1, &Recorder::gps_callback,this);
		
		mSubSatelliteMsg = nh.subscribe(nh_private.param<string>("satellite_topic","/satellite"),1, &Recorder::satellite_callback, this);
		
		mTimer = nh.createTimer(ros::Duration(nh_private.param<float>("record_duration",1.0)),&Recorder::timer_callback, this);
		
		string file_name1 = nh_private.param<string>("satellite_file", "satellite.txt");
		string file_name2 = nh_private.param<string>("gps_file", "gps.txt");
		
		mOutFileSatellite.open(file_name1);
		if(!mOutFileSatellite.is_open())
		{
			ROS_ERROR("open %s failed!!!",file_name1.c_str());
			return false;
		}
		
		mOutFileInspvax.open(file_name2);
		if(!mOutFileInspvax.is_open())
		{
			ROS_ERROR("open %s failed!!!",file_name2.c_str());
			return false;
		}
		
		
		ROS_INFO("open %s and %s ok ...",file_name1.c_str(),file_name2.c_str());
		
			
		mOutFileInspvax << "latitude\t" << "longitude\t" <<"yaw\t" << "height\r\n";
		mOutFileSatellite << "navigation_system\t" << "satellite_num\t" << "satellite_frequency\t" << "elevation\t" << "azimuth\t" << "snr\r\n"; 
		return true;
	}



private:
	ros::Subscriber mSubGpsMsg;
	ros::Subscriber mSubSatelliteMsg;
	ros::Timer mTimer;
	gps_msgs::Inspvax mInspvaxMsg;
	gps_msgs::Satellites mSatellites;
	
	ofstream mOutFileInspvax;
	ofstream mOutFileSatellite;
	
};

int main(int argc, char** argv)
{
	Recorder recorder;
	if(!recorder.init(argc,argv))
		return 0;
	ROS_INFO("%s initial ok...",ros::this_node::getName().c_str());
	ros::spin();
	return 0;
}


