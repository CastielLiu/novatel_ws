#include<ros/ros.h>
#include<gps_msgs/Inspvax.h>
#include<fstream>
#include<iomanip>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPoint.h>

using namespace std;

class RecordTofile
{
public:
	RecordTofile()
	{
		ros::NodeHandle nh;
		ros::NodeHandle nh_private("~");
		string gps_topic = nh_private.param<string>("gps_topic","/gps");
		
		sub = nh.subscribe(gps_topic ,10,&RecordTofile::callback,this);
		out_file.open("gps.txt");
	}
	~RecordTofile()
	{
		out_file.close();
	}
	void callback(const gps_msgs::Inspvax::ConstPtr & msg)
	{
		double time = msg->header.stamp.toSec();
		geographic_msgs::GeoPoint point;
		point.latitude = msg->latitude;
		point.longitude = msg->longitude;
		point.altitude = msg->height;
		
		geodesy::UTMPoint utm;
		geodesy::fromMsg(point, utm);
	
		out_file << fixed << setprecision(3) << time << "\t" 
				 << setprecision(7)
				 << msg->latitude << "\t" << msg->longitude << "\t"
				 << setprecision(3)
				 << msg->azimuth << "\t"
				 << utm.easting << "\t" 
				 << utm.northing << "\t" 
				 << utm.altitude <<"\r\n";
						
	}
	
private:
	ros::Subscriber sub;
	ofstream out_file;
};




int main(int argc,char **argv)
{
	ros::init(argc,argv,"recordTofile");
	
	RecordTofile record;
	
	ros::spin();
}

