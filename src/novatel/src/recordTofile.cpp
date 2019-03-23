#include<ros/ros.h>
#include<gps_msgs/Inspvax.h>
#include<fstream>
#include<iomanip>

using namespace std;

class RecordTofile
{
public:
	RecordTofile()
	{
		ros::NodeHandle nh;
		sub = nh.subscribe("/gps",10,&RecordTofile::callback,this);
		out_file.open("gps.txt");
	}
	~RecordTofile()
	{
		out_file.close();
	}
	void callback(const gps_msgs::Inspvax::ConstPtr & msg)
	{
		out_file.setf(ios::fixed);
	//	out_file <<setprecision(8) << msg->latitude <<"," << msg->longitude<<"," 
	//			 << setprecision(2) << msg->azimuth << "\r" << endl;
		out_file <<setprecision(8) << msg->latitude <<"," << msg->longitude<< "\r" << endl;
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

