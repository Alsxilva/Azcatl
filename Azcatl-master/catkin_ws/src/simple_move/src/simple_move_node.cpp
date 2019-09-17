#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <simple_move/SimpleMove.h>

ros::Publisher pubCmdVel;
tf::StampedTransform transform;
tf::Quaternion q;
tf::TransformListener *transformListener;

bool simpleMoveCallback(simple_move::SimpleMove::Request &req , simple_move::SimpleMove::Response &resp){
	//Estados estado = me_inicio;
 	ros::Rate rate(30);

	//float velocidadCrucero = 0;
	//int i = 0;
	//bool withDistance;
	
	//float goalx,goaly,goalt;
	//float robotx,roboty,robott;

	bool listo = false;
	//float error;

	//geometry_msgs::Twist twist;
	//geometry_msgs::Twist zero_twist;
	//zero_twist.linear.x = 0;
	//zero_twist.linear.y = 0;
	//zero_twist.angular.z = 0;

	while(ros::ok() && !listo){
		printf("se solicito:\n distancia: %f\n  angulo: %f\n",req.distance,req.angle);	
		listo = true;	
	}
	printf("temine el servicio\n");
}

int main(int argc,char ** argv){
	
	ros::init(argc,argv,"simple_move_node");
	ros::NodeHandle nh;
	ros::Rate rate(30);

	ros::ServiceServer simpleMoveService = nh.advertiseService("simple_move",simpleMoveCallback);

	while(ros::ok){
		
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
} 
