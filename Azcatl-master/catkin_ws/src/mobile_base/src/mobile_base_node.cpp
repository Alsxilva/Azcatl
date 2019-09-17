/*-------------------------------------------------------------
Medrano Albarran Gerardo
Merino Texta Rogelio de Jesus

Mayo 2019

/*--------------------------------------------------------------*/

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h> 
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

/*--------VAriables de los encoders----------------*/

long currEncoderDer=0;
long currEncoderIzq=0;
long lastEncoderDer=0;
long lastEncoderIzq=0;
/*-----------medidas del robot---------------------*/
#define BASE_WIDTH 0.235//FALTA MEDIR EN METROS
#define r 0.066 //radio de llanta en metros que no e mos medido
#define ppmet 7716.6
/*------------------pocision y orientacion----------*/
double robotX = 0.0;
double robotY = 0.0;
double robotT = 0.0;

void callbackEncoderDer(const std_msgs::Int64::ConstPtr &msg){
	currEncoderDer = msg->data;
}
void callbackEncoderIzq(const std_msgs::Int64::ConstPtr &msg){
	currEncoderIzq = msg->data;
}

ros::Publisher pubVelIzq;
ros::Publisher pubVelDer;
tf::TransformListener* tf_listener;

void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg){
	float velDer = ((msg->linear.x)-(msg->angular.z)*BASE_WIDTH/2.0)/(2.0*3.14159*r);
	float velIzq = ((msg->linear.x)+(msg->angular.z)*BASE_WIDTH/2.0)/(2.0*3.14159*r);
	std_msgs::Float32 msgSend;
	msgSend.data=velIzq;
	pubVelIzq.publish(msgSend);
	msgSend.data = velDer;
	pubVelDer.publish(msgSend);
}
float normalizeAngle(float angle){
	while(angle > M_PI)	angle -= 2.0 * M_PI;
	while(angle > M_PI)	angle += 2.0 * M_PI;
	return angle;
}
void computeOdom(){
	long dIzq = currEncoderIzq - lastEncoderIzq;
	long dDer = currEncoderDer -lastEncoderDer;
	lastEncoderIzq = currEncoderIzq;
	lastEncoderDer = currEncoderDer;

	double disIzq = dIzq/ppmet;
	double disDer = dDer/ppmet;

	double deltaTheta = (disDer-disIzq)/BASE_WIDTH;
	double distX = (disIzq+disDer)/2.0;

	robotT = normalizeAngle(robotT+deltaTheta);
	robotX += distX*cos(robotT);
	robotY += distX*sin(robotT);
}

int main(int argc, char** argv){
	ros::init(argc,argv,"mobile_base_node");
	ros::NodeHandle nh;
	ros::Rate rate(30);

	ros::Subscriber subEncoD = nh.subscribe("/encoDer",1,callbackEncoderDer);
	ros::Subscriber subEncoI = nh.subscribe("/encoIzq",1,callbackEncoderIzq);
	ros::Subscriber subCmdVel = nh.subscribe("/cmd_vel",1,callbackCmdVel);

	ros::Publisher pubJointState = nh.advertise<sensor_msgs::JointState>("/joint_satates",1);

	pubVelIzq = nh.advertise<std_msgs::Float32>("/vel_motor_Izq",1);
	pubVelDer = nh.advertise<std_msgs::Float32>("/vel_motor_Der",1);

	tf_listener = new tf::TransformListener();

	std::string jointNames[2] = {"left_wheel_joint_connect","right_wheel_joint_connect"};
	float jointPositions[2] = {0.0,0.0};
	
	sensor_msgs::JointState jointState;
	jointState.name.insert(jointState.name.begin(),jointNames,jointNames+2);
	jointState.position.insert(jointState.position.begin(),jointPositions,jointPositions + 2);

	tf::TransformBroadcaster br;

	while(ros::ok){

		computeOdom();	//falta implementar
		
		tf::Transform transform;

		transform.setOrigin(tf::Vector3(robotX,robotY,0));
		tf::Quaternion q;
		q.setRPY(0,0,robotT);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"odom","base_link"));
		
		pubJointState.publish(jointState);

		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
	
