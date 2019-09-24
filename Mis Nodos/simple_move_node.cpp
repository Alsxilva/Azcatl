/*-------------------------------------------------------------
Silva Guzmán Alejandro
Revisión
Agosto 2019

/*--------------------------------------------------------------*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <simple_move/SimpleMove.h>

ros::Publisher pubCmdVel;
tf::StampedTransform transform;
tf::Quaternion q;
tf::TransformListener *transformListener;

/*--------------------------Función que llama al servicio--------------------------*/

bool simpleMoveCallback(simple_move::SimpleMove::Request &req , simple_move::SimpleMove::Response &resp){
 	ros::Rate rate(30);
	bool listo = false;

/*----------Comprueba la conexión y el correcto intercambio de mensajes----------*/

	while(ros::ok() && !listo){
		printf("Se solicito:\n Distancia: %f\n Angulo: %f\n",req.distance,req.angle);	
		listo = true;	
	}
	printf("Termine el servicio :)\n");
}

int main(int argc,char ** argv){
	
	ros::init(argc,argv,"simple_move_node");
	ros::NodeHandle nh;
	ros::Rate rate(30);

	// Se instancía el servicio "simpleMoveService" del nodehandle que a su vez
	// invoca la función simpleMoveCallback cuando se detona el servicio 

	ros::ServiceServer simpleMoveService = nh.advertiseService("simple_move",simpleMoveCallback);

/*------------------------------Nodo en constante escucha------------------------------*/

	while(ros::ok){
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
} 
