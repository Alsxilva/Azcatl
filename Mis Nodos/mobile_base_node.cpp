/*-------------------------------------------------------------
Medrano Albarrán Gerardo
Merino Texta Rogelio de Jesus
Mayo 2019

Revision
Silva Guzmán Alejandro
Agosto 2019

/*--------------------------------------------------------------*/

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h> 
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>

/*----------------------Medidas del robot----------------------*/

#define bw 		0.235		//Ancho de la base 									[m]
#define r 		0.066		//Radio de llanta en metros que no hemos medido		[m]
#define ppm 	7716.6		//Pulsos por metro (Encoder)

/*------------------Variables de los encoders------------------*/

long actualEncoderDer=0;
long actualEncoderIzq=0;
long ultimoEncoderDer=0;
long ultimoEncoderIzq=0;

void callbackEncoderDer(const std_msgs::Int64::ConstPtr &msg){
	actualEncoderDer = msg->data;
}
void callbackEncoderIzq(const std_msgs::Int64::ConstPtr &msg){
	actualEncoderIzq = msg->data;
}

/*--------------------Posición y orientación--------------------*/

double robotX = 0.0;
double robotY = 0.0;
double robotT = 0.0;

/*--------------------Declaración variables ROS--------------------*/

ros::Publisher pubVelIzq;
ros::Publisher pubVelDer;
tf::TransformListener* tf_listener;

void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg){
	float velDer = ((msg->linear.x) - (msg -> angular.z) * bw / 2.0) / (2.0 * 3.14159 * r);
	float velIzq = ((msg->linear.x) + (msg -> angular.z) * bw / 2.0) / (2.0 * 3.14159 * r);

	std_msgs::Float32 msgSend;			//Datos que se mandaran en el mensaje.
	msgSend.data=velIzq;				//Velocidad izquierda
	pubVelIzq.publish(msgSend);
	msgSend.data = velDer;				//Velocidad derecha
	pubVelDer.publish(msgSend);
}

/*------Función que retorna el valor del ángulo dado en un rango de 0° a 180°------*/

float normalizeAngle(float angle){
	while(angle > M_PI)	angle -= 2.0 * M_PI;
	while(angle > M_PI)	angle += 2.0 * M_PI;
	return angle;
}

/*------------------------------Odometría------------------------------*/

void computeOdom(){
	long dif_izq = actualEncoderIzq - ultimoEncoderIzq;
	long dif_der = actualEncoderDer - ultimoEncoderDer;
	ultimoEncoderIzq = actualEncoderIzq;
	ultimoEncoderDer = actualEncoderDer;

	double dist_RecorridaIzq = dif_izq/ppm;									//Porcentaje de avance con relación a un metro de distancia
	double dist_RecorridaDer = dif_der/ppm;									//Calculo de distancias recorridas

	double distX = (dist_RecorridaIzq + dist_RecorridaDer) / 2.0;			//Promedio de distancias recorrido
	double deltaTheta = (dist_RecorridaDer - dist_RecorridaIzq) / bw;		//Angulo del robot

	robotT = normalizeAngle(robotT + deltaTheta);							//Regresa angulo entre -180° y 180°
	robotX += distX * cos(robotT);
	robotY += distX * sin(robotT);
}

int main(int argc, char** argv){

	/*--------------------Declaración de variables ROS--------------------*/

	ros::init(argc,argv,"mobile_base_node");		//Inicia nodo de ROS
	ros::NodeHandle nh;
	ros::Rate rate(30);

	ros::Subscriber subEncoD = nh.subscribe("/encoDer",1,callbackEncoderDer);		//Avisa a ROS la 'lectura' de mensajes
	ros::Subscriber subEncoI = nh.subscribe("/encoIzq",1,callbackEncoderIzq);		//y como maximo 1 mensaje en el buffer
	ros::Subscriber subCmdVel = nh.subscribe("/cmd_vel",1,callbackCmdVel);

	ros::Publisher pubJointState = nh.advertise<sensor_msgs::JointState>("/joint_satates",1);

	//--------Método advertise: método del publisher para avisar al máster que se estarán mandando mensajes--------//

	pubVelIzq = nh.advertise<std_msgs::Float32>("/vel_motor_Izq",1);				//Mensaje para el envio de velocidad izquierda
	pubVelDer = nh.advertise<std_msgs::Float32>("/vel_motor_Der",1);				//Mensaje para el envio de velocidad derecha

	tf_listener = new tf::TransformListener();

	//----Joint hace la conexión con las llantas----//

	std::string jointNames[2] = {"left_wheel_joint_connect","right_wheel_joint_connect"};
	float jointPositions[2] = {0.0,0.0};
	
	sensor_msgs::JointState jointState;
	jointState.name.insert(jointState.name.begin(),jointNames,jointNames+2);
	jointState.position.insert(jointState.position.begin(),jointPositions,jointPositions + 2);

	//----Se instancia el locutor que envía los valores de los joints a RVIZ----//

	tf::TransformBroadcaster br;

	while(ros::ok){

		computeOdom();			//Se realiza la odometría: se setea un vector con los valores leídos
		
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(robotX,robotY,0));

		tf::Quaternion q;		//Quaternion con el valor de ángulo. Quaternion: herramienta de ros para aplicar y calcular rotaciones
		q.setRPY(0,0,robotT);
		transform.setRotation(q);		//Indica el ángulo a girar
		
		br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"odom","base_link"));		// El locutor manda las variable a RVIZ
		
		pubJointState.publish(jointState);		// Se publica el estado de los joints

		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}