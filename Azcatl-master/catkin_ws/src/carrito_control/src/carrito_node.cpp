#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#define BW      0.235
#define Pi 	3.1415926
#define D 	23.50000
#define Cll 	41.469 
#define k 	0.08
#define vm	0.50
#define ppr	3200.0
bool act = false;
volatile float enc[2]={0.0,0.0};
int posColor;
float percentColor;
bool flagPosColor = false;
bool flagPercentColor = false;
void callbackDer(const std_msgs::Int64::ConstPtr& msg){
	enc[1] = (float)(msg -> data)/ppr;
	//printf("%li\n",msg->data);
}
void callbackIzq(const std_msgs::Int64::ConstPtr& msg){
	enc[0] = (float)(msg -> data)/ppr;
	act = true;
	//printf("%li\n",msg->data);
}
void callbackPosColor(const std_msgs::Int8::ConstPtr &msg){
	posColor = msg -> data;
	flagPosColor = true;
}
void callbackPercentColor(const std_msgs::Float32::ConstPtr &msg){
	percentColor = msg -> data;
	flagPercentColor = true;
}
//float distancias[];
#define minRange  -80
#define maxRange  80
#define intervalRange  10 //grados
float values[2][(maxRange-minRange)/intervalRange];

float promLeft=0;
float promRight=0;
float promFront=0;
float umbral = 0.2;
bool centralb,centralDerb,centralIzqb,derechab,izquierdab;
bool hokuyoFlag = false;
void callbackHokuyo(const sensor_msgs::LaserScan::ConstPtr &msg){
	int size = ceil((msg->angle_max-msg->angle_min)/msg->angle_increment);
	float values[size];
	float sumLeft = 0;
	float sumFront = 0;
	float sumRight = 0;	
	for(int i = 0 ; i<size ; i++){
		float val = msg->ranges[i];
		values[i] = msg->ranges[i];
		if(val <= 0.01 && i!=0)
			values[i] = 1.2*values[i-1];
		//float angle = (msg->angle_min+(msg->angle_increment*i))*180.0/3.1415926;
	}

	/*-------------------------------Central---------------------------------------*/
	float central[] ={10,20};
	float centralOut = 0;
	int centrali[2];
	centrali[0] = floor((((3.1415926*central[0])/180.0)-msg->angle_min)/(msg->angle_increment));
	centrali[1] = floor((((3.1415926*central[1])/180.0)-msg->angle_min)/(msg->angle_increment));	
	for(int i = centrali[0]; i<centrali[1] ; i++)	
		centralOut += values[i];
	centralOut /= (float)(centrali[1]-centrali[0]);
	centralb = true;
	if(centralOut < umbral)
		centralb = false;
	//printf("Central: %.4f\t",centralOut);
	/*-------------------------------CentralIzq---------------------------------------*/
	float centralIzq[] ={20,30};
	float centralIzqOut = 0;
	int centralIzqi[2];
	centralIzqi[0] = ceil((((3.1415926*centralIzq[0])/180.0)-msg->angle_min)/(msg->angle_increment));
	centralIzqi[1] = floor((((3.1415926*centralIzq[1])/180.0)-msg->angle_min)/(msg->angle_increment));	
	for(int i = centralIzqi[0]; i<centralIzqi[1] ; i++)	
		centralIzqOut += values[i];
	centralIzqOut /= (float)(centralIzqi[1]-centralIzqi[0]);
	centralIzqb = true;
	if(centralIzqOut < umbral)
		centralIzqb = false;
	//printf("Central Izq: %.4f\t",centralIzqOut);
	/*-------------------------------CentralDer---------------------------------------*/
	float centralDer[] = {0,10};
	float centralDerOut = 0;
	int centralDeri[2];
	centralDeri[0] = floor((((3.1415926*centralDer[0])/180.0)-msg->angle_min)/(msg->angle_increment));
	centralDeri[1] = floor((((3.1415926*centralDer[1])/180.0)-msg->angle_min)/(msg->angle_increment));	
	for(int i = centralDeri[0]; i<centralDeri[1] ; i++){	
		centralDerOut += values[i];
		//printf("%f \n",values[i]);
	}
	centralDerOut /= (float)(centralDeri[1]-centralDeri[0]);
	centralDerb = true;
	if(centralDerOut < umbral)
		centralDerb = false;
	//printf("Central Der: %.4f \t",centralDerOut);
	/*-------------------------------Izquierda---------------------------------------*/
	float izquierda[] ={80,90};
	float izquierdaOut = 0;
	int izquierdai[2];
	izquierdai[0] = floor((((3.1415926*izquierda[0])/180.0)-msg->angle_min)/(msg->angle_increment));
	izquierdai[1] = floor((((3.1415926*izquierda[1])/180.0)-msg->angle_min)/(msg->angle_increment));	
	for(int i = izquierdai[0]; i<izquierdai[1] ; i++)	
		izquierdaOut += values[i];
	izquierdaOut /= (float)(izquierdai[1]-izquierdai[0]);
	izquierdab = true;
	if(izquierdaOut < umbral)
		izquierdab = false;
	//printf("Izquierda: %.4f\t",izquierdaOut);
	/*-------------------------------Derecha---------------------------------------*/
	float derecha[] ={-50,-60};
	float derechaOut = 0;
	int derechai[2];
	derechai[0] = floor((((3.1415926*derecha[0])/180.0)-msg->angle_min)/(msg->angle_increment));
	derechai[1] = floor((((3.1415926*derecha[1])/180.0)-msg->angle_min)/(msg->angle_increment));
	for(int i = derechai[1]; i<derechai[0] ; i++)	
		derechaOut += values[i];
	derechaOut /= (float)(derechai[0]-derechai[1]);
	derechab = true;
	if(derechaOut < umbral)
		derechab = false;
	//printf("Derecha: %f\t",derechaOut);
	/*---------------------------------EndRangos-------------------------------------*/	
	//printf("\n");
	hokuyoFlag = true;
}

int nextState = 0;
int main(int argc, char ** argv){
	ros::init(argc, argv, "carrito_node");
	ros::NodeHandle nh;
	ros::Rate rate(20);
	ros::Subscriber encoizq = nh.subscribe("/encoIzq",1,callbackIzq);
	ros::Subscriber encoder = nh.subscribe("/encoDer",1,callbackDer);
	ros::Subscriber subHokuyo = nh.subscribe("/scan",1,callbackHokuyo);
	ros::Subscriber subPosColor = nh.subscribe("/colorPos",1,callbackPosColor);
	ros::Subscriber subPercentColor = nh.subscribe("/percentColor",1, callbackPercentColor); 
	ros::Publisher pubVelMotor = nh.advertise<std_msgs::Float32MultiArray>("/motor_speeds",1);
	ros::Publisher pubColorId = nh.advertise<std_msgs::Int8>("/colorId",1);
	int disOffset;
	std_msgs::Float32MultiArray msg;
	std_msgs::Int8 colorId;
	msg.data.resize(2);
	float dist,angle;
	while(ros::ok()){
		hokuyoFlag = false;
		printf("Esperando nuevos datos de Hokuyo, y de la cÃmara! \n");
		while(!hokuyoFlag && ros::ok()){
			ros::spinOnce();
			rate.sleep();
		}
		
		while(!flagPercentColor && ros::ok()){
			ros::spinOnce();
			rate.sleep();
		}
		while(!flagPosColor && ros::ok()){
			ros::spinOnce;
			rate.sleep();
		}
		//leer e interpretar hokuyo
		//hokuyo:important! + camara ? decicion de movimiento
		//hokuyo + camara ? cambio de color objetivo
		/*switch(nextState){
			case 0:
				//Izquierda 90Â°
				angle = 85;
				dist = 0;
				nextState = 1;
				break;
			case 1:
				//avanza si esta libre
				angle = 0;
				dist = 5;
				nextState = 2;
				break;
			case 2:
				//regresa 90Â
				angle = -85;
				dist = 0;
				nextState = 0;
				break;
			default:
				nextState = 0;
				angle = 0;
				dist = 0;
		}*/
		
		colorId.data = 1;
		pubColorId.publish(colorId);
		ros::spinOnce();
		rate.sleep();
		printf("%d %f \n",posColor,percentColor);
		/*----------------------------------------MAquina de estados-------------------------------------*/
		switch(nextState){
			case 0:
				if(percentColor>0.85){
					printf("Listo!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
					return 0;
				}
				if(posColor == 0){
					angle= 5;
					dist=0;
					nextState = 0;
				}else if(posColor == 2){
					angle = -8;
					dist = 0;
					nextState = 0;
				}else if(posColor == 1){
					angle = 0;
					dist = 0;
					nextState = 1;
				}else{
					angle = 25;
					dist = 0;
					nextState = 0;
				}
				break;
			case 1:
				if(centralb && centralIzqb && centralDerb){
					angle = 0;
					dist = 15;
					nextState = 0;
				}else if(izquierdab){
					angle = 90;
					dist = 15;
					nextState = 2;
				}else if(derechab){
					 angle = 0;
					 dist = 0;
					 nextState = 3;
				}else{
					angle = 0;
					dist = -15;
					nextState = 0;
				}
				break;
			case 2:
				angle = -90;
				dist = 0;
				nextState = 1;
				break;
			case 3:
				if(centralb && centralIzqb && centralDerb){
					angle = 0;
					dist = 15;
					nextState = 0;
				}else if(derechab){
					angle = -90;
					dist = 15;
					nextState = 4;
				}else {
					angle = 0;
					dist = 0;
					nextState = 0;
				}
				break;	
			case 4:
				angle = 90;
				dist = 0;
				nextState = 3;
				break;
			default :
				nextState = 0;
				angle = 0;
				dist = 0;
		}

		//printf("\n Giro Avance: ");
		//scanf("%f",&angle);
		angle*=-1.9;
		//scanf("%f",&dist);
		dist *= 0.9;
		//printf("avance: %f giro: %f",angle/(-1.9),dist/0.9);
		/*------------------------------------Giro-------------------------*/	
		float angulo = angle;
		//printf("Angulo:");
		//scanf("%f",&angulo);
		int anterior = enc[0];
		int actual = enc[0]+100;
		while(anterior != actual){
			act = false;
			while(!act && ros::ok()){
				//printf("Esperando nuevos datos! %f \n",enc[0]);
		       		//pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();
			}
			anterior = actual;
			actual = enc[0];

		}
		float inicio = enc[0];
		//printf(" inicio-> %f",enc[0]);
		float posDer0 = enc[1];
		float posIzq0 = enc[0];
		float posIzqFin = posIzq0 + ((D*Pi*angulo)/(Cll*360.0));
		float posDerFin = posDer0 + ((D*Pi*angulo)/(Cll*360.0));
		float delta  = posIzqFin - posIzq0;
		float limite1 = (delta/3.0)+posIzq0;
		float limite2 = (delta*(2.0/3.0))+posIzq0;
		//printf("%f %f \n",enc[0],limite1);
		//printf("Delta esperado->%f",delta);
			if(angulo>0){
				while(enc[0]<limite1 && ros::ok()){
				//printf("%f %f \n",enc[0],limite1);
				//msg.data[0] = k + (3.0*(vm-k)*((enc[0]-posIzq0)/(delta)));
				//printf("\n**********%f-------------\n",error);
				msg.data[0] = k + (3.0 * (vm - k)*(fabs(enc[0] - posIzq0)/(delta)));
			        msg.data[1] = -msg.data[0];
	       			pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();		
			}
			while(enc[0]<limite2 && ros::ok()){
				msg.data[0] = vm;
				msg.data[1] = -vm;	
	       			pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();		
			}
			while(enc[0]<posIzqFin && ros::ok()){
				//printf("%f %f \n",enc[0],posIzqFin);
				//msg.data[0] = vm - (3.0*(vm-k)*(((enc[0]-posIzq0)/(delta))-(2.0/3.0)));
				msg.data[0] = vm - (3.0*(vm-k)*((fabs(enc[0]-posIzq0)/(delta))-(2.0/3.0)));
				msg.data[1] = -msg.data[0];
	       			pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();		
			}
		}else{
			while(enc[0]>limite1 && ros::ok()){
				//msg.data[0] = 
				msg.data[0] = (-k - (3.0 * (vm - k) * (-fabs(enc[0] - posIzq0)/(delta))));
				msg.data[1] = - msg.data[0];
				pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();
			}
			while(enc[0]>limite2 && ros::ok()){
				msg.data[0] = -vm;
				msg.data[1] = vm;
				pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();
			}
			while(enc[0]>posIzqFin && ros::ok()){
				//msg.data[0] = 
				msg.data[0] = -(((vm-k)/(delta*0.33333*-1))*(enc[0]-posIzqFin)) - k;
				msg.data[1] = -msg.data[0];
				pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();
			}
		}
	//	printf("termine de girar!!!");
		for(int i = 0; i<10 ; i++){
			msg.data[0] = 0.0;
			msg.data[1] = 0.0;
			pubVelMotor.publish(msg);
			ros::spinOnce();
		}
		rate.sleep();
		ros::Duration(1.0).sleep();
		//printf("Eempiezo a avanzar");
		/*------------------------------Distancia-----------------*/
		actual = enc[0]+100;
		while(anterior != actual){
			act = false;
			while(!act && ros::ok()){
				//printf("Esperando nuevos datos! %f \n",enc[0]);
		       		//pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();
			}
			anterior = actual;
			actual = enc[0];

		}
		float av = 10;
		av = dist;
		posIzq0 = enc[0];
		posDer0 = enc[1];
		posIzqFin = posIzq0 + (av/Cll);
		posDerFin = posDer0 + (av/Cll);
		delta = av/Cll;
		limite1 = (delta/3.0) + posIzq0;
		limite2 = (delta*(2.0/3.0))+posIzq0;
		//printf("%f %f \n",enc[0],limite1);
		if(av>0){
		//	printf("Adelante->>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
	//		printf("delta: %.4f \n",delta);
			msg.data[0] = k;
			msg.data[1] = k;
			pubVelMotor.publish(msg);
			ros::spinOnce();
			rate.sleep();
			while(enc[0]<limite1  && ros::ok()){
		//		printf("%f %f %f \n",enc[0],limite1,msg.data[1]);
				float error =fabs(enc[0]- posIzq0); 
		//		printf("\n**********%f-------------\n",error);
				msg.data[0] = k + (3.0 * (vm - k)*(fabs(enc[0] - posIzq0)/(delta)));
				msg.data[1] = msg.data[0];//k + (3.0 * (vm - k)*(fabs(enc[0] - posIzq0)/(delta)));
				pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();
			}
			while(enc[0]<limite2 && ros::ok()){
	//			printf("%f %f %f\n",enc[0],limite2,vm);
				msg.data[0] = vm;
				msg.data[1] = msg.data[0];//vm;
				pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();
			}
			while(enc[0]<posIzqFin && ros::ok()){
		//		printf("%f %f %f\n",enc[0],posIzqFin,msg.data[1]);
				msg.data[0] = vm - (3.0*(vm-k)*((fabs(enc[0]-posIzq0)/(delta))-(2.0/3.0)));
				msg.data[1] = msg.data[0];//vm - (3.0*(vm-k)*((fabs(enc[0]-posIzq0)/(delta))-(2.0/3.0)));
				pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();
			}
		} else{
			//printf("Reversa");
			while(enc[0]>limite1 && ros::ok()){
		//		printf("%f %f %f\n",enc[0],limite1,msg.data[0]);
				//float error = 
			//	printf("\n ********%f*****+ \n",(enc[0]-posIzq0));
				msg.data[0] =(-k - (3.0 * (vm - k) * (-fabs(enc[0] - posIzq0)/(delta))));
				msg.data[1] = msg.data[0];//(-k - (3.0 * (vm - k) * (-fabs(enc[0] - posIzq0)/(delta))));
				pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();
			}
			while(enc[0]>limite2 && ros::ok()){
			//	printf("%f %f %f\n",enc[0],limite2,-vm);
				msg.data[0] = -vm;
				msg.data[1] = msg.data[0];//-vm;
				pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();
			}
			while(enc[0]>posIzqFin && ros::ok()){
			//	printf("%f %f %f\n",enc[0],posIzqFin,msg.data[1]);
				//msg.data[0] = -(vm - (3.0 * (vm - k) * ((fabs(enc[0] - posIzq0)/(delta)) - (2.0/3.0))));
				msg.data[0] = -(((vm-k)/(delta*0.33333*-1))*(enc[0]-posIzqFin)) - k;
				msg.data[1] = msg.data[0]; //-(vm - (3.0 * (vm - k) * ((fabs(enc[0] - posIzq0)/(delta)) - (2.0/3.0))));
				pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();
			}
			//msg.data[0] = -0.0000001;
			//msg.data[1] = -0.0000001;
		}
		msg.data[0] = 0;
		msg.data[1] = 0;
		//delay();
		pubVelMotor.publish(msg);
		//printf("Enviado\n");
		ros::spinOnce();
		rate.sleep();		
		pubVelMotor.publish(msg);
	//	printf("Enviado\n");
		ros::spinOnce();
		rate.sleep();
		//printf("delta:%f \n",enc[0]-inicio);
	}
	return 0;
}
