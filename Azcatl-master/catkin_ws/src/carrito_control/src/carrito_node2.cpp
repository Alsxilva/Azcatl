#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#define BW      0.235
#define Pi 	3.1415926
#define D 	23.50000
#define Cll 	41.469 
#define k 	0.1
#define vm	0.50
#define ppr	3200.0
bool act = false;
volatile float enc[2]={0.0,0.0};
void callbackDer(const std_msgs::Int64::ConstPtr& msg){
	enc[1] = (float)(msg -> data)/ppr;
	printf("%li\n",msg->data);
}
void callbackIzq(const std_msgs::Int64::ConstPtr& msg){
	enc[0] = (float)(msg -> data)/ppr;
	act = true;
	printf("%li\n",msg->data);
}
int main(int argc, char ** argv){
	ros::init(argc, argv, "carrito_node");
	ros::NodeHandle nh;
	ros::Rate rate(20);
	ros::Subscriber encoizq = nh.subscribe("/encoIzq",1,callbackIzq);
	ros::Subscriber encoder = nh.subscribe("/encoDer",1,callbackDer);
	ros::Publisher pubVelMotor = nh.advertise<std_msgs::Float32MultiArray>("/motor_speeds",1);
	ros::Publisher pubColorId = nh.advertise<std_msgs::Int8>("/colorId",1);
	int disOffset;
	std_msgs::Float32MultiArray msg;
	std_msgs::Int8 colorId;
	msg.data.resize(2);
	while(ros::ok()){
		int comp = 1;
		if(comp ==1){
			//leer e interpretar hokuyo
			//hokuyo:important! + camara ? decicion de movimiento
			//hokuyo + camara ? cambio de color objetivo
			colorId.data = 1;
			pubColorId.publish(colorId);
			ros::spinOnce();
			rate.sleep();		

			//msg.data.clear();
			float dist,angle;
			printf("\n Giro Avance: ");
			scanf("%f",&angle);
			angle*=-1.9;
			//msg.data.push_back(aux);
			//printf("\nDistancia:");
			//fflush(stdout);
			scanf("%f",&dist);
			dist *= 0.9;
			//dist = 10.0;
			//msg.data.push_back(aux);
			//pubVelMotor.publish(msg);
		/*------------------------------------Giro-------------------------*/	
			float angulo = angle;
			//printf("Angulo:");
			//scanf("%f",&angulo);
			act = false;
			while(!act && ros::ok()){
				printf("Esperando nuevos datos! %f \n",enc[0]);
			       	//pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();
			}
			float posDer0 = enc[1];
			float posIzq0 = enc[0];
			float posIzqFin = posIzq0 + ((D*Pi*angulo)/(Cll*360.0));
			float posDerFin = posDer0 + ((D*Pi*angulo)/(Cll*360.0));
			float delta  = posIzqFin - posIzq0;
			float limite1 = (delta/3.0)+posIzq0;
			float limite2 = (delta*(2.0/3.0))+posIzq0;
			printf("%f %f \n",enc[0],limite1);
				if(angulo>0){
					while(enc[0]<limite1 && ros::ok()){
					printf("%f %f \n",enc[0],limite1);
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
					printf("%f %f \n",enc[0],posIzqFin);
					//msg.data[0] = vm - (3.0*(vm-k)*(((enc[0]-posIzq0)/(delta))-(2.0/3.0)));
					msg.data[0] = vm - (3.0*(vm-k)*((fabs(enc[0]-posIzq0)/(delta))-(2.0/3.0)));
					msg.data[1] = -msg.data[0];
		       			pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();		
				}

			}else{
				printf("Donde anda el roger?!!!\n");
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
			printf("termine de girar!!!");
			msg.data[0] = 0.0;
			msg.data[1] = 0.0;
			pubVelMotor.publish(msg);
			ros::spinOnce();
			rate.sleep();
			ros::Duration(1.0).sleep();
			printf("Eempiezo a avanzar");
			/*------------------------------Distancia-----------------*/
		
		
			float av = 10;
			av = dist;

			act = false;
			while(!act && ros::ok()){
				printf("Esperando nuevos datos! %f \n",enc[0]);
				ros::spinOnce();
				rate.sleep();
			}
			
			posIzq0 = enc[0];
			posDer0 = enc[1];
			posIzqFin = posIzq0 + (av/Cll);
			posDerFin = posDer0 + (av/Cll);
			delta = av/Cll;
			limite1 = (delta/3.0) + posIzq0;
			limite2 = (delta*(2.0/3.0))+posIzq0;
			printf("%f %f \n",enc[0],limite1);
			if(av>0){
				printf("Adelante->>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
				printf("delta: %.4f \n",delta);
				msg.data[0] = k;
				msg.data[1] = k;
				pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();
				while(enc[0]<limite1  && ros::ok()){
					printf("%f %f %f \n",enc[0],limite1,msg.data[1]);
					float error =fabs(enc[0]- posIzq0); 
					printf("\n**********%f-------------\n",error);
					msg.data[0] = k + (3.0 * (vm - k)*(fabs(enc[0] - posIzq0)/(delta)));
					msg.data[1] = msg.data[0];//k + (3.0 * (vm - k)*(fabs(enc[0] - posIzq0)/(delta)));
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}
				while(enc[0]<limite2 && ros::ok()){
					printf("%f %f %f\n",enc[0],limite2,vm);
					msg.data[0] = vm;
					msg.data[1] = msg.data[0];//vm;
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}
				while(enc[0]<posIzqFin && ros::ok()){
					printf("%f %f %f\n",enc[0],posIzqFin,msg.data[1]);
					msg.data[0] = vm - (3.0*(vm-k)*((fabs(enc[0]-posIzq0)/(delta))-(2.0/3.0)));
					msg.data[1] = msg.data[0];//vm - (3.0*(vm-k)*((fabs(enc[0]-posIzq0)/(delta))-(2.0/3.0)));
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}
			} else{
				printf("Reversa");
				while(enc[0]>limite1 && ros::ok()){
					printf("%f %f %f\n",enc[0],limite1,msg.data[0]);
					//float error = 
					printf("\n ********%f*****+ \n",(enc[0]-posIzq0));
					msg.data[0] =(-k - (3.0 * (vm - k) * (-fabs(enc[0] - posIzq0)/(delta))));
					msg.data[1] = msg.data[0];//(-k - (3.0 * (vm - k) * (-fabs(enc[0] - posIzq0)/(delta))));
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}
				while(enc[0]>limite2 && ros::ok()){
					printf("%f %f %f\n",enc[0],limite2,-vm);
					msg.data[0] = -vm;
					msg.data[1] = msg.data[0];//-vm;
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}
				while(enc[0]>posIzqFin && ros::ok()){
					printf("%f %f %f\n",enc[0],posIzqFin,msg.data[1]);
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
			printf("Enviado\n");
			ros::spinOnce();
			rate.sleep();
		}else{
			//control salvaje
			float Wmax = vm/BW;
			float alpha = 0.1;//Se asume una alpha de modo que la parte lineal tiene a cero
			float beta = 0.01;
			float angulo = 360;
			//Giro
			float posIzq0 = enc[0];
			float posDer0 = enc[1];
			float posIzqFin = posIzq0 + ((D*Pi*angulo)/(Cll*360.0));
			float posDerFin = posDer0 + ((D*Pi*angulo)/(Cll*360.0));
			float error = posIzqFin - enc[0];	
			while(error > 0.0 && ros::ok()){
				msg.data[0] =-(BW*0.5*Wmax)*((2/(1+exp(error/beta)))-1);
				msg.data[1] =(BW*0.5*Wmax)*((2/(1+exp(error/beta)))-1) ;
				pubVelMotor.publish(msg);
				error = posIzqFin - enc[0];
				printf("%f %f\n",msg.data[0],error);
				ros::spinOnce();
				rate.sleep();
			}
			getchar();
		}		
		pubVelMotor.publish(msg);
		printf("Enviado\n");
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
