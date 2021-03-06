/*-------------------------------------------------------------
Silva Guzmán Alejandro
Dr. Savage Carmona
Octubre 2019

/*--------------------------------------------------------------*/

#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
//#include <std_msgs/Float32MultiArray.h>	
	
#define k 					0.1				//Constante de desfase 
#define vm					0.50			//Velocidad maxima [m/s]
#define bw  				0.235			//Ancho de la base del robøt  			[m]
#define ppr					3200.0			//Pulsos por revolucion (Encoder)
#define c_llant 			41.469 			//Circunferencia de cada llantas 		[cm]
#define d_llants			23.50000		//Distancia entre llantas contrarias 	[cm]
#define pi 					3.1415926		//Valor de pi

int total_steps;
bool act = false;			 		//Bool para impedir que se avance mientras se gira y viceversa.
volatile float enc[2]={0.0,0.0};


/*------------------------------------Encoders------------------------------------*/	

void callbackDer(const std_msgs::Int64::ConstPtr& msg){
	enc[1] = (float)(msg -> data)/ppr;							//Encoder 1 = Encoder derecho
	//printf("%li\n",msg -> data);								//Imprime el valor (float) del encoder
}

void callbackIzq(const std_msgs::Int64::ConstPtr& msg){
	enc[0] = (float)(msg -> data)/ppr;							//Encoder 0 = Encoder izquierdo
	//printf("%li\n",msg -> data);								//Imprime el valor (float) del encoder
	act = true;
}

/*------------------------------------Inicio del Main------------------------------------*/	

int main(int argc, char ** argv){
	ros::init(argc, argv, "simple_move_2");	//Inicio de ROS. Necesarios Argc y Argv. Tercer argumento: nombre del nodo.
	ros::NodeHandle nh;						//Manipulador de nodos: nodo publico 
	ros::Rate rate(20);						//Tasa-cantidad de frecuencia 
	
	/*------------------------------------Se instancian subscriptores y publicadores------------------------------------*/	
							
	ros::Subscriber encoizq = nh.subscribe("/encoIzq",1,callbackIzq);					//Regresa ROS subscriber y avisa a ROS la 'lectura'				 
	ros::Subscriber encoder = nh.subscribe("/encoDer",1,callbackDer);					//de mensajes y como maximo 1 mensaje en el buffer.				
																									
	//ros::Publisher pubVelMotor 	= nh.advertise<std_msgs::Float32MultiArray>("/motor_speeds",1);		//Regresa ROS publisher y avisa a ROS la 'publicacion'  				
	ros::Publisher pubVelMotorDer 	= nh.advertise<std_msgs::Float32>("/motorR_speed",1);
	ros::Publisher pubVelMotorIzq 	= nh.advertise<std_msgs::Float32>("/motorL_speed",1);
	
	ros::Publisher pubKpPid = nh.advertise<std_msgs::Int8>("/kpPID",1);				//de mensajes y como maximo 1 mensaje en el buffer.
	//ros::Publisher pubKiPid = nh.advertise<std_msgs::Int8>("/kiPID",1);
	//ros::Publisher pubKdPid = nh.advertise<std_msgs::Int8>("/kdPID",1);

	int disOffset;
	float dist, angle, distancia_usuario, angulo_usuario;
	int kp, ki, kd;

	std_msgs::Float32 msgR;		//Variable que almacena mensaje que vamos a enviar 
	std_msgs::Float32 msgL;

	std_msgs::Int8 kpROS;
	std_msgs::Int8 kiROS;
	std_msgs::Int8 kdROS;


	while(ros::ok()){						//Ejecuta ROS mientras no exista alguna interrupción.
		
/*------------------------------------Maquina de estados para evadir obstaculos------------------------------------*/	

		int step = 0;			
		printf("\n\nIntroduzca:\n");
		printf("El numero de pasos deseado: ");
		scanf("%dd",&total_steps);
		printf("Angulo a girar por paso [grados]: ");
		scanf("%f",&angulo_usuario);
		printf("Distancia a recorrer por paso [cm]: ");
		scanf("%f",&distancia_usuario);

		/*-----------Comunicacion con Arduino para publicacion de K's del PID-----------*/

		printf("Los siguientes datos en formato de entero:\n");
		printf("Constante Proporcional (Kp): ");
		scanf("%d",&kp);
		printf("Constante Integral (Ki): ");
		scanf("%d",&ki);
		printf("Constante Derivativa (Kd): ");
		scanf("%d",&kd);

		kpROS.data = kp;
		kiROS.data = ki;	
		kdROS.data = kd;	
		pubKpPid.publish(kpROS);			//Emite mensaje con las constantes para el PID
		//pubKiPid.publish(kdROS);
		//pubKdPid.publish(kiROS);
		//ros::spinOnce();
		//rate.sleep();

		printf("\nEsperando a PID de Arduino.");
			while(!act && ros::ok()){
				printf("..");				//Imprimira ".." mientras espera respuesta de Arduino(PID)
				ros::spinOnce();
				rate.sleep();
			}

		/*----------------------Inicia maquina de estados----------------------*/

		step = 0;
		int next_state = 0;				

		while (step < total_steps ){

			printf("\n\nEsperando a Arduino.");
			while(!act && ros::ok()){
				printf("..");						//Imprimira ".." mientras espera respuesta de Arduino(Fotoresistencias)
				ros::spinOnce();
				rate.sleep();
			}

			printf("\n\n--------------Step: %d de: %d--------------", step+1, total_steps);
			
			angle = angulo_usuario;  
			dist = distancia_usuario;
            step ++;

			/*-----------------Inserción de valores: angulo de giro y distancia-----------------*/
	
			dist  *= 0.784;						//Factor de corrección.

			/*--------------------------Giro--------------------------*/	

			float angulo = angle;
			int anterior = enc[0];				//Variable para determinar si cambio orientacion de la llanta
			int actual   = enc[0] + 100;		//Variable para determinar si cambio orientacion de la llanta

			//-------------------Loop para seguir recibiendo datos-----------------------/	

			printf("\n\n1-->Esperando motores.");
			while(anterior != actual){
				act = false;
				printf(".");
				while(!act && ros::ok()){				
					printf(".");
					//printf("\nEncoder derecho  : %.4f", enc[1]);
					//printf("\nEncoder izquierdo: %.4f", enc[0]);
					ros::spinOnce();								//Recibe llamadas de vuelta al subscriber
					rate.sleep();									//Espera mientras el mensaje es emitido
				}
				anterior = actual;			//Actualiza valores de los encoders
				actual = enc[0];			//para la siguiente comprobacion
			}

			//-------------------Parámetros usados en el perfil trapezoidal de giro-----------------------*/	

			float inicio  = enc[0];														//Posicion inicial del encoder derecho
			float posIzq0 = enc[0];														//Posición actual del encoder 0: izquierdo
			float posDer0 = enc[1];														//Posición actual del encoder 1: derecho
			float posIzqFin = posIzq0 + ((d_llants * pi * angulo) / (c_llant * 360.0));	//Posiciones finales de los encoders
			float posDerFin = posDer0 + ((d_llants * pi * angulo) / (c_llant * 360.0));	//a las que se quiere llegar.
			float delta   = posIzqFin - posIzq0;										//Distancia a avanzar = distancia sobre circunferencia de llantas: número de vueltas por llanta para alcanzar la distancia 
			float limite1 = (delta / 3.0) + posIzq0;									//Primer segmento que recorre el robot: un tercio de la diferencia de pi y pf mas la posición inicial 
			float limite2 = (delta * (2.0 / 3.0)) + posIzq0;							//Segundo segmento que recorre el robot: dos tercios de la diferencia de pi y pf mas la posicion inicial

			//printf("%f %f \n",enc[0],limite1);

			if(angulo > 0.001){		//Para angulo mayor a cero (rotación a la derecha)
				
			//-----------------------------Perfil trapezoidal-----------------------------------*/	

				//-----------------Primera parte del perfil trapezoidal-----------------------*/	

				printf("\n\n--->Dare giro a la derecha");
				printf("\n\tGrados = %.4f\t[°]",angulo_usuario);

				while(enc[0] < limite1 && ros::ok()){
					//printf("%f %f \n",enc[0],limite1);
					//printf("\n\n--->Giro Derecha 1");
					
					//msgL.data = k + (3.0 * (vm - k) * (fabs(enc[0] - posIzq0)/(delta)));		//Las llantas del lado izquierdo giran hacia adelante
				    //msgR.data = -msgL.data;													//mientras que las del lado derecho hacia atras.
		       		//pubVelMotorIzq.publish(msgL);									
					//pubVelMotorDer.publish(msgR);												//Emite mensaje con la velocidad de los motores
					
					msgL.data = k + (3.0 * (vm - k) * (fabs(enc[0] - posIzq0)/(delta)));
					msgR.data = -msgL.data;											

					pubVelMotorIzq.publish(msgL);									
					pubVelMotorDer.publish(msgR);												
					ros::spinOnce();
					rate.sleep();		
				}

				//-----------------Segunda parte del perfil trapezoidal-----------------*/	

				while(enc[0] < limite2 && ros::ok()){
					//printf("\n--->Giro Derecha 2");
					msgL.data = vm;				//Alcanza velocidad máxima
					msgR.data = -vm;	
		       		pubVelMotorIzq.publish(msgL);									
					pubVelMotorDer.publish(msgR);
					ros::spinOnce();
					rate.sleep();		
				}

				//-----------------Tercera parte del perfil trapezoidal-----------------*/	

				while(enc[0] < posIzqFin && ros::ok()){
					//printf("\n--->Giro Derecha 3");
					//printf("%f %f \n",enc[0],posIzqFin);
					msgL.data = vm - (3.0 * (vm - k) * ((fabs(enc[0] - posIzq0) / (delta)) - (2.0 / 3.0)));
					msgR.data = -msgL.data;
		       		pubVelMotorIzq.publish(msgL);									
					pubVelMotorDer.publish(msgR);
					ros::spinOnce();
					rate.sleep();		
				}

				printf("\n\n---Termine de girar a la derecha---");

			}else if (angulo < -0.001){				//Para angulo menor a cero (rotacion a la izquierda)
				
				//---------------------------Perfil trapezoidal---------------------------*/	

				//-----------------Primera parte del perfil trapezoidal-----------------*/	

				printf("\n\n--->Dare giro a la izquierda");
				printf("\n\tGrados = %.4f\t[°]",angulo_usuario);
				
				while(enc[0] > limite1 && ros::ok()){
					//printf("\n\n--->Giro Izquierda 1");
					msgL.data = (-k - (3.0 * (vm - k) * (-fabs(enc[0] - posIzq0)/(delta))));		//Las llantas del lado derecho giran hacia adelante
					msgR.data = - msgL.data;													//mientras que las del lado izquierdo hacia atras.
					pubVelMotorIzq.publish(msgL);									
					pubVelMotorDer.publish(msgR);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Segunda parte del perfil trapezoidal-----------------*/	

				while(enc[0] > limite2 && ros::ok()){
					//printf("\n--->Giro Izquierda 2");
					msgL.data = -vm;
					msgR.data = vm;						//Alcanza velocidad máxima
					pubVelMotorIzq.publish(msgL);									
					pubVelMotorDer.publish(msgR);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Tercera parte del perfil trapezoidal-----------------*/	

				while(enc[0] > posIzqFin && ros::ok()){
					//printf("\n--->Giro Izquierda 3");
					msgL.data = -(((vm - k) / (delta * 0.33333 *-1)) * (enc[0] - posIzqFin)) - k;
					msgR.data = -msgL.data;
					pubVelMotorIzq.publish(msgL);									
					pubVelMotorDer.publish(msgR);
					ros::spinOnce();
					rate.sleep();
				}
				printf("\n\n---Termine de girar a la izquierda---");
			}

			
			/*----------------------------Reset de encoders----------------------------*/	
			for(int i = 0; i<10 ; i++){
				msgL.data = 0.0;
				msgR.data = 0.0;
				pubVelMotorIzq.publish(msgL);									
				pubVelMotorDer.publish(msgR);
				ros::spinOnce();
				rate.sleep();
			}	
			rate.sleep();							//Duerme al robot, determinado por la frecuencia indicada (20 Hz)
			ros::Duration(1.0).sleep();				//Duerme al robot por un segundo
			
			/*--------------------------------------Avance--------------------------------------*/
				
			//printf("Comenzando el avance...");
			float av = 10;
			av 	= dist;
			act = false;
			actual = enc[0] + 100;

			//-------------------Loop para seguir recibiendo datos-------------------//
			
			printf("\n\n2-->Esperando motores/.");
			while(anterior != actual){
				act = false;
				printf(".");
				while(!act && ros::ok()){				
					printf(".");
					//printf("\nEncoder derecho  : %.4f", enc[1]);
					//printf("\nEncoder izquierdo: %.4f", enc[0]);
					ros::spinOnce();
					rate.sleep();
				}
				anterior = actual;
				actual = enc[0];
			}

			//-------------------Parámetros usados en el perfil trapezoidal de avance positivo-------------------*/	
			
			posIzq0 = enc[0];								//Posición actual del encoder 0: izquierdo
			posDer0 = enc[1];								//Posición actual del encoder 1: derecho
			delta   = av / c_llant;							//Distancia a avanzar = distancia sobre circunferencia de llantas: numero de vueltas por llanta para alcanzar la distancia deseada.
			posIzqFin = posIzq0 + (av / c_llant);			//Posiciones finales de los encoders
			posDerFin = posDer0 + (av / c_llant);			//a las que se quiere llegar.
			limite1 = (delta / 3.0) + posIzq0;				//Primer segmento que recorre el robot: un tercio de la diferencia de pi y pf mas la posicion inicial 
			limite2 = (delta * (2.0 / 3.0)) + posIzq0;		//Segundo segmento que recorre el robot: dos tercios de la diferencia de pi y pf mas la posicion inicial
			//printf("%f %f \n",enc[0],limite1);

			//---------------------------------------------Avance---------------------------------------------*/	

			if(av > 0.001){			//Si la distancia a moverse es positiva (hacia adelante)

				printf("\n\n--->Avanzare");
				printf("\n\tDistancia = %f [cm] %f",distancia_usuario, av);

				//printf("Comenzando movimiento hacia adelante...");
				//printf("Delta: %.4f \n",delta);
				msgL.data = k;					//Desfase de inicio del perfil trapezoidal
				msgR.data = k;
				pubVelMotorIzq.publish(msgL);									
				pubVelMotorDer.publish(msgR);
				ros::spinOnce();
				rate.sleep();

				//---------------------------Perfil trapezoidal---------------------------*/	

				//-----------------Primera parte del perfil trapezoidal-----------------*/	


				while(enc[0] < limite1  && ros::ok()){
					//printf("%f %f %f \n",enc[0],limite1,msgR.data);
					//printf("\n\n--->Avance 1");
					float error = fabs(enc[0]- posIzq0); 
					//printf("\n**********%f-------------\n",error);
					msgL.data = k + (3.0 * (vm - k) * (fabs(enc[0] - posIzq0) / (delta)));		//Ambos secciones de llantas, izquierda
					msgR.data = msgL.data;														//y derecha, giraran hacia adelante
					printf("\n\t1.\n\tMizq = %f [cm]\n\tMder = %f [cm]",msgL.data,msgR.data);
					pubVelMotorIzq.publish(msgL);									
					pubVelMotorDer.publish(msgR);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Segunda parte del perfil trapezoidal-----------------*/	

				while(enc[0] < limite2 && ros::ok()){
					//printf("%f %f %f\n",enc[0],limite2,vm);
					//printf("\n--->Avance 2");
					msgL.data = vm;
					msgR.data = msgL.data;
					printf("\n\t2.\n\tMizq = %f [cm]\n\tMder = %f [cm]",msgL.data,msgR.data);
					pubVelMotorIzq.publish(msgL);									
					pubVelMotorDer.publish(msgR);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Tercera parte del perfil trapezoidal-----------------*/	

				while(enc[0] < posIzqFin && ros::ok()){
					//printf("%f %f %f\n",enc[0],posIzqFin,msgR.data);
					//printf("\n--->Avance 3");
					msgL.data = vm - (3.0 * (vm - k) * ((fabs(enc[0] - posIzq0) / (delta)) - (2.0 / 3.0)));
					msgR.data = msgL.data;
					printf("\n\t3.\n\tMizq = %f [cm]\n\tMder = %f [cm]",msgL.data,msgR.data);
					pubVelMotorIzq.publish(msgL);									
					pubVelMotorDer.publish(msgR);
					ros::spinOnce();
					rate.sleep();
				}

				printf("\n\n----Termine de avanzar----");

			//-----------------------------------------------Retroceso-----------------------------------------------*/	

			}else if(av < -0.001){		//Si la distancia a moverse es negativa (reversa)

				//printf("Comenzando movimiento hacia atras...");

				//---------------------------Perfil trapezoidal---------------------------*/

				//-----------------Primera parte del perfil trapezoidal-----------------*/	

				printf("\n\n--->Retrocedere");
				printf("\n\tDistancia = %f [cm]",distancia_usuario);

				while(enc[0]>limite1 && ros::ok()){
					//printf("%f %f %f\n",enc[0],limite1,msgL.data);
					//printf("\n ********%f*****+ \n",(enc[0]-posIzq0));
					//printf("\n\n--->Reversa 1");
					msgL.data =(-k - (3.0 * (vm - k) * (-fabs(enc[0] - posIzq0) / (delta))));		//Ambas secciones de llantas, izquierda
					msgR.data = msgL.data;	
					printf("\n\t1.\n\tMizq = %f [cm]\n\tMder = %f [cm]",msgL.data,msgR.data);
					pubVelMotorIzq.publish(msgL);									
					pubVelMotorDer.publish(msgR);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Segunda parte del perfil trapezoidal-----------------*/	

				while(enc[0] > limite2 && ros::ok()){
					//printf("%f %f %f\n",enc[0],limite2,-vm);
					//printf("\n--->Reversa 2");
					msgL.data = -vm;
					msgR.data = msgL.data;
					printf("\n\t2.\n\tMizq = %f [cm]\n\tMder = %f [cm]",msgL.data,msgR.data);
					pubVelMotorIzq.publish(msgL);									
					pubVelMotorDer.publish(msgR);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Tercera parte del perfil trapezoidal-----------------*/	

				while(enc[0] > posIzqFin && ros::ok()){
					//printf("%f %f %f\n",enc[0],posIzqFin,msgR.data);
					//printf("\n--->Reversa 3");
					msgL.data = -(((vm - k) / (delta * 0.33333 * -1)) * (enc[0] - posIzqFin)) - k;
					msgR.data = msgL.data;
					printf("\n\t3.\n\tMizq = %f [cm]\n\tMder = %f [cm]",msgL.data,msgR.data);
					pubVelMotorIzq.publish(msgL);									
					pubVelMotorDer.publish(msgR);
					ros::spinOnce();
					rate.sleep();
				}

				printf("\n\n----Termine de retroceder----");
			}

			/*----------------------------Reset de encoders----------------------------*/	
			for(int i = 0; i<10 ; i++){
				msgL.data = 0.0;
				msgR.data = 0.0;
				pubVelMotorIzq.publish(msgL);									
				pubVelMotorDer.publish(msgR);
				ros::spinOnce();
				rate.sleep();
			}
			rate.sleep();							//Duerme al robot, determinado por la frecuencia indicada (20 Hz)
			ros::Duration(1.0).sleep();				//Duerme al robot por un segundo
		}

	}
	return 0;
}