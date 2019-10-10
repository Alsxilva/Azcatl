/*-------------------------------------------------------------
Silva Guzmán Alejandro
Dr. Savage Carmona
Agosto 2019

/*--------------------------------------------------------------*/

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
	
#define k 			0.1				//Constante de desfase 
#define vm			0.50			//Velocidad maxima [m/s]
#define bw  		0.235			//Ancho de la base del robøt  			[m]
#define ppr			3200.0			//Pulsos por revolucion (Encoder)
#define c_llant 	41.469 			//Circunferencia de cada llantas 		[cm]
#define d_llants	23.50000		//Distancia entre llantas contrarias 	[cm]
#define pi 			3.1415926		//Valor de pi

bool act = false;			 		//Bool para impedir que se avance mientras se gira y viceversa.
volatile float enc[2]={0.0,0.0};

/*------------------------------------Encoders------------------------------------*/	

void callbackDer(const std_msgs::Int64::ConstPtr& msg){
	enc[1] = (float)(msg -> data)/ppr;							//Encoder 1 = Encoder derecho
	printf("%li\n",msg -> data);								//Imprime el valor (float) del encoder
}

void callbackIzq(const std_msgs::Int64::ConstPtr& msg){
	enc[0] = (float)(msg -> data)/ppr;							//Encoder 0 = Encoder izquierdo
	printf("%li\n",msg -> data);								//Imprime el valor (float) del encoder
	act = true;
}

/*------------------------------------Hokuyo------------------------------------*/	

/*--------------------------------Variables--------------------------------*/	


#define maxRange   80			//Rango de lecturas para el hokuyo
#define minRange  -80								
#define intervalRange  10 		//Grados para cada intervalo

float umbral   =  0.2;			//Rango para determinar si se encuentra un obstaculo en determinada posicion					
float promLeft =	0;
float promRight=	0;
float promFront=	0;
float values[2][(maxRange - minRange) / intervalRange];	//Dos arreglos de 16 valores cada uno
	
bool hokuyoFlag = false;
bool central_flag, centralDer_flag, centralIzq_flag, derecha_flag, izquierda_flag;		//Banderas para determinar zona de obstaculo

/*--------------------------------Funcion--------------------------------*/	

void callbackHokuyo(const sensor_msgs::LaserScan::ConstPtr &msg){

	int size = ceil((msg -> angle_max - msg -> angle_min) / msg -> angle_increment);	//Redondea al entero de arriba mas cercano. 
																						//En especifico, redondea a un numero entero el numero de lecturas 
	float sumLeft  = 0;																	//"disponibles" del hokuyoque se mandan desde el nodo "hokuyo_node.py"							
	float sumFront = 0;	
	float sumRight = 0;	
	float values[size];
	
	for(int i = 0 ; i < size ; i++){			//Almacena los valores del hokuyo en un arreglo
		float val = msg -> ranges[i];
		values[i] = msg -> ranges[i];
		
		if(val <= 0.01 && i!=0)					//Evita que se se almacenen valores muy pequeños
			values[i] = 1.2 * values[i-1];
			//float angle = (msg->angle_min+(msg->angle_increment*i))*180.0/3.1415926;
	}

	/*-------------------------------Region central-------------------------------*/

	int centrali[2];
	float centralOut = 0;
	float central[]  = {10,20};		//Rangos para la region central -> Checar documento "Estructura del robot".
		
	centrali[0] = floor((((pi * central[0]) / 180.0) - msg->angle_min) / (msg->angle_increment));	//floor((((pi * 10) / 180) - 1.824262524000698) / 0.006144178739745) = floor(-268.5028657) = -268
	centrali[1] = floor((((pi * central[1]) / 180.0) - msg->angle_min) / (msg->angle_increment));	//floor((((pi * 20) / 180) - 1.824262524000698) / 0.006144178739745) = floor(-240.0966404) = -240
				//Funcion "floor" redondea al entero de abajo mas cercano
	
	for(int i = centrali[0]; i < centrali[1]; i++)	
		centralOut += values[i];
	
	centralOut /= (float)(centrali[1] - centrali[0]);
	central_flag = true;
	
	if(centralOut < umbral)
		central_flag = false;
	
	printf("Central: %.4f\t",centralOut);

	/*---------------------------Region Central-Izquierda---------------------------*/

	int centralIzqi[2];
	float centralIzqOut = 0;
	float centralIzq[]  = {20,30};	//Rangos para la region Izquierda -> Checar documento "Estructura del robot".
	
	centralIzqi[0] =  ceil((((pi * centralIzq[0]) / 180.0) - msg->angle_min) / (msg->angle_increment));	// ceil((((pi * 20) / 180) - 1.824262524000698) / 0.006144178739745) = 	ceil(-240.0966404) = -241; ceil() para no tomar los mismos valores que en la region anterior
	centralIzqi[1] = floor((((pi * centralIzq[1]) / 180.0) - msg->angle_min) / (msg->angle_increment));	//floor((((pi * 30) / 180) - 1.824262524000698) / 0.006144178739745) = floor(-211.6904152) = -211
	
	for(int i = centralIzqi[0]; i < centralIzqi[1] ; i++)	
		centralIzqOut += values[i];

	centralIzqOut /= (float)(centralIzqi[1] - centralIzqi[0]);
	centralIzq_flag = true;
	
	if(centralIzqOut < umbral)
		centralIzq_flag = false;
	
	printf("Central Izq: %.4f\t",centralIzqOut);

	/*---------------------------Region Central-Derecha---------------------------*/

	int centralDeri[2];
	float centralDerOut = 0;
	float centralDer[]  = {0,10};	//Rangos para la region derecha -> Checar documento "Estructura del robot".

	centralDeri[0] = floor((((pi * centralDer[0]) / 180.0) - msg->angle_min) / (msg->angle_increment));	//floor((((pi *  0) / 180) - 1.824262524000698) / 0.006144178739745) = floor(-296.9090909) = -296
	centralDeri[1] = floor((((pi * centralDer[1]) / 180.0) - msg->angle_min) / (msg->angle_increment));	//floor((((pi * 10) / 180) - 1.824262524000698) / 0.006144178739745) = floor(-268.5028647) = -268

	for(int i = centralDeri[0]; i<centralDeri[1] ; i++)	
		centralDerOut += values[i];

	centralDerOut /= (float)(centralDeri[1] - centralDeri[0]);
	centralDer_flag = true;

	if(centralDerOut < umbral)
		centralDer_flag = false;
	
	printf("Central Der: %.4f \t",centralDerOut);

	/*---------------------------Region Izquierda---------------------------*/

	int izquierdai[2];
	float izquierdaOut = 0;
	float izquierda[] ={80,90};		//Rangos para la region izquierda -> Checar documento "Estructura del robot".

	izquierdai[0] = floor((((pi * izquierda[0]) / 180.0)-msg->angle_min) / (msg->angle_increment));		//floor((((pi * 80) / 180) - 1.824262524000698) / 0.006144178739745) = floor(-69.65928898) = -69
	izquierdai[1] = floor((((pi * izquierda[1]) / 180.0)-msg->angle_min) / (msg->angle_increment));		//floor((((pi * 90) / 180) - 1.824262524000698) / 0.006144178739745) = floor(-41.25306374) = -41
	
	for(int i = izquierdai[0]; i < izquierdai[1] ; i++)	
		izquierdaOut += values[i];
	
	izquierdaOut /= (float)(izquierdai[1] - izquierdai[0]);
	izquierda_flag = true;
	
	if(izquierdaOut < umbral)
		izquierda_flag = false;
	
	printf("Izquierda: %.4f\t",izquierdaOut);

	/*---------------------------Region Derecha---------------------------*/

	int derechai[2];
	float derechaOut = 0;
	float derecha[] ={-50,-60}; 	//Rangos para la region derecha -> Checar documento "Estructura del robot".
		
	derechai[0] = floor((((pi * derecha[0]) / 180.0)-msg->angle_min)/(msg->angle_increment));		//floor((((pi * -50) / 180) - 1.824262524000698) / 0.006144178739745) = floor(-438.9402171) = -438	
	derechai[1] = floor((((pi * derecha[1]) / 180.0)-msg->angle_min)/(msg->angle_increment));		//floor((((pi * -60) / 180) - 1.824262524000698) / 0.006144178739745) = floor(-467.3464424) = -467
	
	for(int i = derechai[1]; i < derechai[0] ; i++)	
		derechaOut += values[i];
	
	derechaOut /= (float)(derechai[0] - derechai[1]);
	derecha_flag = true;
	
	if(derechaOut < umbral)
		derecha_flag = false;
	
	printf("Derecha: %f\t",derechaOut);

	/*---------------------------EndRangos---------------------------*/	

	hokuyoFlag = true;

}

int *next_state = 0;		//Inicia maquina de estados

/*------------------------------------Inicio del Main------------------------------------*/	

int main(int argc, char ** argv){
	ros::init(argc, argv, "evasor_obstaculos_hokuyo");	//Inicio de ROS. Necesarios Argc y Argv. Tercer argumento: nombre del nodo.
	ros::NodeHandle nh;						//Manipulador de nodos: nodo publico 
	ros::Rate rate(20);						//Tasa/cantidad de frecuencia 
	
	/*------------------------------------Se instancian subscriptores y publicadores------------------------------------*/	

	ros::Subscriber subHokuyo   	= nh.subscribe("/scan",1,callbackHokuyo);						//Regresa ROS subscriber y avisa a ROS la 'lectura'
	ros::Subscriber encoizq 		= nh.subscribe("/encoIzq",1,callbackIzq);						// de mensajes y como maximo 1 mensaje en el buffer. 
	ros::Subscriber encoder 		= nh.subscribe("/encoDer",1,callbackDer);			
																									
	ros::Publisher pubVelMotor 	= nh.advertise<std_msgs::Float32MultiArray>("/motor_speeds",1);		//Regresa ROS publisher y avisa a ROS la 'publicacion'  				
																									//de mensajes y como maximo 1 mensaje en el buffer.
	int disOffset	;
	float dist,angle;
	std_msgs::Float32MultiArray msg;		//Variable que almacena mensaje que vamos a enviar 
	msg.data.resize(2);						//Tamaño de la variable msg
	
	while(ros::ok()){						//Ejecuta ROS mientras no exista alguna interrupción.
		
		hokuyoFlag = false;					//Bandera para detectar la conexion con el hokuyo
		printf("Iniciando laser Hokuyo...\n");

		while(!hokuyoFlag && ros::ok()){
			ros::spinOnce();
			rate.sleep();
		}

/*------------------------------------Maquina de estados para evadir obstaculos------------------------------------*/	

		int total_steps, step=0;			
		printf("\n Introduzca el numero de pasos deseado:");
		scanf("%d",&total_steps);

		step = 0;
		int state = *next_state;

		while (step < total_steps ){
			switch(state){
				case 0:
					if (!central_flag & !centralDer_flag & !centralIzq_flag){			// Sin obstaculo enfrente, avanza
						angle = 0;
						dist = 15;
                        *next_state = 0;
                        step++;
	                }
	                else{
                        if 		(!izquierda_flag & !central_flag  &  derecha_flag){		// Obstaculo en la derecha
                            *next_state = 1;
                        }
                        else if ( izquierda_flag & !central_flag  & !derecha_flag){		// Obstaculo en la izquierda
                            *next_state = 3;
                        }
                        else if (!izquierda_flag &  central_flag  & !derecha_flag){		// Obstaculo enfrente
                            *next_state = 5;
                        }
	                }
					break;

				/*---------------------------Obstaculo en la derecha---------------------------*/
					
				case 1: 			// Reversa
	                angle = 0;
					dist = -15;
	                *next_state = 2;
               		break;

		        case 2: 			// Giro izquierdo
	                angle = -45;
					dist = 0;
	                *next_state = 0;
	                step++;
	                break;

	            /*---------------------------Obstaculo en la izquierda---------------------------*/

		        case 3: 			// Reversa
	                angle = 0;
					dist = -15;
	                *next_state = 4;
	                break;

		        case 4: 			// Giro derecho
	                angle = 45;
					dist = 0;
	                *next_state = 0;
	                step++;
	                break;

	            /*---------------------------Obstaculo enfrente---------------------------*/

		        case 5: 			// Reversa
	                angle = 0;
					dist = -15;
	                *next_state = 6;
	                break;

		        case 6: 			// Giro izquierdo
	                angle = -45;
					dist = 0;
	                *next_state = 7;
	                break;

		        case 7:				// Giro izquierdo
	                angle = -45;
					dist = 0;
	                *next_state = 8;
	                break;

		        case 8: 			// Avanza
	                angle = 0;
					dist = 15;
	                *next_state = 9;
	                break;

		        case 9: 			// Avanza
	                angle = 0;
					dist = 15;
	                *next_state = 10;
	                break;

		        case 10: 			// Giro derecho
	                angle = 45;
					dist = 0;
	                *next_state = 11;
	                break;

		        case 11: 			// Giro derecho
	                angle = 45;
					dist = 0;
	                *next_state = 0;
	                step++;
	                break;
 			}

			float dist,angle;

			/*--------------------------------Inserción de valores: angulo de giro y distancia--------------------------------*/

			angle *=-1.9;						//Factor de corrección.
			dist  *= 0.9;						//encFactor de corrección.

			/*------------------------------------Giro------------------------------------*/	

			float angulo = angle;
			int anterior = enc[0];			//Variable para determinar si cambio orientacion de la llanta
			int actual   = enc[0] + 100;	//Variable para determinar si cambio orientacion de la llanta

			//-------------------Loop para seguir recibiendo datos-----------------------*/	

			while(anterior != actual){
				act = false;
				while(!act && ros::ok()){				
					//printf("Esperando nuevos datos... %f \n",enc[0]);
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

			if(angulo > 0){		//Para angulo mayor a cero (rotación a la derecha)
				
				//---------------------------Perfil trapezoidal---------------------------------*/	

				//-----------------Primera parte del perfil trapezoidal-----------------------*/	

				while(enc[0] < limite1 && ros::ok()){
					//printf("%f %f \n",enc[0],limite1);
					msg.data[0] = k + (3.0 * (vm - k) * (fabs(enc[0] - posIzq0)/(delta)));		//Las llantas del lado izquierdo giran hacia adelante
				    msg.data[1] = -msg.data[0];													//mientras que las del lado derecho hacia atras.
		       		pubVelMotor.publish(msg);		//Emite mensaje con la velocidad de los motores
					ros::spinOnce();
					rate.sleep();		
				}

				//-----------------Segunda parte del perfil trapezoidal-----------------------*/	

				while(enc[0] < limite2 && ros::ok()){
					msg.data[0] = vm;				//Alcanza velocidad máxima
					msg.data[1] = -vm;	
		       		pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();		
				}

				//-----------------Tercera parte del perfil trapezoidal-----------------------*/	

				while(enc[0] < posIzqFin && ros::ok()){
					printf("%f %f \n",enc[0],posIzqFin);
					msg.data[0] = vm - (3.0 * (vm - k) * ((fabs(enc[0] - posIzq0) / (delta)) - (2.0 / 3.0)));
					msg.data[1] = -msg.data[0];
		       		pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();		
				}

			}else{				//Para angulo menor a cero (rotacion a la izquierda)
				
				//---------------------------Perfil trapezoidal---------------------------------*/	

				//-----------------Primera parte del perfil trapezoidal-----------------------*/	

				while(enc[0] > limite1 && ros::ok()){
					msg.data[0] = (-k - (3.0 * (vm - k) * (-fabs(enc[0] - posIzq0)/(delta))));		//Las llantas del lado derecho giran hacia adelante
					msg.data[1] = - msg.data[0];													//mientras que las del lado izquierdo hacia atras.
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Segunda parte del perfil trapezoidal-----------------------*/	

				while(enc[0] > limite2 && ros::ok()){
					msg.data[0] = -vm;
					msg.data[1] = vm;						//Alcanza velocidad máxima
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Tercera parte del perfil trapezoidal-----------------------*/	

				while(enc[0] > posIzqFin && ros::ok()){
					msg.data[0] = -(((vm - k) / (delta * 0.33333 *-1)) * (enc[0] - posIzqFin)) - k;
					msg.data[1] = -msg.data[0];
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}
			}

			printf("Termine de girar :)");

			/*----------------------------Reset de encoders----------------------------*/	

			for(int i = 0; i<10 ; i++){
				msg.data[0] = 0.0;
				msg.data[1] = 0.0;
				pubVelMotor.publish(msg);
				ros::spinOnce();
				printf("Enviado :)\n");
			}	

			rate.sleep();							//Duerme al robot, determinado por la frecuencia indicada (20 Hz)
			ros::Duration(1.0).sleep();				//Duerme al robot por un segundo
			
			/*--------------------------------------Avance---------------------------------------*/
				
			printf("Comenzando el avance...");
			float av = 10;
			av 	= dist;
			act = false;
			actual = enc[0] + 100;

			//-------------------Loop para seguir recibiendo datos-----------------------*/	
			while(anterior != actual){
				act = false;
				while(!act && ros::ok()){
					//printf("Esperando nuevos datos... %f \n",enc[0]);
					ros::spinOnce();
					rate.sleep();
				}
				anterior = actual;
				actual = enc[0];
			}

			//-------------------Parámetros usados en el perfil trapezoidal de avance positivo-----------------------*/	
			
			posIzq0 = enc[0];								//Posición actual del encoder 0: izquierdo
			posDer0 = enc[1];								//Posición actual del encoder 1: derecho
			delta   = av / c_llant;							//Distancia a avanzar = distancia sobre circunferencia de llantas: numero de vueltas por llanta para alcanzar la distancia deseada.
			posIzqFin = posIzq0 + (av / c_llant);			//Posiciones finales de los encoders
			posDerFin = posDer0 + (av / c_llant);			//a las que se quiere llegar.
			limite1 = (delta / 3.0) + posIzq0;				//Primer segmento que recorre el robot: un tercio de la diferencia de pi y pf mas la posicion inicial 
			limite2 = (delta * (2.0 / 3.0)) + posIzq0;		//Segundo segmento que recorre el robot: dos tercios de la diferencia de pi y pf mas la posicion inicial
			//printf("%f %f \n",enc[0],limite1);

			//---------------------------------------------Avance---------------------------------------------------*/	

			if(av > 0){			//Si la distancia a moverse es positiva (hacia adelante)

				printf("Comenzando movimiento hacia adelante...");
				//printf("Delta: %.4f \n",delta);
				msg.data[0] = k;					//Desfase de inicio del perfil trapezoidal
				msg.data[1] = k;
				pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();

				//---------------------------Perfil trapezoidal---------------------------------*/	

				//-----------------Primera parte del perfil trapezoidal-----------------------*/	

				while(enc[0] < limite1  && ros::ok()){
					//printf("%f %f %f \n",enc[0],limite1,msg.data[1]);
					float error = fabs(enc[0]- posIzq0); 
					printf("\n**********%f-------------\n",error);
					msg.data[0] = k + (3.0 * (vm - k) * (fabs(enc[0] - posIzq0) / (delta)));		//Ambos secciones de llantas, izquierda
					msg.data[1] = msg.data[0];														//y derecha, giraran hacia adelante
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Segunda parte del perfil trapezoidal-----------------------*/	

				while(enc[0] < limite2 && ros::ok()){
					//printf("%f %f %f\n",enc[0],limite2,vm);
					msg.data[0] = vm;
					msg.data[1] = msg.data[0];
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Tercera parte del perfil trapezoidal-----------------------*/	

				while(enc[0] < posIzqFin && ros::ok()){
					//printf("%f %f %f\n",enc[0],posIzqFin,msg.data[1]);
					msg.data[0] = vm - (3.0 * (vm - k) * ((fabs(enc[0] - posIzq0) / (delta)) - (2.0 / 3.0)));
					msg.data[1] = msg.data[0];
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}

			//-----------------------------------------------Retroceso---------------------------------------------------*/	

			}else{		//Si la distancia a moverse es negativa (reversa)

				printf("Comenzando movimiento hacia atras...");

				//---------------------------Perfil trapezoidal---------------------------------*/

				//-----------------Primera parte del perfil trapezoidal-----------------------*/	

				while(enc[0]>limite1 && ros::ok()){
					//printf("%f %f %f\n",enc[0],limite1,msg.data[0]);
					//printf("\n ********%f*****+ \n",(enc[0]-posIzq0));
					msg.data[0] =(-k - (3.0 * (vm - k) * (-fabs(enc[0] - posIzq0) / (delta))));		//Ambas secciones de llantas, izquierda
					msg.data[1] = msg.data[0];														//y derecha, giran hacia atras
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Segunda parte del perfil trapezoidal-----------------------*/	

				while(enc[0] > limite2 && ros::ok()){
					//printf("%f %f %f\n",enc[0],limite2,-vm);
					msg.data[0] = -vm;
					msg.data[1] = msg.data[0];
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Tercera parte del perfil trapezoidal-----------------------*/	

				while(enc[0] > posIzqFin && ros::ok()){
					//printf("%f %f %f\n",enc[0],posIzqFin,msg.data[1]);
					msg.data[0] = -(((vm - k) / (delta * 0.33333 * -1)) * (enc[0] - posIzqFin)) - k;
					msg.data[1] = msg.data[0];
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}
			}

			/*----------------------------Reset de encoders----------------------------*/	

			for(int i = 0; i<10 ; i++){
				msg.data[0] = 0.0;
				msg.data[1] = 0.0;
				pubVelMotor.publish(msg);
				printf("Enviado :)\n");
				ros::spinOnce();
			}

			rate.sleep();							//Duerme al robot, determinado por la frecuencia indicada (20 Hz)
			ros::Duration(1.0).sleep();				//Duerme al robot por un segundo
		}

	}
	return 0;
}