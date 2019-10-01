/*-------------------------------------------------------------
Silva Guzmán Alejandro
Revision y formato
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

/*---------------------Variables para deteccion de color---------------------*/	

int posColor;
float percentColor;
bool flagPosColor = false;
bool flagPercentColor = false;

void callbackPosColor(const std_msgs::Int8::ConstPtr &msg){
	posColor = msg -> data;
	flagPosColor = true;
}
void callbackPercentColor(const std_msgs::Float32::ConstPtr &msg){
	percentColor = msg -> data;
	flagPercentColor = true;
}

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
bool centralb,centralDerb,centralIzqb,derechab,izquierdab;

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
	centralb = true;
	
	if(centralOut < umbral)
		centralb = false;
	
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
	centralIzqb = true;
	
	if(centralIzqOut < umbral)
		centralIzqb = false;
	
	printf("Central Izq: %.4f\t",centralIzqOut);

	/*---------------------------Region Central-Derecha---------------------------*/

	int centralDeri[2];
	float centralDerOut = 0;
	float centralDer[]  = {0,10};	//Rangos para la region derecha -> Checar documento "Estructura del robot".

	centralDeri[0] = floor((((pi * centralDer[0]) / 180.0) - msg->angle_min) / (msg->angle_increment));	//floor((((pi *  0) / 180) - 1.824262524000698) / 0.006144178739745) = floor(-296.9090909) = -296
	centralDeri[1] = floor((((pi * centralDer[1]) / 180.0) - msg->angle_min) / (msg->angle_increment));	//floor((((pi * 10) / 180) - 1.824262524000698) / 0.006144178739745) = floor(-268.5028647) = -268

	for(int i = centralDeri[0]; i<centralDeri[1] ; i++){	
		centralDerOut += values[i];
		//printf("%f \n",values[i]);
	}

	centralDerOut /= (float)(centralDeri[1] - centralDeri[0]);
	centralDerb = true;
	if(centralDerOut < umbral)
		centralDerb = false;
	
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
	izquierdab = true;
	
	if(izquierdaOut < umbral)
		izquierdab = false;
	
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
	derechab = true;
	
	if(derechaOut < umbral)
		derechab = false;
	
	printf("Derecha: %f\t",derechaOut);

	/*---------------------------EndRangos---------------------------*/	

	//printf("\n");
	hokuyoFlag = true;

}

int nextState = 0;		//Inicia maquina de estados

/*------------------------------------Inicio del Main------------------------------------*/	

int main(int argc, char ** argv){
	ros::init(argc, argv, "carrito_node");	//Inicio de ROS. Necesarios Argc y Argv. Tercer argumento: nombre del nodo.
	ros::NodeHandle nh;						//Manipulador de nodos: nodo publico 
	ros::Rate rate(20);						//Tasa/cantidad de frecuencia 
	
	/*------------------------------------Se instancian subscriptores y publicadores------------------------------------*/	

	ros::Subscriber subHokuyo   	= nh.subscribe("/scan",1,callbackHokuyo);						//Regresa ROS subscriber y avisa a ROS la 'lectura'
	ros::Subscriber encoizq 		= nh.subscribe("/encoIzq",1,callbackIzq);						// de mensajes y como maximo 1 mensaje en el buffer. 
	ros::Subscriber encoder 		= nh.subscribe("/encoDer",1,callbackDer);						
	ros::Subscriber subPosColor 	= nh.subscribe("/colorPos",1,callbackPosColor);
	ros::Subscriber subPercentColor = nh.subscribe("/percentColor",1, callbackPercentColor); 
	
	ros::Publisher pubColorId 	= nh.advertise<std_msgs::Int8>("/colorId",1);						//Regresa ROS publisher y avisa a ROS la 'publicacion'  
	ros::Publisher pubVelMotor 	= nh.advertise<std_msgs::Float32MultiArray>("/motor_speeds",1);		//de mensajes y como maximo 1 mensaje en el buffer.

	int disOffset;
	float dist,angle;
	std_msgs::Int8 colorId;					//Variable para la deteccion de color con camara
	std_msgs::Float32MultiArray msg;		//Variable que almacena mensaje que vamos a enviar 
	msg.data.resize(2);						//Tamaño de la variable msg
	
	while(ros::ok()){						//Ejecuta ROS mientras no exista alguna interrupción.
		
		hokuyoFlag = false;					//Bandera para detectar la conexion con el hokuyo
		printf("Iniciando laser Hokuyo y camara...\n");

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

		//Leer e interpretar hokuyo
		//hokuyo:important! + camara ? decision de movimiento
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

		/*--------------------Maquina de estados para el seguimiento de color--------------------*/
		
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

		int comp = 2;						//Variable comp: decide entre pedir valores o usar datos predeterminados.

		if(comp ==1){						//Pide valores de distancia y giro que hará el robot.

			float dist,angle;

			/*--------------------------------Inserción de valores: angulo de giro y distancia--------------------------------*/

			//fflush(stdin);					//Limpia buffer de entrada para evitar basura indeseada.
			printf("\n Angulo de giro(°):");
			scanf("%f",&angle);
			angle*=-1.9;						//Factor de corrección.
			//fflush(stdout);					//Limpia buffer de salida para evitar basura indeseada.

			//fflush(stdin);
			printf("\n Distancia de avance(m):");
			scanf("%f",&dist);
			dist *= 0.9;						//encFactor de corrección.
			//fflush(stdout);

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

			float inicio = enc[0];														//Posicion inicial del encoder derecho
			float posIzq0 = enc[0];														//Posición actual del encoder 0: izquierdo
			float posDer0 = enc[1];														//Posición actual del encoder 1: derecho
			float delta   = posIzqFin - posIzq0;										//Distancia a avanzar = distancia sobre circunferencia de llantas: número de vueltas por llanta para alcanzar la distancia 
			float limite1 = (delta / 3.0) + posIzq0;									//Primer segmento que recorre el robot: un tercio de la diferencia de pi y pf mas la posición inicial 
			float limite2 = (delta * (2.0 / 3.0)) + posIzq0;							//Segundo segmento que recorre el robot: dos tercios de la diferencia de pi y pf mas la posicion inicial
			float posIzqFin = posIzq0 + ((d_llants * pi * angulo) / (c_llant * 360.0));	//Posiciones finales de los encoders
			float posDerFin = posDer0 + ((d_llants * pi * angulo) / (c_llant * 360.0));	//a las que se quiere llegar.

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

		}else{

			/*----------------------------Control de posición con leyes de control vistas en el diplomado----------------------------*/

			float Wmax = vm / bw;
			float beta = 0.01;
			float alpha = 0.1;		//Se asume una alpha de modo que la parte lineal tiene a cero
			float angulo = 360;
			
			//Giro
			float posIzq0 = enc[0];
			float posDer0 = enc[1];
			float posIzqFin = posIzq0 + ((d_llants * pi * angulo) / (c_llant * 360.0));
			float posDerFin = posDer0 + ((d_llants * pi * angulo) / (c_llant * 360.0));
			float error = posIzqFin - enc[0];	

			while(error > 0.0 && ros::ok()){
				msg.data[0] = -(bw * 0.5 * Wmax) * ((2 / (1 + exp(error / beta)))-1);
				msg.data[1] =  (bw * 0.5 * Wmax) * ((2 / (1 + exp(error / beta)))-1);
				pubVelMotor.publish(msg);
				error = posIzqFin - enc[0];
				printf("%f %f\n",msg.data[0],error);
				ros::spinOnce();
				rate.sleep();
			} 
			getchar();
		}	
	}
	return 0;
}