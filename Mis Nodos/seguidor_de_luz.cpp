/*-------------------------------------------------------------
Silva Guzmán Alejandro
Dr. Savage Carmona
Septiembre 2019

/*--------------------------------------------------------------*/

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
	
#define k 					0.1				//Constante de desfase 
#define vm					0.50			//Velocidad maxima [m/s]
#define bw  				0.235			//Ancho de la base del robøt  			[m]
#define ppr					3200.0			//Pulsos por revolucion (Encoder)
#define c_llant 			41.469 			//Circunferencia de cada llantas 		[cm]
#define d_llants			23.50000		//Distancia entre llantas contrarias 	[cm]
#define pi 					3.1415926		//Valor de pi
#define intensidad_debil 	341.0			//Definicion de intensidades:
#define intensidad_media 	536.0			//---> Checar documentacion
#define intensidad_alta 	731.0
#define intensidad_maxima   1023.0

int intensity;
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

/*------------------------------------Fotoresistencias------------------------------------*/

float photosensors[8];
bool photosensors_flag;
bool posicion_resistencias[8];		//Banderas para determinar zona de obstaculo
									//Checar documentacion: 4 -> Frente 
									//						3 -> Frente derecha
									//						2 -> Derecha
									//						1 -> Atras derecha
									//						0 -> Atras 
									//						7 -> Atras izquierda
									//						6 -> Izquierda 
									//						5 -> Frente izquierda

void callbackPhotosensors(const std_msgs::Float32MultiArray::ConstPtr &msg){
	int i;

	for(i=0; i<8; i++){
		photosensors[i] = msg -> data[i];		
	}

  	printf("\n\n----->Datos de PR recibidos correctamente :)");	

  	switch(intensity){
		case 1: 
			for (i=0; i<8; i++){
				if (photosensors[i] < 50)
					printf("\n\n--->ERROR: checar resistencia: ",i,"\n--->Posible falso contacto");
				else if (photosensors[i] > intensidad_debil && photosensors[i] < intensidad_media)
					posicion_resistencias[i] == true;
				else posicion_resistencias[i] == false;
			}
			break;
		case 2:
			for (i=0; i<8; i++){
					if (photosensors[i] < 50)
						printf("\n\n--->ERROR: checar resistencia: ",i,"\n--->Posible falso contacto");
					else if (photosensors[i] > intensidad_media && photosensors[i] < intensidad_alta)
						posicion_resistencias[i] == true;
					else posicion_resistencias[i] == false;
				}
			break;
		case 3:
			for (i=0; i<8; i++){
				if (photosensors[i] < 50)
					printf("\n\n--->ERROR: checar resistencia: ",i,"\n--->Posible falso contacto");
				else if (photosensors[i] > intensidad_alta && photosensors[i] < intensidad_maxima)
					posicion_resistencias[i] == true;
				else posicion_resistencias[i] == false;
			}
			break;
		default:
			printf("\n\n---> ERROR: Seleccione una opcion valida");
			break;

	}

    photosensors_flag = true;
}

void data_photosensors(){
	/*---------------------------Datos obtenidos---------------------------*/
	//Imprime datos obtenidos de las fotoresistencias. 
	//Imprimira "False" si dentro de esa region NO hay una intensidad de luz > intensidad_x
	//Imprimira "True " si dentro de esa region SI hay una intensidad de luz > intensidad_x

	for(int i=0; i<8; i++){	
		switch (i){
			case 0:	
				printf("\nAtras:\t\t\t");
				break;
			case 1:	
				printf("\nAtras derecha:\t\t");
				break;
			case 2:	
				printf("\nDerecha:\t\t");
				break;
			case 3:	
				printf("\nFrente derecha:\t\t");
				break;
			case 4:	
				printf("\nFrente:\t\t\t");
				break;
			case 5:	
				printf("\nFrente izquierda:\t");
				break;
			case 6:	
				printf("\nIzquierda:\t\t");
				break;
			case 7:	
				printf("\nAtras izquierda:\t");
				break;

		}
		printf("\n\tFotoresistencia [%d] = %.2f\t",i,photosensors[i]);	
		printf(posicion_resistencias[i] ? "True" : "False");

	}

}

/*------------------------------------Inicio del Main------------------------------------*/	

int main(int argc, char ** argv){
	ros::init(argc, argv, "seguidor_de_luz");	//Inicio de ROS. Necesarios Argc y Argv. Tercer argumento: nombre del nodo.
	ros::NodeHandle nh;							//Manipulador de nodos: nodo publico 
	ros::Rate rate(20);							//Tasa-cantidad de frecuencia 
	
	/*------------------------------------Se instancian subscriptores y publicadores------------------------------------*/	
							
	ros::Subscriber encoizq 		= nh.subscribe("/encoIzq",1,callbackIzq);					//Regresa ROS subscriber y avisa a ROS la 'lectura'				 
	ros::Subscriber encoder 		= nh.subscribe("/encoDer",1,callbackDer);					//de mensajes y como maximo 1 mensaje en el buffer.				
	ros::Subscriber subPhotoresist 	= nh.subscribe("/photoSensors",1,callbackPhotosensors);			
																									
	ros::Publisher pubVelMotor 	= nh.advertise<std_msgs::Float32MultiArray>("/motor_speeds",1);		//Regresa ROS publisher y avisa a ROS la 'publicacion'  				
																									//de mensajes y como maximo 1 mensaje en el buffer.
	int disOffset;
	float dist, angle;
	float distancia_usuario, angulo_usuario;
	std_msgs::Float32MultiArray msg;		//Variable que almacena mensaje que vamos a enviar 
	msg.data.resize(2);						//Tamaño de la variable msg

	while(ros::ok()){						//Ejecuta ROS mientras no exista alguna interrupción.
		
/*------------------------------------Maquina de estados para evadir obstaculos------------------------------------*/	

		int total_steps, step=0;			
		printf("\n\nIntroduzca:\nEl numero de pasos deseado: ");
		scanf("%d",&total_steps);
		printf("Intensidad a detectar de luz:\n\t\t1. Debil 30%c-50%c\n\t\t2. Media 50%c-70%c\n\t\t3. Alta  70%c-100%c\n\t\tOpcion: ",37,37,37,37,37,37);
		scanf("%d",&intensity);
		printf("Distancia a recorrer por paso [cm]: ");
		scanf("%f",&distancia_usuario);
		printf("Angulo a girar por paso [grados]: ");
		scanf("%f",&angulo_usuario);

		step = 0;
		int next_state = 0;				//Inicia maquina de estados

		while (step < total_steps ){

			photosensors_flag = false;						//Bandera para detectar la conexion con el hokuyo
			printf("\n\nEsperando a Fotoresistencias.");
			
			while(!photosensors_flag && ros::ok()){
				printf("..");						//Imprimira ".." mientras espera respuesta de Arduino(Fotoresistencias)
				ros::spinOnce();
				rate.sleep();
			}

			printf("\n\n--------------Step: %d de: %d--------------", step+1, total_steps);
			switch(next_state){
				case 0:
					if 			(next_state == 0){			// Sin obstaculo enfrente, avanza
						angle = 0;
						dist = 0;
                        next_state = 0;
                        step++;
                        printf("\n\n----->Holi \t\t");
                        data_photosensors();
	                }
	                /*else{
                        if 		(!izquierda_frente_flag & !izquierda_atras_flag & !derecha_frente_flag & !derecha_atras_flag){		// Obstaculo en la derecha
                            next_state = 1;
                            printf("\n\n----->Obstaculo en la derecha \t\t");                          
                  		    data_photosensors();
                        }
                        else if (!izquierda_frente_flag & !izquierda_atras_flag & !derecha_frente_flag & !derecha_atras_flag){		// Obstaculo en la izquierda
                            next_state = 3;
                            printf("\n\n----->Obstaculo en la izquierda \t\t");
                            data_photosensors();
                        }
                        else if (!izquierda_frente_flag & !izquierda_atras_flag & !derecha_frente_flag & !derecha_atras_flag){		// Obstaculo enfrente
                            next_state = 5;
                            printf("\n\n----->Obstaculo enfrente \t\t");
                            data_photosensors();
                        }
                        else printf("\n\n-----> Otro caso:\t");
                        	data_photosensors();
	                }
					break;

				/*---------------------------Obstaculo en la derecha---------------------------*/
					
				/*case 1: 			// Reversa
	                angle = 0;
					dist = -distancia_usuario;
	                next_state = 2;
	                printf("\n\nCase 1");
               		break;

		        case 2: 			// Giro izquierdo
	                angle = -angulo_usuario;
					dist = 0;
	                next_state = 0;
	                step++;
	                printf("\n\nCase 2");
	                break;

	            /*---------------------------Obstaculo en la izquierda---------------------------*/

		        /*case 3: 			// Reversa
	                angle = 0;
					dist = -distancia_usuario;
	                next_state = 4;
	                printf("\n\nCase 3");
	                break;

		        case 4: 			// Giro derecho
	                angle = angulo_usuario;
					dist = 0;
	                next_state = 0;
	                step++;
	                printf("\n\nCase 4");
	                break;

	            /*---------------------------Obstaculo enfrente---------------------------*/

		        /*case 5: 			// Reversa
	                angle = 0;
					dist = -distancia_usuario;
	                next_state = 6;
	                printf("\n\nCase 5");
	                break;

		        case 6: 			// Giro izquierdo
	                angle = -angulo_usuario;
					dist = 0;
	                next_state = 7;
	                printf("\n\nCase 6");
	                break;

		        case 7:				// Giro izquierdo
	                angle = -angulo_usuario;
					dist = 0;
	                next_state = 8;
	                printf("\n\nCase 7");
	                break;

		        case 8: 			// Avanza
	                angle = 0;
					dist = distancia_usuario;
	                next_state = 9;
	                printf("\n\nCase 8");
	                break;

		        case 9: 			// Avanza
	                angle = 0;
					dist = distancia_usuario;
	                next_state = 10;
	                printf("\n\nCase 9");
	                break;

		        case 10: 			// Giro derecho
	                angle = angulo_usuario;
					dist = 0;
	                next_state = 11;
	                printf("\n\nCase 10");
	                break;

		        case 11: 			// Giro derecho
	                angle = angulo_usuario	;
					dist = 0;
	                next_state = 0;
	                printf("\n\nCase 11");
	                step++;
	                break;*/
 			}

			/*-----------------Inserción de valores: angulo de giro y distancia-----------------*/

			angle *=-1.9;						//Factor de corrección.
			dist  *= 0.9;						//encFactor de corrección.

			/*--------------------------Giro--------------------------*/	

			float angulo = angle;
			int anterior = enc[0];				//Variable para determinar si cambio orientacion de la llanta
			int actual   = enc[0] + 100;		//Variable para determinar si cambio orientacion de la llanta

			//-------------------Loop para seguir recibiendo datos-----------------------/	

			while(anterior != actual){
				act = false;
				while(!act && ros::ok()){				
					printf("\n1-->Esperando motores...");
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
					msg.data[0] = k + (3.0 * (vm - k) * (fabs(enc[0] - posIzq0)/(delta)));		//Las llantas del lado izquierdo giran hacia adelante
				    msg.data[1] = -msg.data[0];													//mientras que las del lado derecho hacia atras.
		       		pubVelMotor.publish(msg);		//Emite mensaje con la velocidad de los motores
					ros::spinOnce();
					rate.sleep();		
				}

				//-----------------Segunda parte del perfil trapezoidal-----------------*/	

				while(enc[0] < limite2 && ros::ok()){
					//printf("\n--->Giro Derecha 2");
					msg.data[0] = vm;				//Alcanza velocidad máxima
					msg.data[1] = -vm;	
		       		pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();		
				}

				//-----------------Tercera parte del perfil trapezoidal-----------------*/	

				while(enc[0] < posIzqFin && ros::ok()){
					//printf("\n--->Giro Derecha 3");
					//printf("%f %f \n",enc[0],posIzqFin);
					msg.data[0] = vm - (3.0 * (vm - k) * ((fabs(enc[0] - posIzq0) / (delta)) - (2.0 / 3.0)));
					msg.data[1] = -msg.data[0];
		       		pubVelMotor.publish(msg);
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
					msg.data[0] = (-k - (3.0 * (vm - k) * (-fabs(enc[0] - posIzq0)/(delta))));		//Las llantas del lado derecho giran hacia adelante
					msg.data[1] = - msg.data[0];													//mientras que las del lado izquierdo hacia atras.
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Segunda parte del perfil trapezoidal-----------------*/	

				while(enc[0] > limite2 && ros::ok()){
					//printf("\n--->Giro Izquierda 2");
					msg.data[0] = -vm;
					msg.data[1] = vm;						//Alcanza velocidad máxima
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Tercera parte del perfil trapezoidal-----------------*/	

				while(enc[0] > posIzqFin && ros::ok()){
					//printf("\n--->Giro Izquierda 3");
					msg.data[0] = -(((vm - k) / (delta * 0.33333 *-1)) * (enc[0] - posIzqFin)) - k;
					msg.data[1] = -msg.data[0];
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}
				printf("\n\n---Termine de girar a la izquierda---");
			}

			
			/*----------------------------Reset de encoders----------------------------*/	
			for(int i = 0; i<10 ; i++){
				msg.data[0] = 0.0;
				msg.data[1] = 0.0;
				pubVelMotor.publish(msg);
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
			while(anterior != actual){
				act = false;
				while(!act && ros::ok()){
					printf("\n2-->Esperando motores...");
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
				msg.data[0] = k;					//Desfase de inicio del perfil trapezoidal
				msg.data[1] = k;
				pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();

				//---------------------------Perfil trapezoidal---------------------------*/	

				//-----------------Primera parte del perfil trapezoidal-----------------*/	


				while(enc[0] < limite1  && ros::ok()){
					//printf("%f %f %f \n",enc[0],limite1,msg.data[1]);
					//printf("\n\n--->Avance 1");
					float error = fabs(enc[0]- posIzq0); 
					//printf("\n**********%f-------------\n",error);
					msg.data[0] = k + (3.0 * (vm - k) * (fabs(enc[0] - posIzq0) / (delta)));		//Ambos secciones de llantas, izquierda
					msg.data[1] = msg.data[0];														//y derecha, giraran hacia adelante
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Segunda parte del perfil trapezoidal-----------------*/	

				while(enc[0] < limite2 && ros::ok()){
					//printf("%f %f %f\n",enc[0],limite2,vm);
					//printf("\n--->Avance 2");
					msg.data[0] = vm;
					msg.data[1] = msg.data[0];
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Tercera parte del perfil trapezoidal-----------------*/	

				while(enc[0] < posIzqFin && ros::ok()){
					//printf("%f %f %f\n",enc[0],posIzqFin,msg.data[1]);
					//printf("\n--->Avance 3");
					msg.data[0] = vm - (3.0 * (vm - k) * ((fabs(enc[0] - posIzq0) / (delta)) - (2.0 / 3.0)));
					msg.data[1] = msg.data[0];
					pubVelMotor.publish(msg);
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
					//printf("%f %f %f\n",enc[0],limite1,msg.data[0]);
					//printf("\n ********%f*****+ \n",(enc[0]-posIzq0));
					//printf("\n\n--->Reversa 1");
					msg.data[0] =(-k - (3.0 * (vm - k) * (-fabs(enc[0] - posIzq0) / (delta))));		//Ambas secciones de llantas, izquierda
					msg.data[1] = msg.data[0];														//y derecha, giran hacia atras
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Segunda parte del perfil trapezoidal-----------------*/	

				while(enc[0] > limite2 && ros::ok()){
					//printf("%f %f %f\n",enc[0],limite2,-vm);
					//printf("\n--->Reversa 2");
					msg.data[0] = -vm;
					msg.data[1] = msg.data[0];
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}

				//-----------------Tercera parte del perfil trapezoidal-----------------*/	

				while(enc[0] > posIzqFin && ros::ok()){
					//printf("%f %f %f\n",enc[0],posIzqFin,msg.data[1]);
					//printf("\n--->Reversa 3");
					msg.data[0] = -(((vm - k) / (delta * 0.33333 * -1)) * (enc[0] - posIzqFin)) - k;
					msg.data[1] = msg.data[0];
					pubVelMotor.publish(msg);
					ros::spinOnce();
					rate.sleep();
				}

				printf("\n\n----Termine de retroceder----");
			}

			/*----------------------------Reset de encoders----------------------------*/	
			for(int i = 0; i<10 ; i++){
				msg.data[0] = 0.0;
				msg.data[1] = 0.0;
				pubVelMotor.publish(msg);
				ros::spinOnce();
				rate.sleep();
			}
			rate.sleep();							//Duerme al robot, determinado por la frecuencia indicada (20 Hz)
			ros::Duration(1.0).sleep();				//Duerme al robot por un segundo
		}

	}
	return 0;
}