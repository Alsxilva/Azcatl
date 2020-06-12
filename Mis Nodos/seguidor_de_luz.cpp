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
#define ppr					1636.0			//Pulsos por revolucion (Encoder)
#define c_llant 			41.469 			//Circunferencia de cada llantas 		[cm]
#define d_llants			23.50000		//Distancia entre llantas contrarias 	[cm]
#define pi 					3.1415926		//Valor de pi
#define intensidad_debil 	341.0			//Definicion de intensidades:
#define intensidad_media 	536.0			//---> Checar documentacion
#define intensidad_alta 	731.0
#define intensidad_muy_alta	876.0
#define intensidad_maxima   1023.0

int intensity, total_steps;
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

int posicion_luz = 5;
float photosensors[8];
bool photosensors_flag;
bool posicion_resistencias[8];		//Banderas para determinar zona de obstaculo
									//Checar documentacion: 0 -> Frente 
									//						1 -> Frente derecha
									//						2 -> Derecha
									//						3 -> Atras derecha
									//						4 -> Atras 
									//						5 -> Atras izquierda
									//						6 -> Izquierda 
									//						7 -> Frente izquierda

void callbackPhotosensors(const std_msgs::Float32MultiArray::ConstPtr &msg){
	int i;

	for(i=0; i<8; i++){
		photosensors[i] = msg -> data[i];		
	}

  	//printf("\n\n----->Datos de PR recibidos correctamente :)");	

  	switch(intensity){
		case 1: 
			for (i=0; i<8; i++){
				if (photosensors[i] < 5){
						printf("\n\n--->ERROR: checar resistencia: %d\n--->Posible falso contacto",i);
						total_steps = 0;	
				}
				else if (photosensors[i] > intensidad_debil && photosensors[i] < intensidad_media)
					posicion_resistencias[i] = true;
				else posicion_resistencias[i] = false;
			}
			break;
		case 2:
			for (i=0; i<8; i++){
					if (photosensors[i] < 5){
						printf("\n\n--->ERROR: checar resistencia: %d\n--->Posible falso contacto",i);
						total_steps = 0;	
					}
					else if (photosensors[i] > intensidad_media && photosensors[i] < intensidad_alta)
						posicion_resistencias[i] = true;
					else posicion_resistencias[i] = false;
				}
			break;
		case 3:
			for (i=0; i<8; i++){
				if (photosensors[i] < 5){
						printf("\n\n--->ERROR: checar resistencia: %d\n--->Posible falso contacto",i);
						total_steps = 0;	
				}
				else if (photosensors[i] > intensidad_alta && photosensors[i] < intensidad_muy_alta)
					posicion_resistencias[i] = true;
				else posicion_resistencias[i] = false;
			}
			break;
		case 4:
			for (i=0; i<8; i++){
				if (photosensors[i] < 5){
						printf("\n\n--->ERROR: checar resistencia: %d\n--->Posible falso contacto",i);
						total_steps = 0;	
				}
				else if (photosensors[i] > intensidad_muy_alta && photosensors[i] < intensidad_maxima)
					posicion_resistencias[i] = true;
				else posicion_resistencias[i] = false;
			}
			break;
		default:
			printf("\n\n---> ERROR: Seleccione una opcion valida");
			total_steps = 0;	
			break;

	}
	
	if 		(photosensors[7] > photosensors[1] & photosensors[7] > photosensors[3] & photosensors[7] > photosensors[5]){
		if (photosensors[0] > photosensors[7])
			posicion_luz = 4;
		else 
			posicion_luz = 3;
		}

	else if (photosensors[1] > photosensors[3] & photosensors[1] > photosensors[5] & photosensors[1] > photosensors[7]){
		if (photosensors[0] > photosensors[1])
			posicion_luz = 4;
		else 
			posicion_luz = 2;
		}

	else if	(photosensors[5] > photosensors[1] & photosensors[5] > photosensors[3] & photosensors[5] > photosensors[7])
		posicion_luz = 1;
	else if (photosensors[3] > photosensors[1] & photosensors[3] > photosensors[5] & photosensors[3] > photosensors[7])
		posicion_luz = 0;

    photosensors_flag = true;
}

void data_photosensors(){
	/*---------------------------Datos obtenidos---------------------------*/
	//Imprime datos obtenidos de las fotoresistencias. 
	//Imprimira "False" si dentro de esa region NO hay una intensidad de luz > intensidad_x
	//Imprimira "True " si dentro de esa region SI hay una intensidad de luz > intensidad_x
	
	printf("\n\n\t\t\tIntensidad ");
		if (intensity == 1)
			printf("Baja: 341 a 536\t\tEstado:");
		else if (intensity == 2)
			printf("Media: 536 a 731\t\tEstado:");
		else if(intensity == 3)
			printf("Alta: 731 a 875\t\tEstado:");
		else if(intensity == 4)
			printf("Muy Alta: 875 a 1023\t\tEstado:");


	for(int i=0; i<8; i++){	
		switch (i){
			case 0:	
				printf("\nFrente:\t\t");
				break;
			case 1:	
				printf("\nFrente derecha:\t");
				break;
			case 2:	
				printf("\nDerecha:\t");
				break;
			case 3:	
				printf("\nAtras derecha:\t");
				break;
			case 4:	
				printf("\nAtras:\t\t");
				break;
			case 5:	
				printf("\nAtras izquierda:");
				break;
			case 6:	
				printf("\nIzquierda:\t");
				break;
			case 7:	
				printf("\nFrente izquierda:");
				break;
		}
		printf("\tFotoresistencia [%d] = %.2f\t\t",i,photosensors[i]);	
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
	float dist = 0;		
	float angle = 0;
	float distancia_usuario, angulo_usuario;
	std_msgs::Float32MultiArray msg;		//Variable que almacena mensaje que vamos a enviar 
	msg.data.resize(2);						//Tamaño de la variable msg

	while(ros::ok()){						//Ejecuta ROS mientras no exista alguna interrupción.
		
/*------------------------------------Maquina de estados para evadir obstaculos------------------------------------*/	

		int step=0;			
		printf( "\033[2J\033[1;1H" );			
		printf("\n--------------------------- Seguidor de Luz ---------------------------");
		printf("\n-------------------------------- START --------------------------------");
		printf("\n\nIntroduzca:\n\n");
		printf("El numero de pasos deseado [entero]: ");
		scanf("%d",&total_steps);
		printf("Angulo a girar por paso [°]: ");
		scanf("%f",&angulo_usuario);
		printf("Distancia a recorrer por paso [cm]: ");
		scanf("%f",&distancia_usuario);
		printf("Intensidad a detectar de luz:\n\t\t1. Debil 30%c-50%c\n\t\t2. Media 50%c-70%c\n\t\t3. Alta  70%c-90%c\n\t\t4. Muy Alta  90%c-100%c\n\t\tOpcion: ",37,37,37,37,37,37,37,37);
		scanf("%d",&intensity);

		step = 0;
		int next_state = 0;				//Inicia maquina de estados

		while (step < total_steps ){

			//-------------------Loop para seguir recibiendo datos-----------------------/
			printf("\n\n--->Esperando a Arduino.");
			while(!photosensors_flag && ros::ok()){
				printf("..");						//Imprimira ".." mientras espera respuesta de Arduino(Fotoresistencias)
				ros::spinOnce();
				rate.sleep();
			}

			photosensors_flag = false;						//Bandera para detectar la conexion con el hokuyo
			printf( "\033[2J\033[1;1H" );
			printf("\n------------------------------Step: %d de: %d------------------------------",step+1,total_steps);

			switch(next_state){
				case 0:
					if (posicion_resistencias[0] | posicion_resistencias[1] | posicion_resistencias[2] | posicion_resistencias[3] |
						posicion_resistencias[4] | posicion_resistencias[5] | posicion_resistencias[6] | posicion_resistencias[7]){			// Sin obstaculo enfrente, avanza
						angle = 0;  
						dist = 0;
                        next_state = 0;
                        step = total_steps;
                        data_photosensors();
                        printf("\n\n----->Llegue a la fuente de luz :D");
                        printf("\n\n----->Llegue a la fuente de lu :D");
                        printf("\n\n----->Llegue a la fuente de :D");
                        printf("\n\n----->Llegue a la fuente  :D");
                        printf("\n\n----->Llegue a la fuent :D");
                        printf("\n\n----->Llegue a la fue :D");
                        printf("\n\n----->Llegue a la f :D");
                        printf("\n\n----->Llegue a la :D");
                        printf("\n\n----->Llegue a  :D");
                        printf("\n\n----->Llegue  :D");
                        ros::Duration(2.0).sleep();
	                }
	                else{
						dist = distancia_usuario;
                        if  (posicion_luz == 4){			// Luz Adelante Izquierda
                  		    data_photosensors();
	               			angle = 0;
                            printf("\n\n----->Luz Adelante al centro \t\t");                          
                            next_state = 7;
                            break;
                        }
                        else if(posicion_luz == 3){			// Luz Adelante Izquierda
                  		    data_photosensors();
	               			angle = 0;
                            printf("\n\n----->Luz Adelante Izquierda \t\t");                          
                            next_state = 1;
                            break;
                        }
                        else if (posicion_luz == 2){		// Luz Adelante Derecha
                            data_photosensors();
                            angle = 0;
                            printf("\n\n----->Luz Adelante Derecha \t\t");
                            next_state = 2;
                            break;
                        }
                        else if (posicion_luz == 1){		// Luz Atras Izquierda
                            data_photosensors();
                            angle = 0;
                            printf("\n\n----->Luz Atras Izquierda \t\t");
                            next_state = 3;
                            break;
                        }
                        else if (posicion_luz == 0){		//Luz Atras Derecha
                        	data_photosensors();
                        	dist = 0;
                        	angle = 0;
                        	printf("\n\n-----> Luz Atras Derecha \t\t");
                        	next_state = 5;
                        	break;
                        }	
                        else if (posicion_luz == 5){
                        	data_photosensors();
                        	dist = 0;
                        	angle = 0;
                        	printf("\n\n-----> ERROR: \n 1.Checar conexion con Arduino\n 2.Habitacion muy oscura\n 3.Checar resistencias");
                        	ros::Duration(1.0).sleep();
                        	break;
                        }
                        else{
                        	data_photosensors();
                        	printf("\n\n-----> Otro caso:\t");
                        	break;
                        }
	                }
					break;

 				/*---------------------------Luz Frente Izquierda---------------------------*/

		        case 1: 			// Giro izquierdo
	                printf("\n\n--->Case 1: giro izquierdo");
	                angle = angulo_usuario;
					dist = 0;
	                next_state = 0;
	                step++;
	                break;

	            /*---------------------------Luz Frente Derecha---------------------------*/

		        case 2: 			// Giro derecho
	                printf("\n\n--->Case 2: giro derecho");
	                angle = -angulo_usuario;
					dist = 0;
	                next_state = 0;
	                step++;
	                break;

	            /*---------------------------Luz Detras Izquierda---------------------------*/

		        case 3: 			// Giro izquierdo
	                printf("\n\n--->Case 3: giro izquierdo");
	                angle = angulo_usuario;
					dist = 0;
	                next_state = 4;
	                break;

		        case 4:				// Giro izquierdo
	                printf("\n\n--->Case 4: giro izquierdo");
	                angle = angulo_usuario;
					dist = 0;
	                next_state = 0;
	                step++;
	                break;

	            /*---------------------------Luz Detras Derecha---------------------------*/

	            case 5: 			// Giro derecho
	                printf("\n\n--->Case 5: giro derecho");
	                angle = -angulo_usuario;
					dist = 0;
	                next_state = 6;
	                break;

		        case 6:				// Giro derecho
	                printf("\n\n--->Case 6: giro derecho");
	                angle = -angulo_usuario;
					dist = 0;
	                next_state = 0;
	                step++;
	                break;

	 			/*---------------------------Luz Frente al Centro---------------------------*/

		        case 7: 		// Avance
	                printf("\n\n--->Case 7: avance");
	                angle = 0;
					dist = distancia_usuario;
	                next_state = 0;
	                step++;
	                break;

 			}

			/*-----------------Inserción de valores: angulo de giro y distancia-----------------*/

			angle *= -1.35;						//Factor de corrección.
			dist  *= 0.784;						//Factor de corrección.

			/*--------------------------Giro--------------------------*/	

			float angulo = angle;
			int anterior_izq = enc[0];				//Variable para determinar cambio orientacion de las llantas
			int actual_izq   = enc[0] + 100;		
			int anterior_der = enc[1];				
			int actual_der   = enc[1] + 100;		

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
				
				//-------------------Loop para seguir recibiendo datos-----------------------/	

				printf("\n\n--->Esperando motores 1.");
				while(anterior_izq != actual_izq & anterior_der != actual_der){
					act = false;
					printf(".");
					while(!act && ros::ok()){				
						printf(".");
						//printf("\nEncoder derecho  : %.4f", enc[1]);
						//printf("\nEncoder izquierdo: %.4f", enc[0]);
						ros::spinOnce();								//Recibe llamadas de vuelta al subscriber
						rate.sleep();									//Espera mientras el mensaje es emitido
					}
					anterior_izq = actual_izq;			//Actualiza valores de los encoders
					actual_izq = enc[0];			//para la siguiente comprobacion
					anterior_der = actual_der;			//Actualiza valores de los encoders
					actual_der = enc[1];			//para la siguiente comprobacion
				}

			//-----------------------------Perfil trapezoidal-----------------------------------*/	

				//-----------------Primera parte del perfil trapezoidal-----------------------*/	

				printf("\n\n--->Dare giro a la derecha");
				printf("\n\tGrados = %.4f\t[°]",angulo_usuario);
				//printf("\n\tGrados = %.4f\t[°] %.4f\t",angulo_usuario, angulo);

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
					
				//-------------------Loop para seguir recibiendo datos-----------------------/	

				printf("\n\n--->Esperando motores 1.");
				while(anterior_izq != actual_izq & anterior_der != actual_der){
					act = false;
					printf(".");
					while(!act && ros::ok()){				
						printf(".");
						//printf("\nEncoder derecho  : %.4f", enc[1]);
						//printf("\nEncoder izquierdo: %.4f", enc[0]);
						ros::spinOnce();								//Recibe llamadas de vuelta al subscriber
						rate.sleep();									//Espera mientras el mensaje es emitido
					}
					anterior_izq = actual_izq;			//Actualiza valores de los encoders
					actual_izq = enc[0];			//para la siguiente comprobacion
					anterior_der = actual_der;			//Actualiza valores de los encoders
					actual_der = enc[1];			//para la siguiente comprobacion
				}

				//---------------------------Perfil trapezoidal---------------------------*/	

				//-----------------Primera parte del perfil trapezoidal-----------------*/	

				printf("\n\n--->Dare giro a la izquierda");
				printf("\n\tGrados = %.4f\t[°]",angulo_usuario);
				//printf("\n\tGrados = %.4f\t[°] %.4f\t",angulo_usuario, angulo);
				
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
			//rate.sleep();							//Duerme al robot, determinado por la frecuencia indicada (20 Hz)
			//ros::Duration(1.0).sleep();				//Duerme al robot por un segundo
			
			/*--------------------------------------Avance--------------------------------------*/
				
			//printf("Comenzando el avance...");
			float av = 10;
			av 	= dist;
			act = false;
			actual_izq   = enc[0] + 100;		
			actual_der   = enc[1] + 100;		

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
				
				//-------------------Loop para seguir recibiendo datos-----------------------/	

				printf("\n\n--->Esperando motores 2.");
				while(anterior_izq != actual_izq & anterior_der != actual_der){
					act = false;
					printf(".");
					while(!act && ros::ok()){				
						printf(".");
						//printf("\nEncoder derecho  : %.4f", enc[1]);
						//printf("\nEncoder izquierdo: %.4f", enc[0]);
						ros::spinOnce();								//Recibe llamadas de vuelta al subscriber
						rate.sleep();									//Espera mientras el mensaje es emitido
					}
					anterior_izq = actual_izq;			//Actualiza valores de los encoders
					actual_izq = enc[0];			//para la siguiente comprobacion
					anterior_der = actual_der;			//Actualiza valores de los encoders
					actual_der = enc[1];			//para la siguiente comprobacion
				}


				//---------------------------Perfil trapezoidal---------------------------*/	

				//-----------------Primera parte del perfil trapezoidal-----------------*/	

				printf("\n\n--->Avanzare");
				printf("\n\tDistancia = %f [cm]",distancia_usuario);
				//printf("\n\tDistancia = %f [cm] %f",distancia_usuario, av);

				//printf("Comenzando movimiento hacia adelante...");
				//printf("Delta: %.4f \n",delta);
				//msg.data[0] = k;					//Desfase de inicio del perfil trapezoidal
				//msg.data[1] = k;
				//pubVelMotor.publish(msg);
				//ros::spinOnce();
				//rate.sleep();

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

				//-------------------Loop para seguir recibiendo datos-----------------------/	

				printf("\n\n--->Esperando motores 2.");
				while(anterior_izq != actual_izq & anterior_der != actual_der){
					act = false;
					printf(".");
					while(!act && ros::ok()){				
						printf(".");
						//printf("\nEncoder derecho  : %.4f", enc[1]);
						//printf("\nEncoder izquierdo: %.4f", enc[0]);
						ros::spinOnce();								//Recibe llamadas de vuelta al subscriber
						rate.sleep();									//Espera mientras el mensaje es emitido
					}
					anterior_izq = actual_izq;			//Actualiza valores de los encoders
					actual_izq = enc[0];			//para la siguiente comprobacion
					anterior_der = actual_der;			//Actualiza valores de los encoders
					actual_der = enc[1];			//para la siguiente comprobacion
				}

				//---------------------------Perfil trapezoidal---------------------------*/

				//-----------------Primera parte del perfil trapezoidal-----------------*/	

				printf("\n\n--->Retrocedere");
				printf("\n\tDistancia = %f [cm]",distancia_usuario);
				//printf("\n\tDistancia = %f [cm] %f",distancia_usuario,av);

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
			//ros::Duration(1.0).sleep();				//Duerme al robot por un segundo
		}
		printf("\n\n\n--------------------------- TERMINE PASOS ----------------------------");
		ros::Duration(2.0).sleep();
	}
	return 0;
}