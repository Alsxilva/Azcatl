/*---------------------------------------------------------------Librerías---------------------------------------------*/
#include <ros.h>
#include <std_msgs/Float32MultiArray.h> 
#include <std_msgs/Int64.h> 
/*--------------------------------------------------------------Constantes----------------------------------------------*/
//Motores
#define PWM_MotorD 7 //Velocidad de los motores Derechos
#define PWM_MotorI 6 //Velocidad de los motores Izquierdos
#define MotorD_A 50 //Direccion de los motores Derechos A
#define MotorD_B 51 //Direccion de los motores Derechos B
#define MotorI_A 48 //52->48 //Direccion de los motores Izquierdos A
#define MotorI_B 49 //53->49//Direccion de los motores Izquierdos B
//Enconders
//Motor 1 --> Enfrente Derecha
#define Enc_M1A 2//Interrup0
#define Enc_M1B 4
//Motor 2 --> Enfrente Izquierda
#define Enc_M2A 3//Interrup1
#define Enc_M2B 5
//Motor 3 --> Medio Derecha
#define Enc_M3A 18//Interrup5
#define Enc_M3B 17
//Motor 4 --> Medio Izquierda
#define Enc_M4A 19//Interrup4
#define Enc_M4B 22
 //Motor 5 -->  Atras Derecha
#define Enc_M5A 20//Interrup3
#define Enc_M5B 23
//Motor 6 --> Atras Izquierda
#define Enc_M6A 21//Interrup2
#define Enc_M6B 24
//Máximos y minimos valores de RPMs que puede conseguir el motor
#define maxPID 66
#define minPID 13
//Máximos y minimos valores de PWM que puede conseguir el motor
#define maxPWM 255
#define minPWM 0
//Sensores de contacto
#define Cont_Der 28 //Sensor de contacto derecho
#define Cont_Izq 30 //Sensor de contacto derecho
/*-------------------------------------------------------------Variables----------------------------------------------------*/
int fotoR[8]= {A0,A1,A2,A3,A4,A5,A6,A7}; //Fotoresistencias
int MD_A=0; //Valor de dirección del MotorD_A
int MD_B=0; //Valor de dirección del MotorD_B
int MI_A=0; //Valor de dirección del MotorI_A
int MI_B=0; //Valor de dirección del MotorI_B
int pwm_D = 0; //Valor de dirección del Motor_D
int pwm_I = 0; //Valor de dirección del Motor_I
float RPM_der = 0; //Revoluciones por minuto calculadas
float RPM_izq = 0; //Revoluciones por minuto calculadas
float velocidad_der = 0; //Velocidad en [m/s]
float velocidad_izq = 0; //Velocidad en [m/s]
int cambioderecha = 0;
float velocidadder = 0;
int cambioizquierda = 0;
float velocidadizq = 0;
float data[8];
//Almacenaiento de pulsos de cada motor
volatile long ContM1=0;
volatile long ContM2=0;
volatile long ContM3=0;
volatile long ContM4=0;
volatile long ContM5=0;
volatile long ContM6=0;

//Variables para el cálculo de la velocidad
unsigned long timeold = 0;  // Tiempo
unsigned int pulsesperturn = 3200; // Número de pulsos por vuelta del motor, por canal = 3267.  //1600 Según yo recuerdo eran 3200 con el RISING
float wheel_diameter = 123.40/1000.0;// Diámetro de la rueda pequeña[mm]  //[m]
int ratio = 1;  // Relación de transmisión 

//Definición de variables para el PID derecho
float error_D = 0;
float errorAnterior_D = 0; 
float P_D = 0; //Acción proporcional
float I_D = 0; //Acción integral
float D_D = 0; //Acción derivativa
float Kp_D = 35.0;//0.4 //Constante proporcional
float Ki_D = 0.1;//0.025; //Constante integral
float Kd_D = 0.1;//2.9; //Constante derivativa

//Definición de variables para el PID izquierdo
float error_I = 0;
float errorAnterior_I = 0; 
float P_I = 0; //Acción Proporcional
float I_I = 0; //Acción integral
float D_I = 0; //Acción /DErivativa
float Kp_I = 40.0;//0.4 //Constante proporcional
float Ki_I = 0.1;//0.025 //Constante integral
float Kd_I = 0.1;//2.9//Constante derivativa
/*
 *  kp    arranque  direccion
    40 ->  bien     bien
*/
long long int encoDerAvg,encoIzqAvg,totalIzq,totalDer;



//Definición de las señales de error, control y la salida del sistema.
float r_t_D = 0 ; //[m/s]
float r_t_I = 0 ; //[m/s]
float r_tder = 0; //[m/s]
float r_tizq = 0; //[m/s]
volatile float y_t_D = 0;
volatile float y_t_I = 0;
float ut_D = 0;
float ut_I = 0;

//Tiempo del ciclo del PID
float cycleTime = 200; //ms
///Tiempo del ciclo del PID en segundo
//¡NO MODIFICAR, SE CALCULA SOLA!
float cycleTimeSeconds = 0; //s  //Variable auxiliar del tiempo del ciclo del PID
/*--------------------------------------------------------Funciones-------------------------------------------------------*/
//Funciones que recibe las velocidades
void speedsCallback(const std_msgs::Float32MultiArray& msg){
  if(msg.data_length < 2){
     r_tizq=0;
     r_tder=0;
     return;
  }else{
    r_tizq=msg.data[0];
    r_tder=msg.data[1];
  }
  velocidadder = fabs(r_tder);
  velocidadizq = fabs(r_tizq); 
}

//Contador de pulsos del encoder 1
void encoderM1aEvent(){
  if(digitalRead(Enc_M1A)==HIGH){
    if(digitalRead(Enc_M1B)==LOW)
      ContM1--; //A=1 & B=0
    else
      ContM1++; //A=1 & B=1
  }else{
    if(digitalRead(Enc_M1B)==LOW)
      ContM1++; //A=0 & B=0
    else
      ContM1--; //A=0 & B=1
  }
}
//Contador de pulsos del encoder 2
void encoderM2aEvent(){
  if(digitalRead(Enc_M2A)==HIGH){
    if(digitalRead(Enc_M2B)==LOW)
      ContM2++; //A=1 & B=0
    else 
      ContM2--; //A=1 & B=1
  }else{
    if(digitalRead(Enc_M2B)==LOW)
      ContM2--; //A=0 & B=0
    else
      ContM2++; //A=0 & B=1
  }
}
//Contador de pulsos del encoder 3
void encoderM3aEvent(){
  if(digitalRead(Enc_M3A)==HIGH){
    if(digitalRead(Enc_M3B)==LOW)
      ContM3--; //A=1 & B=0           
    else 
      ContM3++; //A=1 & B=1              
  }else{
    if(digitalRead(Enc_M3B)==LOW)  
      ContM3++; //A=0 & B=0           
    else 
      ContM3--; //A=0 & B=1               
  }
}
//Contador de pulsos del encoder 4
void encoderM4aEvent(){
  if(digitalRead(Enc_M4A)==HIGH){
    if(digitalRead(Enc_M4B)==LOW)
      ContM4++; //A=1 & B=0
    else  
      ContM4--; //A=1 & B=1                
  }else{
    if(digitalRead(Enc_M4B)==LOW)  
      ContM4--; //A=0 & B=0              
    else
      ContM4++; //A=0 & B=1                  
  }
}
//Contador de pulsos del encoder 5
void encoderM5aEvent(){
  if(digitalRead(Enc_M5A)==HIGH){
    if(digitalRead(Enc_M5B)==LOW)
      ContM5--; //A=1 & B=0               
    else
      ContM5++; //A=1 & B=1                     
  }else{
    if(digitalRead(Enc_M5B)==LOW)
      ContM5++; //A=0 & B=0
    else
      ContM5--; //A=0 & B=1                   
  }
}
//Contador de pulsos del encoder 6
void encoderM6aEvent(){
  if(digitalRead(Enc_M6A)==HIGH){
    if(digitalRead(Enc_M6B)==LOW)
      ContM6++; //A=1 & B=0
    else
      ContM6--; //A=1 & B=1                  
  }else{
    if(digitalRead(Enc_M6B)==LOW)
      ContM6--; //A=0 & B=0               
    else
      ContM6++; //A=0 & B=1                   
  }
}
//Perfil trapezoidal del velocidad
void calculoVelocidad()
{
  if (millis() - timeold >= 10){
      float tol = 2.0;
      int minDer = 0;
    //filtro por el disparo de encoders
      if((abs(ContM2)<minDer && ContM2!=0) || minDer==0)
        minDer = abs(ContM2);
      if((abs(ContM4)<minDer && ContM4!=0) || minDer==0)
        minDer = abs(ContM4);
      if((abs(ContM6)<minDer && ContM6!=0) || minDer==0)
        minDer = abs(ContM6);
        
      int minIzq = 0;
      if((abs(ContM1)<minIzq && ContM1!=0) || minIzq==0)
        minIzq = abs(ContM1);
      if((abs(ContM3)<minIzq && ContM3!=0) || minIzq==0)
        minIzq = abs(ContM3);
      if((abs(ContM5)<minIzq && ContM5!=0) || minIzq==0)
        minIzq = abs(ContM5);
        
      if(abs(ContM1)>tol*minIzq)
        ContM1 = (ContM1<0)?-minIzq:minIzq;   //operador ternario por si el niño no estudió :D
      if(abs(ContM3)>tol*minIzq)
        ContM3 = (ContM3<0)?-minIzq:minIzq;
      if(abs(ContM5)>tol*minIzq)
        ContM5 = (ContM5<0)?-minIzq:minIzq;
      if(abs(ContM2)>tol*minDer)
        ContM2 = (ContM2<0)?-minIzq:minIzq;
      if(abs(ContM4)>tol*minDer)
        ContM4 = (ContM4<0)?-minIzq:minIzq;
      if(abs(ContM6)>tol*minDer)
        ContM6 = (ContM6<0)?-minIzq:minIzq;
      
      encoDerAvg = (ContM2+ContM4+ContM6)/3;
      encoIzqAvg = (ContM1+ContM3+ContM5)/3;
      //periodo maximo de muestreo para la velocidad
      // Uptade every one second, this will be equal to reading frecuency (Hz). // Se actualiza cada segundo
      noInterrupts(); //Don't process interrupts during calculations // Desconectamos la interrupción para que no actué en esta parte del programa.
      //***************************************************************************************
      RPM_der = (1000.0*60.0*(fabs(encoDerAvg) / float(pulsesperturn)))/float(millis() - timeold);
      //RPM_der = 120.0 * (float)abs(ContM4) / (float)pulsesperturn * 1000.0 / float(millis() - timeold) ; // Note that this would be 60*1000/(millis() - timeold)*pulses if the interrupt. happened once per revolution // Calculamos las revoluciones por minuto
      //Ojo con la fórmula de arriba, la variable rpm tiene que ser tipo float porque salen decimales en medio de la operación.
      //velocidad_der = RPM_der *3.1416 * wheel_diameter  / 60; // Cálculo de la velocidad de la rueda en [Km/h]
      velocidad_der = RPM_der * (3.141592 * wheel_diameter)/60.0; //[m/s]
      ContM2 = 0;  // Inicializamos los pulsos
      ContM4 = 0;  // Inicializamos los pulsos
      ContM6 = 0;  // Inicializamos los pulsos
      //***************************************************************************************
      RPM_izq = (1000.0*60.0*(fabs(encoIzqAvg) / float(pulsesperturn)))/float(millis() - timeold);
      //RPM_izq = 120 * abs(ContM3) / pulsesperturn * 1000 / (millis() - timeold) ; // Note that this would be 60*1000/(millis() - timeold)*pulses if the interrupt. happened once per revolution // Calculamos las revoluciones por minuto
      //Ojo con la fórmula de arriba, la variable rpm tiene que ser tipo float porque salen decimales en medio de la operación.
      //velocidad_izq = RPM_izq *3.1416 * wheel_diameter  / 60; // Cálculo de la velocidad de la rueda en [Km/h]
      velocidad_izq = RPM_izq * (3.141592 * wheel_diameter)/60.0; //[m/s]
      ContM1 = 0;  // Inicializamos los pulsos.
      ContM3 = 0;  // Inicializamos los pulsos.
      ContM5 = 0;  // Inicializamos los pulsos.
      
      timeold = millis(); // Almacenamos el tiempo actual.
      interrupts(); // Restart the interrupt processing // Reiniciamos la interrupción    
  }
}
/*-----------------------------------------------------------------Control PID-----------------------------------------------------------------------*/
float ut_D_anterior = 0;
float ut_I_anterior = 0;
float pid_tiempo,pid_tiempo_anterior;
float I_D_suma;
float ev, Rplus;
const float krp = 0.001;
//#define krp 5.0

void pid(){
  pid_tiempo = float(millis())/1000.0;  //[seg]
  //pid velocidades
  ev = velocidad_der-velocidad_izq;
  Rplus = krp*ev;
  //****************PID DERECHO***************************
  y_t_D = velocidad_der;  //real  [m/s]
  r_t_D =velocidadder;    //deseada  [m/s]
  error_D = r_t_D - y_t_D - Rplus;//Cálculo del error
  P_D = Kp_D * error_D;//Acción proporcional
  D_D = Kd_D * (error_D - errorAnterior_D)/(pid_tiempo-pid_tiempo_anterior);
  I_D += Ki_D * 0.5 * (error_D + errorAnterior_D) * (pid_tiempo-pid_tiempo_anterior); 
  ut_D = ut_D_anterior + P_D + D_D + I_D;// + I_D + D_D;//suma de las tres acciones para obtener la señal de control
  if(ut_D > 255)  ut_D = 255;
  else if(ut_D < 0) ut_D = 0;
  
  if(velocidadder == 0){
    ut_D = 0;
    errorAnterior_D = 0;
    error_D = 0;
  }
  
  errorAnterior_D = error_D;
  ut_D_anterior = ut_D; 
  //****************PID IZQUIERDO****************************
  y_t_I = velocidad_izq;
  r_t_I =velocidadizq;
  error_I = r_t_I - y_t_I + Rplus;//Cálculo del error
  P_I = Kp_I * error_I;//Acción proporcional
  //I_I += Ki_I * error_I * cycleTimeSeconds;// /Calculo acción integrativa
  //D_I = Kd_I * (error_I - errorAnterior_I) / cycleTimeSeconds;//cálculo acción derivativa
  ut_I =ut_I_anterior + P_I + I_I + D_I;//suma de las tres acciones para obtener la señal de control
  if(ut_I > 255)  ut_I = 255;
  else if(ut_I < 0) ut_I = 0;
  
  if(velocidadizq == 0){
    ut_I = 0;
    errorAnterior_I = 0;
    error_I = 0;
  }
  
  errorAnterior_I = error_I;
  ut_I_anterior = ut_I;
  
  
  pid_tiempo_anterior = pid_tiempo;
}

//Función que dependiendo del valor de "r_tder" cambia los enables de los motores
void elecciongiro(){
  if (r_tder < 0){
    MD_B = 1;
    MD_A = 0;
  }
  else{
    MD_A = 1;
    MD_B = 0;
  }
  if (r_tizq < 0)
  {
    MI_A = 1;
    MI_B = 0;
  }
  else
  {
    MI_B = 1;
    MI_A = 0;
  }
}

void rpmToPwm(){
  //Convertimos el valor de RPMs a PWM
  pwm_I = ut_I;//map(ut_I, minPID, maxPID, minPWM, maxPWM);
  pwm_D = ut_D;//map(ut_D, minPID, maxPID, minPWM, maxPWM);
}
/*-----------------------------------------------------------------ROS--------------------------------------------------------------*/
ros::NodeHandle nh;
std_msgs::Float32MultiArray photoSensors;
std_msgs::Int64 encoderIzq;
std_msgs::Int64 encoderDer;
ros::Subscriber<std_msgs::Float32MultiArray> subSpeeds("/motor_speeds", &speedsCallback);
ros::Publisher pubSensors("/photoSensors",&photoSensors);
ros::Publisher pubEncoIzq("/encoIzq", &encoderIzq);
ros::Publisher pubEncoDer("/encoDer", &encoderDer);


void setup() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);  //Baudaje de comunicación
  nh.initNode();  //Se inicializa el nodo  
  nh.advertise(pubSensors);
  nh.advertise(pubEncoIzq);
  nh.advertise(pubEncoDer); 
  nh.subscribe(subSpeeds); 
  photoSensors.data_length = 8; 
  
  //Encoders
  pinMode(Enc_M1A,INPUT);
  pinMode(Enc_M1B,INPUT);
  pinMode(Enc_M2A,INPUT);
  pinMode(Enc_M2B,INPUT);
  pinMode(Enc_M3A,INPUT);
  pinMode(Enc_M3B,INPUT);
  pinMode(Enc_M4A,INPUT);
  pinMode(Enc_M4B,INPUT);
  pinMode(Enc_M5A,INPUT);
  pinMode(Enc_M5B,INPUT);
  pinMode(Enc_M6A,INPUT);
  pinMode(Enc_M6B,INPUT);
    
  //Motors
  pinMode(PWM_MotorD,OUTPUT);
  pinMode(PWM_MotorI,OUTPUT);
  pinMode(MotorD_A,OUTPUT);
  pinMode(MotorD_B,OUTPUT);
  pinMode(MotorI_A,OUTPUT);
  pinMode(MotorI_A,OUTPUT);
  
  //Interruptions
  attachInterrupt(0,encoderM1aEvent,CHANGE); //Int0 =Enc_M1A = pin2
  attachInterrupt(1,encoderM2aEvent,CHANGE); //Int1 =Enc_M2A = pin3
  attachInterrupt(5,encoderM3aEvent,CHANGE); //Int5 =Enc_M3A = pin18
  attachInterrupt(4,encoderM4aEvent,CHANGE); //Int4 =Enc_M4A = pin19
  attachInterrupt(3,encoderM5aEvent,CHANGE); //Int3 =Enc_M5A = pin20
  attachInterrupt(2,encoderM6aEvent,CHANGE); //Int2 =Enc_M6A = pin21
  
  //sensors
   pinMode(Cont_Der,INPUT);
   pinMode(Cont_Izq,INPUT);

  //Set of values
  ContM1 = 0;
  ContM2 = 0;
  ContM3 = 0;
  ContM4 = 0;
  ContM5 = 0;
  ContM6 = 0;
  RPM_der = 0;  
  RPM_izq = 0;  
  timeold = 0;
  velocidadder = 0;
  velocidadizq = 0;
  r_tizq=0;
  r_tder=0; 
  cycleTimeSeconds = cycleTime / 1000;
}

void emulaVelocidades(float Vizq,float Vder){
    r_tizq=Vizq;
    r_tder=Vder*1.025;
    velocidadder = fabs(r_tder);
    velocidadizq = fabs(r_tizq);
}

void loop(){
  Serial.println(ContM3);
  //ligth sensors readings
  for(int i=0;i<8;i++)
     data[i]=analogRead(fotoR[i]);
     
   //contact sensors readings
   //data[8]=digitalRead(Cont_Der);
   //data[9]=digitalRead(Cont_Izq);
  //emulaVelocidades(-1*0.387,-1*0.387);  //[m/s] (izq, der)
  elecciongiro();
  calculoVelocidad();
  pid();
  rpmToPwm();
  
  //current speeds readings
  //data[10]=velocidad_izq;
  //data[11]=velocidad_der;
  
  //Direcciones y velocidad de lo motores
  digitalWrite(MotorD_A,MD_A); //Motores Derechos
  digitalWrite(MotorD_B,MD_B);
  digitalWrite(MotorI_A,MI_A);  //Motores Izquierda
  digitalWrite(MotorI_B,MI_B);

  analogWrite(PWM_MotorD, pwm_D);
  analogWrite(PWM_MotorI, pwm_I);
  //Serial.println(pwm_D);

  /*Serial.print(velocidad_izq*10);
  Serial.print(",");
  Serial.println(velocidad_der*10);
                                    */
  photoSensors.data = data;
  totalDer += encoDerAvg;
  totalIzq += encoIzqAvg;
  encoderDer.data = totalDer;
  encoderIzq.data = totalIzq; 
  pubSensors.publish(&photoSensors);
  pubEncoIzq.publish(&encoderIzq);
  pubEncoDer.publish(&encoderDer);
 
  nh.spinOnce();
  delay(30);
}
