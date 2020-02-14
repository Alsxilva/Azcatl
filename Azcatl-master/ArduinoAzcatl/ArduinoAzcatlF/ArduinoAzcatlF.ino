/*-------------------------------------------------------------
Silva Guzmán Alejandro ->Formato
Dr. Savage Carmona
Septiembre 2019
/*--------------------------------------------------------------*/

/*----------------------Librerías----------------------*/
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int64.h>  
#include <std_msgs/Int16MultiArray.h> 
#include <std_msgs/Float32.h> 

/*--------------------------Pines--------------------------*/

/*----------------------Motores----------------------*/
#define MotorD_A    50  //Pin de los motores Derechos A
#define MotorD_B    51  //Pin de los motores Derechos B
#define MotorI_A    48  //Pin de los motores Izquierdos A: 52->48
#define MotorI_B    49  //Pin de los motores Izquierdos B: 53->49
#define PWM_MotorD   7  //Pin de los motores Derechos
#define PWM_MotorI   6  //Pin de los motores Izquierdos

/*----------------------Encoders----------------------*/
//Cada motor en llanta cuenta con variables en encoders. Ambar nos sirven para determinar las 2 direcciones
//posibles del giro de llanta.

//Motor 1 --> Enfrente Derecha
#define Enc_M1A 2     //Interrup0
#define Enc_M1B 4

//Motor 2 --> Enfrente Izquierda
#define Enc_M2A 3     //Interrup1
#define Enc_M2B 5

//Motor 3 --> Medio Derecha
#define Enc_M3A 18    //Interrup5
#define Enc_M3B 17

//Motor 4 --> Medio Izquierda
#define Enc_M4A 19    //Interrup4
#define Enc_M4B 22

 //Motor 5 -->  Atras Derecha
#define Enc_M5A 20    //Interrup3
#define Enc_M5B 23

//Motor 6 --> Atras Izquierda
#define Enc_M6A 21    //Interrup2
#define Enc_M6B 24

/*----------------------Revoluciones/min----------------------*/
//Máximos y minimos valores de RPM que puede conseguir el motor
#define maxPID 66
#define minPID 13

//----------------------Pulse Width Modulation----------------------
//Máximos y minimos valores de PWM que puede conseguir el motor (entiendo 8 bits usados)
#define maxPWM 255
#define minPWM 0

/*----------------------Sensores infrarojo----------------------*/
#define Cont_Der 28   //Sensor de contacto derecho
#define Cont_Izq 30   //Sensor de contacto derecho

/*----------------------Variables----------------------*/

int MD_A = 0;       //Valor de dirección del MotorD_A
int MD_B = 0;       //Valor de dirección del MotorD_B
int MI_A = 0;       //Valor de dirección del MotorI_A
int MI_B = 0;       //Valor de dirección del MotorI_B
int pwm_D = 0;      //Valor de dirección del Motor_D
int pwm_I = 0;      //Valor de dirección del Motor_I
int cambioderecha = 0;
int cambioizquierda = 0;
int fotoR[8] = {A0,A1,A2,A3,A4,A5,A6,A7};  //Fotoresistencias

int data[8];              //Arreglo que almacenara las lecturas de las fotoresistencias.
float RPM_der = 0;        //Revoluciones por minuto calculadas
float RPM_izq = 0;        //Revoluciones por minuto calculadas
float velocidadder = 0;
float velocidadizq = 0;
float velocidad_R = 0;  //Velocidad en [m/s]
float velocidad_L = 0;  //Velocidad en [m/s]

//----------------------Almacenaiento de pulsos de cada motor----------------------
volatile long ContM1=0;
volatile long ContM2=0;
volatile long ContM3=0;
volatile long ContM4=0;
volatile long ContM5=0;
volatile long ContM6=0;

//----------------------Variables para el cálculo de la velocidad----------------------

int ratio = 1;                          //Relación de transmisión 
float wheel_diameter  = 123.40/1000.0;  //Diámetro de la rueda pequeña[mm] //
float wheel_circunference = wheel_diameter * 3.141592;    //Circunferencia de la rueda
unsigned int pulsesPerWheel_Spin = 3200;      //Número de pulsos por vuelta desde el encoder de cada motor, por canal = 3267.  //Checar si eran 1600. Recuerdo eran 3200 con el RISING.
unsigned long timeold = 0;                    //Variable de tiempo antigua relativa
long long int encoDerAvg, encoIzqAvg, totalIzq, totalDer;

//------Definición de las señales de error, control y la salida del sistema para el control del PID------

float inputUtFunc_R = 0;             //Salida del accionador, pasando ya por PID derecho en [m/s]
float inputUtFunc_L = 0;            //          ""                ""           izquierdo en [m/s]
float referenceFunc_R = 0;          //Entrada al PID derecho en [m/s]
float referenceFunc_L = 0;          //      ""       izquierdo en [m/s]
float referenceFuncValue_R = 0;     //Valor de entrada al PID derecho en [m/s] deseado desde usuario
float referenceFuncValue_L = 0;     //          ""            izquierdo en [m/s] 
float kpRef = 0;
float kdRef = 0;
float kiRef = 0;      
float kp = 0;
float ki = 0;
float kd = 0;               

volatile float outputFunc_L  = 0;
volatile float outputFunc_R = 0;

//----------------------Funcion que recibe las velocidades----------------------

void speedLCallback(const std_msgs::Float32& msgL){
  referenceFuncValue_L = msgL.data;     //Valores que vienen desde nodo del algoritmo de comportamiento                                mportamiento
  velocidadizq = fabs(referenceFuncValue_L); 
}
void speedRCallback(const std_msgs::Float32& msgR){
  referenceFuncValue_R = msgR.data;     //Valores que vienen desde nodo del algoritmo de comportamiento                     mportamiento
  velocidadder = fabs(referenceFuncValue_R); 
}

//----------------------Funciones que reciben las constantes para el PID----------------------

void kpCallback(const std_msgs::Int8& msg){
  kp = msg.data;        
  printf("%f",kp);                  
}
void kdCallback(const std_msgs::Int8& msg){
  kd = msg.data;        
  printf("%f",kd);                  
}
void kiCallback(const std_msgs::Int8& msg){
  ki = msg.data;        
  printf("%f",ki);                  
}

//----------------------Contador de pulsos del encoder 1----------------------
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

//----------------------Contador de pulsos del encoder 2----------------------
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

//----------------------Contador de pulsos del encoder 3----------------------
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

//----------------------Contador de pulsos del encoder 4----------------------
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

//----------------------Contador de pulsos del encoder 5----------------------
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

//----------------------Contador de pulsos del encoder 6----------------------
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

//----------------------Perfil trapezoidal de velocidad----------------------

void calculoVelocidad(){
  if (millis() - timeold >= 10){
    
    //Arduino function "milis()" shows the time in miliseconds since the Arduino board started 
    //running the code itself
      
      float tol = 2.0;

    //----------------------Filtro por el disparo de encoders----------------------

      //----------Asignacion entre valores minimos de los encoders por cada lado de llantas----------
      int minDer = 0;
      if((abs(ContM2) < minDer && ContM2 != 0) || minDer == 0)
        minDer = abs(ContM2);
      if((abs(ContM4) < minDer && ContM4 != 0) || minDer == 0)
        minDer = abs(ContM4);
      if((abs(ContM6) < minDer && ContM6 != 0) || minDer == 0)
        minDer = abs(ContM6);
        
      int minIzq = 0;
      if((abs(ContM1) < minIzq && ContM1 != 0) || minIzq == 0)
        minIzq = abs(ContM1);
      if((abs(ContM3) < minIzq && ContM3 != 0) || minIzq == 0)
        minIzq = abs(ContM3);
      if((abs(ContM5) < minIzq && ContM5 != 0) || minIzq == 0)
        minIzq = abs(ContM5);
        
      //--------------------Asignacion de sentido para el contador del encoder--------------------
      //----------Operador ternario, ejemplo-> variable = expresion ? true_value : false_value;----------

      //----------Entiendo: si el valor minimo almacenado por los encoders es superado hasta por el doble
      //----------en alguno de ellos, se igualara, este, al valor minimo
      //----------Entiendo: evita que exista mucha diferencia entre lecturas de encoders

      if(abs(ContM1) > tol * minIzq)
        ContM1 = (ContM1 < 0)? -minIzq:minIzq;   
      if(abs(ContM3) > tol * minIzq)
        ContM3 = (ContM3 < 0)? -minIzq:minIzq;
      if(abs(ContM5) > tol * minIzq)
        ContM5 = (ContM5 < 0)? -minIzq:minIzq;
      if(abs(ContM2) > tol * minDer)
        ContM2 = (ContM2 < 0)? -minIzq:minIzq;
      if(abs(ContM4) > tol * minDer)
        ContM4 = (ContM4 < 0)? -minIzq:minIzq;
      if(abs(ContM6) > tol * minDer)
        ContM6 = (ContM6 < 0)? -minIzq:minIzq;
      
      //  Se omiten los motores 3 y 4 debido a que la construccion del robot no permite estos
      //  esten a la misma altura, haciendo que los motores de las esquinas hagan mayor contacto.
      //  Solo tomaremos en cuenta los motores de las esquinas.

      //  encoDerAvg = (ContM2+ContM4+ContM6)/3;
      //  encoIzqAvg = (ContM1+ContM3+ContM5)/3;

      encoDerAvg = (ContM2+ContM6)/2;
      encoIzqAvg = (ContM1+ContM5)/2;

      //Periodo maximo de muestreo para la velocidad. Se actualiza cada segundo
      //Uptade every one second, this will be equal to reading frecuency (Hz). 
      
      noInterrupts(); // Don't process interrupts during calculations during critical or time-sensitive code
                      // Desconectamos la interrupción para que no actué en esta seccion de codigo importante
      
      //--------------------Calculo de RPM en lado derecho--------------------

      RPM_der = (1000.0 * 60.0 * (fabs(encoDerAvg) / float(pulsesPerWheel_Spin))) / float(millis() - timeold);
      velocidad_R = RPM_der * wheel_circunference / 60.0;     //Velocidad en [m/s]
      ContM2 = 0;  // Reinicializamos los pulsos en encoder.
      ContM4 = 0;  //                 ""
      ContM6 = 0;  //                 ""
      
      //--------------------Calculo de RPM en lado izquierdo--------------------
      RPM_izq = (1000.0 * 60.0 * (fabs(encoIzqAvg) / float(pulsesPerWheel_Spin))) / float(millis() - timeold);
      velocidad_L = RPM_izq * wheel_circunference / 60.0;     //Velocidad en [m/s]
      ContM1 = 0;  // Reinicializamos los pulsos en encoder.
      ContM3 = 0;  //                 ""
      ContM5 = 0;  //                 ""
      
      timeold = millis();   // Almacenamos el tiempo actual.
      interrupts();         // Reactivation of interrupt function 
                            // Reactivamos la funcion interrupción    
  }
}

//----------Definición de variables para el PID derecho----------
float P_R = 0;        //Acción proporcional
float I_R = 0;        //Acción integral
float D_R = 0;        //Acción derivativa
//float Kp_R = kp * 0.875;  //Constante proporcional  = 0.4
//float Ki_R = ki;      //Constante integral      = 0.025
//float Kd_R = kd;      //Constante derivativa    = 2.9
float Kp_R = 55; 
float Ki_R = 0;
float Kd_R = 0;
float error_R = 0;
float last_error_R = 0; 

//----------Definición de variables para el PID izquierdo----------
float P_L = 0;        //Acción Proporcional
float I_L = 0;        //Acción Integral
float D_L = 0;        //Acción Derivativa
//float Kp_I = kp;      //Constante proporcional  = 0.4
//float Ki_I = ki;      //Constante integral      = 0.025
//float Kd_I = kd;      //Constante derivativa    = 2.9
float Kp_I = 61;
float Ki_I = 0;
float Kd_I = 0;
float error_L = 0;
float last_error_L = 0; 

//Tiempo del ciclo del PID
float cycleTime = 200;   // [ms]

///Tiempo del ciclo del PID en segundo 
// NOTA:No modificar, se calcula sola.
float cycleTimeSeconds = 0; //Variable auxiliar del tiempo del ciclo del PID, en [s]

//---------------------------------------Control PID---------------------------------------

float error_between_speeds, Rplus;
float lastInputUtFunc_R = 0;
float lastInputUtFunc_L = 0;
float pid_tiempo, pid_tiempo_anterior;
const float krp = 0.001;

void pid(){

  pid_tiempo = float(millis())/1000.0;  // [s]

  //----------------------------PID: velocidades----------------------------

  error_between_speeds = velocidad_R - velocidad_L;   //Diferencia de velocidades entre lado izquierdo y derecho de llantas
  Rplus = krp * error_between_speeds;                     

  //--------------------------------Right PID--------------------------------

  outputFunc_R = velocidad_R;                         //Funcion de salida Real  [m/s]
  referenceFunc_R = velocidadder;                     //Funcion Deseada  [m/s]
  error_R = referenceFunc_R - outputFunc_R - Rplus;   //Cálculo del error total. Se agrega el error ente llantas

  P_R = Kp_R * error_R;                                                               //Acción proporcional
  D_R = Kd_R * (error_R - last_error_R) / (pid_tiempo - pid_tiempo_anterior);         //Acción derivativa
  I_R += Ki_R * 0.5 * (error_R + last_error_R) * (pid_tiempo - pid_tiempo_anterior);  //Acción integral  
  
  inputUtFunc_R = lastInputUtFunc_R + P_R + D_R + I_R;    //Suma de las tres acciones para obtener la señal de control
  
  if(inputUtFunc_R > 255) inputUtFunc_R = 255;    
  else if(inputUtFunc_R < 0) inputUtFunc_R = 0;
  
  if(velocidadder == 0){
    inputUtFunc_R = 0;
    last_error_R = 0;
    error_R = 0;
  }
  
  last_error_R = error_R;
  lastInputUtFunc_R = inputUtFunc_R; 

  //--------------------------------Left PID--------------------------------

  outputFunc_L = velocidad_L;
  referenceFunc_L = velocidadizq;
  error_L = referenceFunc_L - outputFunc_L + Rplus;           //Cálculo del error
 
  P_L = Kp_I * error_L;                                             //Acción proporcional
  D_L = Kd_I * (error_L - last_error_L) / cycleTimeSeconds;         //Acción derivativa
  I_L += Ki_I * 0.5 * (error_L + last_error_L) * cycleTimeSeconds;  //Acción integral
 
  inputUtFunc_L = lastInputUtFunc_L + P_L + D_L + I_L;    //Suma de las tres acciones para obtener la señal de control
  
  if(inputUtFunc_L > 255) inputUtFunc_L = 255;
  else if(inputUtFunc_L < 0) inputUtFunc_L = 0;
  
  if(velocidadizq == 0){
    inputUtFunc_L = 0;
    last_error_L = 0;
    error_L = 0;
  }
  
  last_error_L = error_L;
  lastInputUtFunc_L = inputUtFunc_L; 
  pid_tiempo_anterior = pid_tiempo;
}

//Función que dependiendo del valor de "referenceFuncValue_R" cambia los enables de los motores
void elecciongiro(){
  if (referenceFuncValue_R < 0){
    MD_B = 1;
    MD_A = 0;
  }
  else{
    MD_A = 1;
    MD_B = 0;
  }
  if (referenceFuncValue_L < 0){
    MI_A = 1;
    MI_B = 0;
  }
  else{
    MI_B = 1;
    MI_A = 0;
  }
}

void rpmToPwm(){
  //Convertimos el valor de RPMs a PWM (Pulse Width Modulation)
  pwm_I = inputUtFunc_L;//map(inputUtFunc_L, minPID, maxPID, minPWM, maxPWM);
  pwm_D = inputUtFunc_R;//map(inputUtFunc_R, minPID, maxPID, minPWM, maxPWM);
}

//--------------------------------ROS---------------------------------

ros::NodeHandle nh;
std_msgs::Int64 encoderIzq;
std_msgs::Int64 encoderDer;
std_msgs::Int16MultiArray photoSensors;
ros::Publisher pubEncoIzq("/encoIzq", &encoderIzq);
ros::Publisher pubEncoDer("/encoDer", &encoderDer);
ros::Publisher pubSensors("/photoSensors",&photoSensors);
ros::Subscriber<std_msgs::Float32> subSpeedL("/motorL_speed", &speedLCallback);
ros::Subscriber<std_msgs::Float32> subSpeedR("/motorR_speed", &speedRCallback);
ros::Subscriber<std_msgs::Int8> subKpPid("/kpPID", &kpCallback);
//ros::Subscriber<std_msgs::Int8> subKdPid("/kdPID", &kdCallback);
//ros::Subscriber<std_msgs::Int8> subKiPid("/kiPID", &kiCallback);

void setup() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);  //Baudaje de comunicación
  nh.initNode();                      //Se inicializa el nodo  
  nh.advertise(pubSensors);
  nh.advertise(pubEncoIzq);
  nh.advertise(pubEncoDer); 
  nh.subscribe(subKpPid);  
  //nh.subscribe(subKdPid);  
  //nh.subscribe(subKiPid); 
  nh.subscribe(subSpeedL);
  nh.subscribe(subSpeedR);
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
    
  //Motores
  pinMode(PWM_MotorD,OUTPUT);
  pinMode(PWM_MotorI,OUTPUT);
  pinMode(MotorD_A,OUTPUT);
  pinMode(MotorD_B,OUTPUT);
  pinMode(MotorI_A,OUTPUT);
  pinMode(MotorI_A,OUTPUT);
  
  //Interrupciones
  attachInterrupt(0,encoderM1aEvent,CHANGE); //Int0 =Enc_M1A = pin2
  attachInterrupt(1,encoderM2aEvent,CHANGE); //Int1 =Enc_M2A = pin3
  attachInterrupt(5,encoderM3aEvent,CHANGE); //Int5 =Enc_M3A = pin18
  attachInterrupt(4,encoderM4aEvent,CHANGE); //Int4 =Enc_M4A = pin19
  attachInterrupt(3,encoderM5aEvent,CHANGE); //Int3 =Enc_M5A = pin20
  attachInterrupt(2,encoderM6aEvent,CHANGE); //Int2 =Enc_M6A = pin21
  
  //Sensores
   pinMode(Cont_Der,INPUT);
   pinMode(Cont_Izq,INPUT);

  //Valores iniciales
  referenceFuncValue_L = 0;
  referenceFuncValue_R = 0; 
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
  cycleTimeSeconds = cycleTime / 1000;
}

void loop(){
  Serial.println(ContM3);
  
  //Lecturas de fotoresistencias
  for(int i=0;i<8;i++)
     data[i]=analogRead(fotoR[i]);
     
   //Lecturas de sensores infrarrojo
   //data[8]=digitalRead(Cont_Der);
   //data[9]=digitalRead(Cont_Izq);
   
  elecciongiro();
  calculoVelocidad(); 
  //nh.spinOnce();
  pid();
  rpmToPwm();
  
  //current speeds readings
  //data[10]=velocidad_L;
  //data[11]=velocidad_R;
  
  //Direcciones y velocidad de lo motores
  digitalWrite(MotorD_A, MD_A);  //Motores Derechos
  digitalWrite(MotorD_B, MD_B);
  digitalWrite(MotorI_A, MI_A);  //Motores Izquierda
  digitalWrite(MotorI_B, MI_B);

  analogWrite(PWM_MotorD, pwm_D);
  analogWrite(PWM_MotorI, pwm_I);

  totalDer += encoDerAvg;
  totalIzq += encoIzqAvg;
  photoSensors.data = data;
  encoderDer.data = totalDer;
  encoderIzq.data = totalIzq; 
  pubSensors.publish(&photoSensors);
  pubEncoIzq.publish(&encoderIzq);
  pubEncoDer.publish(&encoderDer);
 
  nh.spinOnce();
  delay(30);
}
