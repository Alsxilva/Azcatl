#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>


using namespace cv;

int width = 320;
int height = 240;
//H:130 S:136 V:223  rosa
//H:110 S:193 V:232  naranja
//H:155 S:103 Vi:124  morado
//H:87-94  S:122-150 V:133-170   amarillo
Scalar minimoC1 = Scalar(120,0,150);//120,0,0 130,255,255
Scalar maximoC1 = Scalar(130,150,200);
Scalar minimoC2 = minimoC1;
Scalar maximoC2 = maximoC1;
Scalar minimo,maximo;


float percent;

void centroide(cv::Mat* imagen,int* Centroide){
         unsigned long long int sumax = 0;
         unsigned long long int sumay = 0;
         unsigned long long int conteo = 0;
         for(int x=0 ; x<(*imagen).rows ;x++){
                 for(int y=0 ; y<(*imagen).cols ; y++){
                         if((*imagen).at<cv::Vec3b>(x,y)[0] != 0 ){
                                 //printf("%d\n",(*imagen).at<cv::Vec3b>(10,10)[0]);
                                 //getchar();
                                 conteo++;
                                 sumax += x;
                                 sumay += y;
                         }
                 }
         }
         percent =((float)(conteo))/((float)((*imagen).cols * (*imagen).rows));
         //percent = conteo;
         if(percent > 0.0025){
                 sumax /= conteo;
                 sumay /= conteo;
                 Centroide[0] = sumax;
                 Centroide[1] = sumay;
         }else{
                 Centroide[0] = -100;
                 Centroide[1] = -100;
         }
         //printf("%llu x:%llu y:%llu \n",conteo,Centroide[0],Centroide[1]);
}
std_msgs::Float32 percentColor;
std_msgs::Int8 colorPos;

int main(int argc,char **argv){
	printf("Start web cam Node \n");
	ros::init(argc,argv,"webcam_man_pub");
	ros::NodeHandle nh;
	ros::Rate rate(50);
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pubImage=it.advertise("/webcam_image",1);
	ros::Publisher pubPosColor = nh.advertise<std_msgs::Int8>("/colorPos",1);
	ros::Publisher pubPercentColor = nh.advertise<std_msgs::Float32>("/percentColor", 1);
	VideoCapture capture;
	Mat frame;
        capture.open(0);
	int i = 0;
	minimo = minimoC1;
	maximo = maximoC1;
	while(ros::ok()){
		//printf("%d\n",i++);
		capture.read(frame);
		float scale = 0.25;
		cv::resize(frame,frame,cv::Size(frame.cols*scale,frame.rows*scale),0,0);
		Mat original = frame;
		int tam = 3;
                Mat element = cv::getStructuringElement(0,cv::Size( 2*tam + 1, 2*tam+1 ),cv::Point( tam, tam ));
	        dilate(frame, frame, element );	
		cvtColor(frame,frame,cv::COLOR_RGB2HSV);
		Mat mask;
		inRange(frame, minimo,maximo,mask);


		Mat imagenAND;
		bitwise_and(frame,frame,imagenAND,mask);
		int Centroide[2];
		centroide(&imagenAND,Centroide);        
		circle(frame, cv::Point(Centroide[1],Centroide[0]),10,Scalar(0,0,255),-1);
		int posColor = -1; //-1-> No se encuentra ; 0->Izq ; 1->Centro ; 2->Derecha             
		if(percent<0.0025)
			posColor = -1;	
		else if(Centroide[1]>0 && Centroide[1]<(frame.cols*0.25))
			posColor = 0;
		else if(Centroide[1]>=(frame.cols*0.25) && Centroide[1]<=(frame.cols*0.75))
		        posColor = 1;
		else
		        posColor = 2;
	
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",original).toImageMsg();
		printf("porcentaje: %f Zona: %d\n",percent,posColor);
		colorPos.data = posColor;
		percentColor.data = percent;
		pubPosColor.publish(colorPos);
		pubPercentColor.publish(percentColor);
		pubImage.publish(msg);
		ros::spinOnce();
		rate.sleep();

	}	
	return 0;
}
