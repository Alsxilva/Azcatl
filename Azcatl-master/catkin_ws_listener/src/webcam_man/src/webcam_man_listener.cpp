#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>



#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <vector>


#include "opencv2/highgui.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>



#include <iostream>
using namespace cv;
using namespace std;


int i = 0;
using namespace std;
cv::Mat frame;
bool imgGet = false;
float percent = 0;
void callbackImage(const sensor_msgs::Image::ConstPtr &msg){
    //printf("%d\n",i++);
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    frame = cv_ptr->image;
    imgGet = true;
}
cv::Mat imagenHSV,imagenRango;
cv::Mat imagen;
Scalar minimoC1 = Scalar(20,0,0);
Scalar maximoC1 = Scalar(60,250,200);
Scalar minimoC2 = Scalar(115,100,209);
Scalar maximoC2 = Scalar(120,255,255);
Scalar minimo,maximo;

void callbackColorId(const std_msgs::Int8::ConstPtr &msg){
    minimo = minimoC1;
    maximo = maximoC1;
    if((msg->data) == 2 ){
    	minimo = minimoC2;
	maximo = maximoC2;
    }

}


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
	percent = conteo/((*imagen).cols * (*imagen).rows);
	if(conteo > 0.05){
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


int main(int argc, char **argv){
    cout<<"Inicializando webcam_man_pub node..."<<endl;
    ros::init(argc, argv, "webcam_man_listener");
    ros::NodeHandle nh;
    minimo = minimoC1;
    maximo = maximoC1;
    //Subscriptor
    ros::Subscriber subImage = nh.subscribe("/webcam_image",0,callbackImage);
    ros::Subscriber subColorId = nh.subscribe("/colorId",1,callbackColorId);
    //Publicadores
    ros::Publisher pubColorPos = nh.advertise<std_msgs::Int8>("/colorPos",1);
    ros::Publisher pubPercentColor = nh.advertise<std_msgs::Float32>("/percentColor",1);
    ros::Rate rate(50);
    while(ros::ok()){
	if(imgGet == true){
		/*------------------codigon--------------------*/
		imagen = frame;
		cv::Mat rangoModificado;
		int tam = 1;
		cv::Mat element = cv::getStructuringElement(0,cv::Size( 2*tam + 1, 2*tam+1 ),cv::Point( tam, tam ));
		cv::dilate( imagen, rangoModificado, element );
		cvtColor(rangoModificado, imagenHSV,  cv::COLOR_RGB2HSV);
		inRange(imagenHSV, minimo,maximo, imagenRango);
		//cv::imshow("Rango", imagenRango);
		//centroide
		cv::Mat imagenAND;
		cv::bitwise_and(imagen,imagen,imagenAND,imagenRango);
		int Centroide[2];
		centroide(&imagenAND,Centroide);	
		circle(frame, cv::Point(Centroide[1],Centroide[0]),10,Scalar(0,0,255),-1);
		int posColor = -1; //-1-> No se encuentra ; 0->Izq ; 1->Centro ; 2->Derecha		
		if(Centroide[1]>0 && Centroide[1]<(frame.cols*0.25))
			posColor = 0;
		else if(Centroide[1]>=(frame.cols*0.25) && Centroide[1]<=(frame.cols*0.75))
			posColor = 1;
		else
			posColor = 2;
		//printf("%d \n",posColor);
		//cv::imshow("Rect image", frame);
		/*---------------------------------*/
    		//cv::imshow("Frame", frame);
		//cv::waitKey(1);
		colorPos.data = posColor;
		percentColor.data = percent;
		pubColorPos.publish(colorPos);
		pubPercentColor.publish(percentColor); 
	}
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
