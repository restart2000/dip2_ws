#include<stdlib.h>
#include<iostream>
#include<string>
#include<algorithm>
#include<cv.h>
#include<highgui.h>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include"ros/ros.h"
#include"std_msgs/String.h"
#include"std_msgs/Bool.h"
#include"std_msgs/Float32.h"
#include<geometry_msgs/Twist.h>
#include"sensor_msgs/Image.h"
#define LINEAR_X 0
#define pi 3.141592653589793

using namespace cv;
using namespace std;

double **GetGaussianKernal(int size,double sigma)
{
	int i,j;
	double **kernal=new double *[size];
	for(i=0;i<size;i++)
	{
		kernal[i]=new double[size];
	}
	
	int center_i,center_j;
	center_i=center_j=size/2; //定义原点的位置
	double sum;
	sum=0;
	
	for(i=0;i<size;i++)
	{
		for(j=0;j<size;j++)
		{
			kernal[i][j]=exp(  (-1)*(((i-center_i)*(i-center_i)+(j-center_j)*(j-center_j)) / (2*sigma*sigma)));
			sum+=kernal[i][j];
		}
	}
	
	for(i=0;i<size;i++)
	{
		for(j=0;j<size;j++)
		{
			kernal[i][j]/=sum;  //归一化求权值
		}
	}
	return kernal;
}


void GaussianFilter(Mat *src,double **kernal,int size)
{
	Mat temp=(*src).clone();
	for(int i=0;i<(*src).rows;i++)
	{
		for(int j=0;j<(*src).cols;j++)
		{
			if(i>(size/2)-1 && j>(size/2)-1 && i<(*src).rows-(size/2) && j<(*src).cols-(size/2))
			//i > (size / 2) - 1 && j > (size / 2) - 1 忽略边缘
			//i < (*src).rows - (size / 2) && j < (*src).cols - (size / 2) 忽略下边缘
			{
				double sum=0;
				for(int k=0;k<size;k++)
				{
					for(int l=0;l<size;l++)
					{
						sum=sum + (*src).ptr<uchar>(i-k+(size/2))[j-l+(size/2)] * kernal[k][l];
					}
				}
				temp.ptr<uchar>(i)[j]=sum;
						//image.ptr<type>(i)[j],OpenCV中访问像素的方法，指针遍历
						//也可以image.at<type>(i, j)[channel]
			}
		}
	}
	*src=temp.clone();
}


Mat Gaussian(Mat input,Mat output,int size,double sigma)
{
	std::vector<cv::Mat> channels;
	cv::split(input,channels);
	
	double **kernal=GetGaussianKernal(size,sigma);
	for(int i=0;i<3;i++)
	{
		GaussianFilter(&channels[i],kernal,size);
	}

	cv::merge(channels,output);
	return output;
}


Mat Dilate(Mat Src, Mat Tem , Mat Dst)
{

	
	int m = (Tem.rows - 1) / 2;
	int n = (Tem.cols - 1) / 2;
	for (int i = m; i < Src.rows - m; i++)
	{
		for (int j = n; j < Src.cols - n; j++)
		{
			//忽略边缘，将起始点定为结构元的终点坐标
            Rect rect(j - m, i - n, Tem.rows, Tem.cols);
			Mat ROI = Src(rect);
			double sum = 0;
			
			for (int k=0;k<Tem.rows;k++)
			{
				for(int l=0;l<Tem.cols;l++)
				{
					sum=sum+ (ROI.at<uchar>(k,l) * Tem.at<uchar>(k,l));
				}
				
			}
			if (sum >= 255)   //fit
				Dst.at<uchar>(i, j) = 255;
			else
				Dst.at<uchar>(i, j) = 0;
		}
	}
	return Dst;
}

Mat Erode(Mat Src,Mat Tem,Mat Dst)
{
	int m = (Tem.rows - 1) / 2;
	int n = (Tem.cols - 1) / 2;
	for (int i = m; i < Src.rows - m; i++)
	{
		for (int j = n; j < Src.cols - n; j++)
		{
            Rect rect(j - m, i - n, Tem.rows, Tem.cols);
			Mat ROI = Src(rect);
			double sum = 0;

			for (int k=0;k<Tem.rows;k++)
			{
				for(int l=0;l<Tem.cols;l++)
				{
					sum=sum+ (ROI.at<uchar>(k,l) * Tem.at<uchar>(k,l));
				}
				
			}
			if (sum == Tem.cols*Tem.rows*255)  //hit
				Dst.at<uchar>(i, j) = 255;
			else
				Dst.at<uchar>(i, j) = 0;
		}
	}
	return Dst;

}

int main(int argc,char **argv)
{
	ROS_WARN("*****START");
	ros::init(argc,argv,"trafficLaneTrack");
	ros::NodeHandle n;
	//ros::Rate loop_rate(10);
	//ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel",5);
	
	VideoCapture capture;
	capture.open(0);
	waitKey(100);
	if(!capture.isOpened())
	{
		printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
		return 0;
	}
	Mat frame;
	int nFrames=0;
	int frameWidth=capture.get(CV_CAP_PROP_FRAME_WIDTH);
	int frameHeight=capture.get(CV_CAP_PROP_FRAME_HEIGHT);

	while(ros::ok())
	{
		/*capture.read(frame); //如果使用imread读取图片，请将该段注释
		if(frame.empty())
		{	
			break;
		}
		imshow("frame",frame);*/
		

		frame=imread("/home/zhang/lena.jpg");  //如果使用该段代码，请注意将相机capture.read关闭
		if (frame.empty())
		{
			printf("Can not load image\n");
			return 0;
		}
		imshow("frame",frame);

		Mat frIn=frame.clone();
		Mat gray;
		Mat frame_guassian;
		Mat frame_Dilate;
		Mat binary;
		Mat frame_Erode;
		int size=5; //gaussian size
		int element_size=5;
		Mat element(element_size,element_size,CV_8U,Scalar(1));
		double sigma=5;


		frame_guassian=Gaussian(frIn,frame_guassian,size,sigma);
		//imshow("frame_guassian",frame_guassian);


		cvtColor(frame, gray, COLOR_BGR2GRAY);
    		imshow("Gray", gray);
		
		
		

		
		waitKey(5);
	}
	return 0;
}
