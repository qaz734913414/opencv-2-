/*=====================================================================================
*                      chi_lun_jian_ce.c --
*	                   Copyleft! 2017 William Yu
*          Some rights reserved：CC(creativecommons.org)BY-NC-SA
*                      Copyleft! 2017 William Yu
*      版权部分所有，遵循CC(creativecommons.org)BY-NC-SA协议授权方式使用
*
* Filename                : chi_lun_jian_ce
* Programmer(s)           : William Yu
* Description             : (OpenCV2.4.9+VS2013)
                            此代码源于博文 http ://blog.csdn.net/a_jia_17/article/details/60340158
                            该博文采取的方法为：取轮廓，求质心，算齿轮分度圆，并切除分度圆，取剩余轮廓数即为齿数    
                            该博文基础上添加了我学习的备注，引入了【大津法】并添加了记时用以优化                     
                            并在该博文基础上改进出另一种齿轮齿数的方法：齿轮检测（腐蚀 + 凸包）                      

* Reference               : http ://blog.csdn.net/a_jia_17/article/details/60340158
* Modification History	  : ver1.0, 2017.02.06, William Yu
*                           ver1.1, 2017.02.21, William Yu add notes
=====================================================================================*/
#include <opencv2/opencv.hpp>      
#include <opencv2/imgproc/imgproc.hpp>   
#include <opencv2/highgui/highgui.hpp>   
#include <math.h>  
#include <iostream>  

using namespace cv;
using namespace std;

//-----------------------------------添加---------------------------------------    
//大津法函数声明
int otsuThreshold(IplImage *frame);
//-----------------------------------添加---------------------------------------    

int main()
{
	//-----------------------------------添加---------------------------------------  
	//记时优化
	int64 st1, et1;
	st1 = cvGetTickCount();
	//-----------------------------------添加---------------------------------------    



		//-----------------------------------【图像的预处理】---------------------------------------    
		//      描述：图像的读入，灰度化，二值化及滤波     
		//-----------------------------------------------------------------------------------------------      
		//【1】载入原始图和Mat变量定义       
		Mat srcImage = imread("齿轮.jpg", 1);  //工程目录下应该有一张名为齿轮.jpg的素材图    
		Mat grayImage, thresholdImage;//临时变量和目标图的定义    

		//【2】显示原始图    
		namedWindow("【1.原始图】", WINDOW_NORMAL);
		imshow("【1.原始图】", srcImage);

		//【3】提取感兴趣域  
		//-----------------------------------添加---------------------------------------  
		//使用cvsetImageROI函数
		//cvsetImageROI(src，cvRect(x, y, width, height))
		//其中x和y为ROI区域的起点，width和height为宽和高，
		//对src提取感兴趣区域后，再对src进行图像处理时只针对提取的感兴趣进行处理，在图像处理时要注意这一点。
		//使用该函数时要对通过cvResetImageROI()函数释放ROI，否则对图像进行处理时只对ROI区域进行对比。
		//-----------------------------------添加---------------------------------------    
		Rect ROIim(300, 100, 600, 600);    //参数需视图片修改
		Mat ROIsrc;
		srcImage(ROIim).copyTo(ROIsrc);
		namedWindow("【2.roiimage】", WINDOW_NORMAL);
		imshow("【2.roiimage】", ROIsrc);

		//【4】转为灰度图，进行图像平滑    
		cvtColor(ROIsrc, grayImage, CV_BGR2GRAY);//转化边缘检测后的图为灰度图     
		namedWindow("【3.灰度图】", WINDOW_NORMAL);
		imshow("【3.灰度图】", grayImage);

		//【5】图像的二值化，采用大津法 
		//-----------------------------------改动---------------------------------------    
		IplImage * frame_gray;
		frame_gray = &IplImage(grayImage);
		int threshold1 = otsuThreshold(frame_gray);           //调用大津法函数
		threshold(grayImage, thresholdImage, threshold1, 255, CV_THRESH_BINARY_INV);   
		namedWindow("【4.二值图】", WINDOW_NORMAL);
		imshow("【4.二值图】", thresholdImage);
		//-----------------------------------改动---------------------------------------   

		//【6】中值滤波  
		medianBlur(thresholdImage, thresholdImage, 3);
		namedWindow("【5.滤波处理结果图】", WINDOW_NORMAL);
		imshow("【5.滤波处理结果图】", thresholdImage);



		//-----------------------------------【图像分割】---------------------------------------    
		//      描述：边缘检测，轮廓提取
		//-----------------------------------------------------------------------------------------------  
		
		//【1】获取轮廓  
		vector<vector<Point>> contours;
		findContours(thresholdImage,            //图像    
			contours,               //轮廓点     
			CV_RETR_EXTERNAL,       //获取轮廓的方法（这里获取外围轮廓）    
			CV_CHAIN_APPROX_NONE    //轮廓近似的方法（这里不近似，获取全部轮廓）  
			);
		//测试轮廓条数  
		cout << "轮廓：" << contours.size() << "条" << "\n";
		for (int i = 0; i<contours.size(); i++)
		{
			cout << "轮廓长：" << contours[i].size() << endl;
		}
		long cmin = 2000;           //参数需视图片修改
		long cmax = 3000;
		vector<vector<Point>>::const_iterator itc = contours.begin();
		while (itc != contours.end())
		{
			if (itc->size() < cmin || itc->size() > cmax)
				itc = contours.erase(itc);
			else
				++itc;
		}
		cout << "有效轮廓：" << contours.size() << "条" << "\n" << endl;

		//【2】画出轮廓 
		Mat ContoursIm(thresholdImage.size(), CV_8U, Scalar(255));//制与原图同大小的画板  
		drawContours(ContoursIm, //画板  
			contours, //轮廓本身（指针）  
			-1,       //轮廓等级（这里画出所有轮廓）  
			Scalar(0),//颜色  
			-1);       //线粗，>=0时轮廓线粗；-1时既是CV_FILLED，填满  
		namedWindow("【6.ContoursIm】", WINDOW_NORMAL);
		imshow("【6.ContoursIm】", ContoursIm);



		//-----------------------------------【测试参数】---------------------------------------    
		//      描述：齿顶圆、齿根圆直径，齿数    
		//----------------------------------------------------------------------------------------------- 

		//【1】获取圆心  
		//计算轮廓矩  
		vector<Moments> mu(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			mu[i] = moments(contours[i], true);
		}
		//计算轮廓的质心  
		vector<Point2f> mc(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
			cout << "质心坐标:" << mc[i] << endl;
		//	cout << "pointPloyTest齿根圆直径:" << pointPolygonTest(contours[i], mc[i], true) * 2 << "\n" << endl;
		}

		//【2】齿根圆、齿顶圆直径  
		double rmax, rmin, r, r2;
		for (int i = 0; i<contours.size(); i++)
		{
			for (int j = 0; j<contours[i].size(); j++)
			{
				r2 = pow(contours[i][j].x - mc[i].x, 2) + pow(contours[i][j].y - mc[i].y, 2);
				r = sqrt(r2);
				if (j == 0)
				{
					rmax = rmin = r;
				}
				if (rmax <= r)
				{
					rmax = r;
				}
				if (rmin >= r)
				{
					rmin = r;
				}
			}
			cout << "齿跟圆直径：" << rmin * 2 << endl;
			cout << "齿顶圆直径：" << rmax * 2 << "\n" << endl;
		}

		//【3】齿数测量  
		//截取轮齿  
		Mat thresholdIm = ContoursIm.clone();
		circle(thresholdIm, mc[0], (rmax + rmin) / 2, Scalar(255), -1);
		namedWindow("【7.画圆】", WINDOW_NORMAL);
		imshow("【7.画圆】", thresholdIm);
		//滤波  
		threshold(thresholdIm, thresholdIm, 0, 255, CV_THRESH_BINARY_INV);
		medianBlur(thresholdIm, thresholdIm, 3);
		namedWindow("【8.二值图2】", WINDOW_NORMAL);
		imshow("【8.二值图2】", thresholdIm);
		//获取轮齿轮廓  
		vector<vector<Point>> contour;
		findContours(thresholdIm, contour,
			CV_RETR_LIST,       //获取轮廓的方法（这里检测所有轮廓，不分级）    
			CV_CHAIN_APPROX_NONE    //轮廓近似的方法（这里不近似，获取全部轮廓）  
			);
		//去除过长或者过短的轮廓  
		int csmin = 10;
		int csmax = 200;
		vector<vector<Point>>::const_iterator itcs = contour.begin();
		while (itcs != contour.end())
		{
			if (itcs->size() < csmin || itcs->size() > csmax)
				itcs = contour.erase(itcs);
			else
				++itcs;
		}
		//绘制轮廓
		Mat ContourIm(thresholdIm.size(), CV_8U, Scalar(255));//制与原图同大小的画板  
		drawContours(ContourIm, contour,
			-1,       //轮廓等级（这里画出所有轮廓）  
			Scalar(0),//颜色  
			2);       //线粗，>=0时轮廓线粗；-1时既是CV_FILLED，填满  
		namedWindow("【9.ContourIm】", WINDOW_NORMAL);
		imshow("【9.ContourIm】", ContourIm);

		//输出齿数  
		cout << "齿数：" << contour.size()<<"\n" << endl;

	//-----------------------------------添加---------------------------------------    
	//时间显示
	et1 = cvGetTickCount();
	cout << "times cost:" << (et1 - st1) / (double)cvGetTickFrequency() / 1000.0 << "milliseconds\n\n";
	//-----------------------------------添加---------------------------------------    
	waitKey(0);
	return 0;
}


//-----------------------------------添加---------------------------------------    
/************************     大津法      **************************/

int otsuThreshold(IplImage *frame)
{
#define GrayScale 256
	int width = frame->width;
	int height = frame->height;
	int pixelCount[GrayScale];
	float pixelPro[GrayScale];
	int i, j, pixelSum = width * height, threshold = 0;
	uchar* data = (uchar*)frame->imageData;  //指向像素数据的指针
	for (i = 0; i < GrayScale; i++)
	{
		pixelCount[i] = 0;
		pixelPro[i] = 0;
	}

	//统计灰度级中每个像素在整幅图像中的个数  
	for (i = 0; i < height; i++)
	{
		for (j = 0; j < width; j++)
		{
			pixelCount[(int)data[i * width + j]]++;  //将像素值作为计数数组的下标
		}
	}

	//计算每个像素在整幅图像中的比例  
	float maxPro = 0.0;
	int kk = 0;
	for (i = 0; i < GrayScale; i++)
	{
		pixelPro[i] = (float)pixelCount[i] / pixelSum;
		if (pixelPro[i] > maxPro)
		{
			maxPro = pixelPro[i];
			kk = i;
		}
	}

	//遍历灰度级[0,255]  
	float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
	for (i = 0; i < GrayScale; i++)     // i作为阈值
	{
		w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
		for (j = 0; j < GrayScale; j++)
		{
			if (j <= i)   //背景部分  
			{
				w0 += pixelPro[j];
				u0tmp += j * pixelPro[j];
			}
			else   //前景部分  
			{
				w1 += pixelPro[j];
				u1tmp += j * pixelPro[j];
			}
		}

		u0 = u0tmp / w0;  //背景均值
		u1 = u1tmp / w1; //前景均值


		u = u0tmp + u1tmp;//总均值的定义
		deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);//类间方差
		if (deltaTmp > deltaMax)
		{
			deltaMax = deltaTmp;
			threshold = i;
		}
	}

	return threshold;
}
//-----------------------------------添加---------------------------------------    
