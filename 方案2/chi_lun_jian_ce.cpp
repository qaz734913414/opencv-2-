/******************************************************************************************************/
/********************************         齿轮检测（腐蚀+凸包）         *******************************/
/***********************************   2017.07.08   by:William Yu  ************************************/
/******************************************************************************************************/
/**** 注 ：此代码部分参考并移植于博文 http ://blog.csdn.net/a_jia_17/article/details/60340158       ***/
/****      该博文采取的方法为：取轮廓，求质心，算齿轮分度圆，并切除分度圆，取剩余轮廓数即为齿数     ***/
/****      此代码采取的方法为：取轮廓，腐蚀齿轮为尖状，求凸包，凸包点数即为齿数          **************/
/******************************************************************************************************/

#include <opencv2/OpenCV.hpp>      
#include <opencv2/imgproc/imgproc.hpp>   
#include <opencv2/highgui/highgui.hpp>   
#include <math.h>  
#include <iostream>  
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
using namespace std;

//大津法函数声明
int otsuThreshold(IplImage *frame);

int main()
{
	//记时优化
	int64 st1, et1;
	st1 = cvGetTickCount();


		//-----------------------------------【图像预处理】---------------------------------------    
		//      描述：图像的读入，选区，灰度化，二值化及滤波     
		//-----------------------------------------------------------------------------------------------  

		//【1】载入原始图和显示图片     
		Mat srcImage = imread("齿轮.jpg", 1);      
		namedWindow("【1.原始图】", WINDOW_AUTOSIZE);
		imshow("【1.原始图】", srcImage);

		//【2】提取感兴趣域  
		Rect ROIim(300, 100, 600, 600);    //参数需视图片修改
		Mat ROIsrc;
		srcImage(ROIim).copyTo(ROIsrc);
		namedWindow("【2.roiimage】", WINDOW_NORMAL);
		imshow("【2.roiimage】", ROIsrc);

		//【3】转为灰度图，进行图像平滑  
		Mat grayImage;
		cvtColor(ROIsrc, grayImage, CV_BGR2GRAY);
		namedWindow("【3.灰度图】", WINDOW_NORMAL);
		imshow("【3.灰度图】", grayImage);

		////【4】图像的二值化
		//Mat thresholdImage;
		//threshold(grayImage, thresholdImage, 126, 255, CV_THRESH_BINARY_INV);   // 第三个参数需视图片修改，可引入大津法？ 
		////注：搜索轮廓认为白色为前景物体，黑色为背景画布, 所以为了后面获取轮廓，需要调整第五个参数，能使齿轮为前景
		//namedWindow("【4.二值图】", WINDOW_NORMAL);
		//imshow("【4.二值图】", thresholdImage);
		
		//【4】图像的大津法二值化
		Mat thresholdImage;
		IplImage * frame_gray;
		frame_gray = &IplImage(grayImage);
		int threshold1 = otsuThreshold(frame_gray);           //调用大津法函数,引入大津法代码速度会慢一些，啊哈哈
		threshold(grayImage, thresholdImage, threshold1, 255, CV_THRESH_BINARY_INV);
		namedWindow("【4.二值图】", WINDOW_NORMAL);
		imshow("【4.二值图】", thresholdImage);

		//【5】中值滤波  
		medianBlur(thresholdImage, thresholdImage, 3);
		namedWindow("【5.滤波】", WINDOW_NORMAL);
		imshow("【5.滤波】", thresholdImage);



		//-----------------------------------【图像分割】---------------------------------------    
		//      描述：边缘检测，轮廓提取    
		//-----------------------------------------------------------------------------------------------  
		
		//【1】获取轮廓  
		vector<vector<Point>> contours;       //数组，点--->一条轮廓--->轮廓集
		//获取轮廓
		findContours(thresholdImage,           
			contours,                  
			CV_RETR_EXTERNAL,       //获取外围轮廓
			CV_CHAIN_APPROX_NONE    //获取全部轮廓
			);
		//显示轮廓条数  
		cout << "轮廓：" << contours.size() << "条" << "\n";
		for (int i = 0; i<contours.size(); i++)
		{
			cout << "轮廓长：" << contours[i].size() << endl;
		}

		//【2】轮廓筛选 
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

		//【3】画出轮廓 
		Mat ContoursIm(thresholdImage.size(), CV_8U, Scalar(0));//制与原图同大小的画板//Scalar(0)画布填充黑色  
		drawContours(ContoursIm, //画板  
			contours, //轮廓本身（指针）  
			-1,       //轮廓等级（这里画出所有轮廓）  
			Scalar(255),//轮廓内填充白色  
			-1);       //线粗，>=0时轮廓线粗；-1时既是CV_FILLED，填满  
		namedWindow("【6.ContoursIm】", WINDOW_NORMAL);
		imshow("【6.ContoursIm】", ContoursIm);



		//-----------------------------------【形态处理】---------------------------------------    
		//      描述：膨胀与腐蚀    
		//-----------------------------------------------------------------------------------------------  
		
		//【1】载入数据
		Mat src;
		src = ContoursIm;
		if (!src.data)
		{
			return -1;
		}

		//【2】腐蚀操作
		Mat dst;  //定义腐蚀结果接收者
		//定义结构元，参数需视图片修改 
		Mat element = getStructuringElement(MORPH_ELLIPSE,    //MORPH_ELLIPSE是椭圆形内核，
			Size(2 * 16 + 1, 2 * 16 + 1),                  	//指定内核大小，16是内核半径
			Point(16, 16));                                 //锚点位置。不指定锚点位置，则默认锚点在内核中心位置。
		//腐蚀
		erode(src,dst, element);
		namedWindow("【7.Erosion】", WINDOW_NORMAL);
		imshow("【7.Erosion】", dst);

		////【2】膨胀操作
		//Mat dst;                
		////定义结构元，参数需视图片修改 
		//Mat element = getStructuringElement(MORPH_ELLIPSE,   
		//	Size(2 * 16 + 1, 2 * 16 + 1),                
		//	Point(16, 16));                               
		////膨胀
		//dilate(src, dst, element);
		//namedWindow("【7.Erosion】", CV_WINDOW_AUTOSIZE);
		//imshow("【7.Dilation】", dst);


		//-----------------------------------【再次图像分割】---------------------------------------    
		//      描述：边缘检测，轮廓提取，计算凸包    
		//----------------------------------------------------------------------------------------------- 

		//【1】获取轮廓
		vector<vector<Point>> contours1;
		findContours(dst,            //图像    
			contours1,               //轮廓点     
			CV_RETR_EXTERNAL,       //获取轮廓的方法（这里获取外围轮廓）    
			CV_CHAIN_APPROX_NONE    //轮廓近似的方法（这里不近似，获取全部轮廓）  
			);

		//【2】取出第0条轮廓
		vector<Point> points_01;   //即降低维度。
		points_01 = contours1[0];  //findContours传出数组是vector<vector<Point>>类型，需要vector<Point>来计算凸包

		//【3】计算凸包
		vector<int> hull;
		convexHull(points_01, hull, 1);

		//【4】绘制轮廓和凸包
		Mat ContoursIm_1(dst.size(), CV_8U, Scalar(0));//制与原图同大小的画板  
		//绘轮廓
		drawContours(ContoursIm_1, //画板  
			contours1, //轮廓本身（指针）  
			-1,       //轮廓等级（这里画出所有轮廓）  
			Scalar(255),//轮廓内填充白色  
			-1);       //线粗，>=0时轮廓线粗；-1时既是CV_FILLED，填满
		//绘凸包
		int hullcount = (int)hull.size();
		Point poi_01 = points_01[hull[hullcount - 1]];
		for (int i = 0; i < hullcount; i++)
		{
			Point pt = points_01[hull[i]];
			line(ContoursIm_1, poi_01, pt, Scalar(255), 3, CV_AA);
			poi_01 = pt;
		}
		namedWindow("【8.Convex Hull】", WINDOW_NORMAL);
		imshow("【8.Convex Hull】", ContoursIm_1);
		


		//-----------------------------------【测试参数】---------------------------------------    
		//      描述：齿数    
		//-----------------------------------------------------------------------------------------------  

		//【1】输出齿数  
		cout << "齿数：" << hull.size() << "\n" << endl;

	//时间显示
	et1 = cvGetTickCount();
	cout << "times cost:" << (et1 - st1) / (double)cvGetTickFrequency() / 1000.0 << "milliseconds\n\n";

	waitKey(0);
	return 0;
}



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