# Kinect获取图像、点云

本文采用的是KinectV2。开篇之前，推荐一个学习Kinect的网站：[4. Kinect 点云 — Kinect Tutorials 0.1 文档 (kinect-tutorials-zh.readthedocs.io)](https://kinect-tutorials-zh.readthedocs.io/zh_CN/latest/kinect2/3_PointCloud.html)

开发基于Kinect for Windows SDK 2.0、Opencv与PCL，Kinect for Windows SDK 2.0获取网址：[Download Kinect for Windows SDK 2.0 from Official Microsoft Download Center](https://www.microsoft.com/en-us/download/details.aspx?id=44561)

配好环境之后尝试

```c++
#include <Kinect.h> 
```

无报错即Kinect环境配置成功。

#### Kinect获取点云的逻辑

1、获取RGB图像（1920*1080）

2、获取深度图像（512*424）

3、深度、RGB图对齐（获得RGBA图）

4、深度图、相机映射

#### Kinect获取数据步骤

初始化：1、获取设备

​				2、打开设备

​				3、获取数据源（RGB、深度、BodyFrame等）

获取数据帧

变量定义

```C++
HRESULT resultc;		
HRESULT resultd;	//状态参数
IKinectSensor* Kinect;             // Kinect sensor
int colorHeight = 0, colorWidth = 0;	//分辨率
int depthHeight = 0, depthWidth = 0;	
IDepthFrameSource* depthSource = nullptr;   //深度数据源
IColorFrameSource* colorSource = nullptr;	//RGB数据源
IDepthFrameReader* depthReader = nullptr;	//深度数据的Reader
IColorFrameReader* colorReader = nullptr;	//RGB数据的Reader
IColorFrame* colorFrame = nullptr;	//RGB帧数据
IDepthFrame* depthFrame = nullptr;		//深度帧数据
ICoordinateMapper* CoordinateMapper;	//坐标映射器
UINT16* depthData = new UINT16[424 * 512];	//深度数据
ColorSpacePoint* colorData = new ColorSpacePoint[424 * 512];	//对齐的彩色数据
DepthSpacePoint* color2DepthData = new DepthSpacePoint[1920 * 1080];	//对齐的深度数据
CameraSpacePoint* depth2Cam = new CameraSpacePoint[424 * 512];	//坐标系数据
//图像输出
cv::Mat color_img;	//RGB图像
cv::Mat color_flip;	//RGB翻转图像
cv::Mat temp;	//十六位深度图
cv::Mat depth_img;	//八位深度图
cv::Mat depthToRGB;	//深度与RGB对齐的图
cv::Mat depthRGB;

//点云输出
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
```

初始化

```C++
void initKinect(){
    //获取Kinect
    resultc = GetDefaultKinectSensor(&Kinect);  
    if (SUCCEEDED(resultc))
        std::cout<<"成功获取设备"<<std::endl;
    else
        std::cout << "无法获取设备" << std::endl;
    //打开Kinect
    resultc = Kinect->Open();           
    if (SUCCEEDED(resultc))
        std::cout << "成功打开设备" << std::endl;
    else
        std::cout << "无法打开设备" << std::endl;
    //获取数据源
    resultc = Kinect->get_DepthFrameSource(&depthSource);
    resultd = Kinect->get_ColorFrameSource(&colorSource);
    if (SUCCEEDED(resultc) && SUCCEEDED(resultd))
        std::cout << "成功获取数据源" << std::endl;
    else
        std::cout << "获取数据源失败" << std::endl;
    //！！！！PS:貌似测试出来HRESULT没什么作用，因此下面就不判断了！！！！
    //获得RGB数据的分辨率(初始化)
    IFrameDescription* colorDescription = nullptr;
    colorSource->get_FrameDescription(&colorDescription);
    colorDescription->get_Height(&colorHeight);
    colorDescription->get_Width(&colorWidth);
    colorDescription->Release();	//释放
    colorSource->OpenReader(&colorReader);
    //取得深度数据的分辨率(初始化)
    IFrameDescription* myDescription = nullptr;
    depthSource->get_FrameDescription(&myDescription);
    myDescription->get_Height(&depthHeight);
    myDescription->get_Width(&depthWidth);
    myDescription->Release();
    depthSource->OpenReader(&depthReader);    //打开深度数据的Reader
    //输出图像初始化
    color_img = cv::Mat(colorHeight, colorWidth, CV_8UC4);	//RGB
    depth_img = cv::Mat(depthHeight, depthWidth, CV_8UC1);	//深度(可视化)
    temp = cv::Mat(depthHeight, depthWidth, CV_16UC1);    //深度
    depthToRGB = cv::Mat(depthHeight, depthWidth, CV_8UC4);	//对齐的图（4通道）
    depthRGB = cv::Mat(depthHeight, depthWidth, CV_8UC3);	//对齐的图（3通道）

    Kinect->get_CoordinateMapper(&CoordinateMapper);	//获取坐标映射器
    //初始化输出点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud = cloudPtr;
}
```

获取数据帧

```C++
int main()
{
	//初始化Kinect
	initKinect();
	//初始化点云显示工具
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("show"));
	viewer->setBackgroundColor(0.5, 0.5, 0.5);  //设置背景
	viewer->addCoordinateSystem(1, "Base_link");  //设置坐标轴尺寸
	//记录时间戳
	auto start = std::chrono::steady_clock::now();
	while (true) {
		//获取深度图
		if (depthReader->AcquireLatestFrame(&depthFrame) == S_OK) { //通过Reader尝试获取最新的一帧深度数据，放入深度帧中,并判断是否成功获取
			depthFrame->CopyFrameDataToArray(424 * 512, depthData);
			depthFrame->CopyFrameDataToArray(depthHeight * depthWidth, (UINT16*)temp.data); //先把数据存入16位的图像矩阵
			temp.convertTo(depth_img, CV_8UC1, 255.0 / 4500);   //再把16位转换为8位
			imshow("Depth", depth_img);
			depthFrame->Release();
		}
		//获取RGB图
		if (colorReader->AcquireLatestFrame(&colorFrame) == S_OK) {
			colorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, (BYTE*)color_img.data, ColorImageFormat::ColorImageFormat_Bgra);	//获取彩色图
			imshow("RGB", color_img);
			colorFrame->Release();
		}
		cv::flip(color_img, color_flip, 1);	//图片翻转
		//深度与RGB对齐
		CoordinateMapper->MapColorFrameToDepthSpace(512 * 424, reinterpret_cast<UINT16*>(temp.data), 1920 * 1080, color2DepthData);
		CoordinateMapper->MapDepthFrameToColorSpace(512 * 424, reinterpret_cast<UINT16*>(temp.data), 512 * 424, colorData);
		for (int i = 0; i < 512 * 424; i++) {
			ColorSpacePoint p = colorData[i];
			if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
			{
				int colorX = static_cast<int>(p.X);
				int colorY = static_cast<int>(p.Y);
				if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
				{
					depthToRGB.data[i * 4] = color_img.data[(colorY * 1920 + colorX) * 4];
					depthToRGB.data[i * 4 + 1] = color_img.data[(colorY * 1920 + colorX) * 4 + 1];
					depthToRGB.data[i * 4 + 2] = color_img.data[(colorY * 1920 + colorX) * 4 + 2];
					depthToRGB.data[i * 4 + 3] = color_img.data[(colorY * 1920 + colorX) * 4 + 3];
					depthRGB.data[i * 3] = color_img.data[(colorY * 1920 + colorX) * 4];
					depthRGB.data[i * 3 + 1] = color_img.data[(colorY * 1920 + colorX) * 4 + 1];
					depthRGB.data[i * 3 + 2] = color_img.data[(colorY * 1920 + colorX) * 4 + 2];
				}
			}
			else {
				depthRGB.data[i * 3] = 0;
				depthRGB.data[i * 3 + 1] = 0;
				depthRGB.data[i * 3 + 2] = 0;
			}
		}
		//转点云
		CoordinateMapper->MapDepthFrameToCameraSpace(512 * 424, reinterpret_cast<UINT16*>(temp.data), 512 * 424, depth2Cam);
		cloud->width = 512 * 424;
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);
		for (size_t i = 0; i < 512; i++) {
			for (size_t j = 0; j < 424; j++) {
				pcl::PointXYZRGB pointTemp;
				if (depth2Cam[i + j * 512].Z > 0.5 && depth2Cam[i + j * 512].Z < 5) {
					pointTemp.x = depth2Cam[i + j * 512].X;
					pointTemp.y = depth2Cam[i + j * 512].Y;
					pointTemp.z = depth2Cam[i + j * 512].Z;
					int X = static_cast<int>(colorData[j * 512 + i].X);
					int Y = static_cast<int>(colorData[j * 512 + i].Y);
					if (X > 0 && Y > 0 && X < 1920 && Y < 1080)
					{
						cv::Vec4b* pixelsRGBImage = color_img.ptr<cv::Vec4b>(Y);
						pointTemp.g = pixelsRGBImage[X][0];
						pointTemp.b = pixelsRGBImage[X][1];
						pointTemp.r = pixelsRGBImage[X][2];
						cloud->points.push_back(pointTemp);
					}
					else continue;
				}
			}
		}
		//点云可视化
		if (cloud->points.size() != 0) {
			viewer->removeAllPointClouds();
			viewer->removeAllShapes();
			viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "origin_cloud");  //显示原始点云
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.25, "origin_cloud");  //设置点尺寸
		}
		viewer->spinOnce();
	}
	depthReader->Release();        //释放不用的变量并且关闭感应器
	colorReader->Release();
	colorSource->Release();
	depthSource->Release();
	Kinect->Close();
	Kinect->Release();
}
```

#### 代码效果

RGB

<img src=".\picture\RGB 2023_4_2 10_14_31.png" alt="RGB 2023_4_2 10_14_31" style="zoom:80%;" />

深度

![Depth 2023_4_2 10_15_00](.\picture\Depth 2023_4_2 10_15_00.png)

点云

<img src=".\picture\show 2023_4_2 10_15_28.png" alt="show 2023_4_2 10_15_28" style="zoom:80%;" />

#### 完整代码

地址：[mai4567/KinectDemo: grabing point cloud from KinectV2 (github.com)](https://github.com/mai4567/KinectDemo)

```c++
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

HRESULT resultc;
HRESULT resultd;	//状态参数
IKinectSensor* Kinect;             // Kinect sensor
int colorHeight = 0, colorWidth = 0;	//分辨率
int depthHeight = 0, depthWidth = 0;
IDepthFrameSource* depthSource = nullptr;   //深度数据源
IColorFrameSource* colorSource = nullptr;	//RGB数据源
IDepthFrameReader* depthReader = nullptr;	//深度数据的Reader
IColorFrameReader* colorReader = nullptr;	//RGB数据的Reader
IColorFrame* colorFrame = nullptr;	//RGB帧数据
IDepthFrame* depthFrame = nullptr;		//深度帧数据
ICoordinateMapper* CoordinateMapper;	//坐标映射器
UINT16* depthData = new UINT16[424 * 512];	//深度数据
ColorSpacePoint* colorData = new ColorSpacePoint[424 * 512];	//对齐的彩色数据
DepthSpacePoint* color2DepthData = new DepthSpacePoint[1920 * 1080];	//对齐的深度数据
CameraSpacePoint* depth2Cam = new CameraSpacePoint[424 * 512];	//坐标系数据
//图像输出
cv::Mat color_img;	//RGB图像
cv::Mat color_flip;	//RGB翻转图像
cv::Mat temp;	//十六位深度图
cv::Mat depth_img;	//八位深度图
cv::Mat depthToRGB;	//深度与RGB对齐的图
cv::Mat depthRGB;

//点云输出
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

void initKinect() {
    //获取Kinect
    resultc = GetDefaultKinectSensor(&Kinect);
    if (SUCCEEDED(resultc))
        std::cout << "成功获取设备" << std::endl;
    else
        std::cout << "无法获取设备" << std::endl;
    //打开Kinect
    resultc = Kinect->Open();
    if (SUCCEEDED(resultc))
        std::cout << "成功打开设备" << std::endl;
    else
        std::cout << "无法打开设备" << std::endl;
    //获取数据源
    resultc = Kinect->get_DepthFrameSource(&depthSource);
    resultd = Kinect->get_ColorFrameSource(&colorSource);
    if (SUCCEEDED(resultc) && SUCCEEDED(resultd))
        std::cout << "成功获取数据源" << std::endl;
    else
        std::cout << "获取数据源失败" << std::endl;
    //！！！！PS:貌似测试出来HRESULT没什么作用，因此下面就不判断了！！！！
    //获得RGB数据的分辨率(初始化)
    IFrameDescription* colorDescription = nullptr;
    colorSource->get_FrameDescription(&colorDescription);
    colorDescription->get_Height(&colorHeight);
    colorDescription->get_Width(&colorWidth);
    colorDescription->Release();	//释放
    colorSource->OpenReader(&colorReader);
    //取得深度数据的分辨率(初始化)
    IFrameDescription* myDescription = nullptr;
    depthSource->get_FrameDescription(&myDescription);
    myDescription->get_Height(&depthHeight);
    myDescription->get_Width(&depthWidth);
    myDescription->Release();
    depthSource->OpenReader(&depthReader);    //打开深度数据的Reader
    //输出图像初始化
    color_img = cv::Mat(colorHeight, colorWidth, CV_8UC4);	//RGB
    depth_img = cv::Mat(depthHeight, depthWidth, CV_8UC1);	//深度(可视化)
    temp = cv::Mat(depthHeight, depthWidth, CV_16UC1);    //深度
    depthToRGB = cv::Mat(depthHeight, depthWidth, CV_8UC4);	//对齐的图（4通道）
    depthRGB = cv::Mat(depthHeight, depthWidth, CV_8UC3);	//对齐的图（3通道）

    Kinect->get_CoordinateMapper(&CoordinateMapper);	//获取坐标映射器
    //初始化输出点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud = cloudPtr;
}

int main()
{
	//初始化Kinect
	initKinect();
	//初始化点云显示工具
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("show"));
	viewer->setBackgroundColor(0.5, 0.5, 0.5);  //设置背景
	viewer->addCoordinateSystem(1, "Base_link");  //设置坐标轴尺寸
	//记录时间戳
	auto start = std::chrono::steady_clock::now();
	while (true) {
		//获取深度图
		if (depthReader->AcquireLatestFrame(&depthFrame) == S_OK) { //通过Reader尝试获取最新的一帧深度数据，放入深度帧中,并判断是否成功获取
			depthFrame->CopyFrameDataToArray(424 * 512, depthData);
			depthFrame->CopyFrameDataToArray(depthHeight * depthWidth, (UINT16*)temp.data); //先把数据存入16位的图像矩阵
			temp.convertTo(depth_img, CV_8UC1, 255.0 / 4500);   //再把16位转换为8位
			imshow("Depth", depth_img);
			depthFrame->Release();
		}
		//获取RGB图
		if (colorReader->AcquireLatestFrame(&colorFrame) == S_OK) {
			colorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, (BYTE*)color_img.data, ColorImageFormat::ColorImageFormat_Bgra);	//获取彩色图
			imshow("RGB", color_img);
			colorFrame->Release();
		}
		cv::flip(color_img, color_flip, 1);	//图片翻转
		//深度与RGB对齐
		CoordinateMapper->MapColorFrameToDepthSpace(512 * 424, reinterpret_cast<UINT16*>(temp.data), 1920 * 1080, color2DepthData);
		CoordinateMapper->MapDepthFrameToColorSpace(512 * 424, reinterpret_cast<UINT16*>(temp.data), 512 * 424, colorData);
		for (int i = 0; i < 512 * 424; i++) {
			ColorSpacePoint p = colorData[i];
			if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
			{
				int colorX = static_cast<int>(p.X);
				int colorY = static_cast<int>(p.Y);
				if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
				{
					depthToRGB.data[i * 4] = color_img.data[(colorY * 1920 + colorX) * 4];
					depthToRGB.data[i * 4 + 1] = color_img.data[(colorY * 1920 + colorX) * 4 + 1];
					depthToRGB.data[i * 4 + 2] = color_img.data[(colorY * 1920 + colorX) * 4 + 2];
					depthToRGB.data[i * 4 + 3] = color_img.data[(colorY * 1920 + colorX) * 4 + 3];
					depthRGB.data[i * 3] = color_img.data[(colorY * 1920 + colorX) * 4];
					depthRGB.data[i * 3 + 1] = color_img.data[(colorY * 1920 + colorX) * 4 + 1];
					depthRGB.data[i * 3 + 2] = color_img.data[(colorY * 1920 + colorX) * 4 + 2];
				}
			}
			else {
				depthRGB.data[i * 3] = 0;
				depthRGB.data[i * 3 + 1] = 0;
				depthRGB.data[i * 3 + 2] = 0;
			}
		}
		//转点云
		CoordinateMapper->MapDepthFrameToCameraSpace(512 * 424, reinterpret_cast<UINT16*>(temp.data), 512 * 424, depth2Cam);
		cloud->width = 512 * 424;
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);
		for (size_t i = 0; i < 512; i++) {
			for (size_t j = 0; j < 424; j++) {
				pcl::PointXYZRGB pointTemp;
				if (depth2Cam[i + j * 512].Z > 0.5 && depth2Cam[i + j * 512].Z < 5) {
					pointTemp.x = depth2Cam[i + j * 512].X;
					pointTemp.y = depth2Cam[i + j * 512].Y;
					pointTemp.z = depth2Cam[i + j * 512].Z;
					int X = static_cast<int>(colorData[j * 512 + i].X);
					int Y = static_cast<int>(colorData[j * 512 + i].Y);
					if (X > 0 && Y > 0 && X < 1920 && Y < 1080)
					{
						cv::Vec4b* pixelsRGBImage = color_img.ptr<cv::Vec4b>(Y);
						pointTemp.g = pixelsRGBImage[X][0];
						pointTemp.b = pixelsRGBImage[X][1];
						pointTemp.r = pixelsRGBImage[X][2];
						cloud->points.push_back(pointTemp);
					}
					else continue;
				}
			}
		}
		//点云可视化
		if (cloud->points.size() != 0) {
			viewer->removeAllPointClouds();
			viewer->removeAllShapes();
			viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "origin_cloud");  //显示原始点云
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.25, "origin_cloud");  //设置点尺寸
		}
		viewer->spinOnce();
	}
	depthReader->Release();        //释放不用的变量并且关闭感应器
	colorReader->Release();
	colorSource->Release();
	depthSource->Release();
	Kinect->Close();
	Kinect->Release();
}

```

#### 注意事项

RGB图像与深度图像与真实场景是一个镜像关系，所以代码处留了opencv的flip函数对图像进行翻转。点云的方向与真实场景相同，点云是基于相机坐标系生成的，我的相机位置是斜向地板的，因此点云坐标系z轴指向地板。

上面的完整代码只是一个简单demo，我个人比较建议将变量和函数封装成一个类，用多线程跑相机获取数据。但是采用多线程务必记得加锁，否则容易出现bug。