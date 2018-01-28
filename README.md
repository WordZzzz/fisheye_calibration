---
title: 鱼眼摄像头标定与畸变校正（双OPENCV版本）
comments: true
mathjax: true
categories:
  - OPENCV
tags:
  - OPENCV2
  - OPENCV3
  - 畸变校正
---

----------

- **转载请注明作者和出处：http://blog.csdn.net/u011475210**
- **代码地址：https://github.com/WordZzzz/fisheye_calibration**
- **软件版本：VS2013+OPENCV2.4.13 OR VS2013+OPENCV3.4.0**
- **编&emsp;&emsp;者：WordZzzz**

----------

[toc]

## 我的代码

&emsp;&emsp;最近在整理自己以前做过的一些东西，这是基于opencv的鱼眼摄像头畸变校正程序的[github地址](https://github.com/WordZzzz/fisheye_calibration)。

其中：

- normal_calibrate：基于OPENCV2与OPENCV3通用的函数实现，可实现USB摄像头实时畸变校正；
- fishey_calibrate：基于OPENCV3独有的fishyey结构体实现，可实现USB摄像头实时畸变校正；
- fishey_calibrate_img：基于OPENCV3独有的fishyey结构体实现，可实现单张图片畸变校正；

&emsp;&emsp;opencv1.0 2.0版只有一种摄像机标定模型，就是普通的小孔成像模型，在cv：：空间下。而从opencv3.0开始，新增了一种鱼眼相机标定模型，在fisheye::空间下。两种模型的主要区别在于像与物的投影关系不同，具体的文献资料依然是数不胜数，这里就不赘述。根据opencv官方文档的建议，在畸变程度较大的广角镜头（比如：鱼眼镜头）上进行摄像机标定和畸变校正，最好是用fisheye模型，该模型在图像边缘畸变程度很大的地方比普通相机模型的效果要好。

&emsp;&emsp;当然，还是要贴上官方文档的：

- [Camera calibration With OpenCV2](https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html)；
- [Camera calibration With OpenCV3](https://docs.opencv.org/trunk/db/d58/group__calib3d__fisheye.html)；

----------

华丽的分割线

----------

## 可供参考资料

&emsp;&emsp;以下是在写这篇博客的时候偶然发现的对opencv两个版本标定过程的讲解，这里直接copy过来并稍微做了一下排版，因为自己忙于为找工作做准备，实在是没时间自己整理了。[原文链接](http://www.cnblogs.com/riddick/p/7811877.html)。

&emsp;&emsp;图像算法中会经常用到摄像机的畸变校正，有必要总结分析OpenCV中畸变校正方法，其中包括普通针孔相机模型和鱼眼相机模型fisheye两种畸变校正方法。

&emsp;&emsp;普通相机模型畸变校正函数针对OpenCV中的cv::initUndistortRectifyMap()，鱼眼相机模型畸变校正函数对应OpenCV中的cv::fisheye::initUndistortRectifyMap()。两种方法算出映射Mapx和Mapy后，统一用cv::Remap()函数进行插值得到校正后的图像。



### FishEye模型的畸变校正。

&emsp;&emsp;方便起见，直接贴出OpenCV源码，我在里面加了注释说明。建议参考[OpenCV官方文档](https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#fisheye-initundistortrectifymap)看畸变模型原理会更清楚。

&emsp;&emsp;简要流程就是：

- 1.求内参矩阵的逆，由于摄像机坐标系的三维点到二维图像平面，需要乘以旋转矩阵R和内参矩阵K。那么反向投影回去则是二维图像坐标乘以  K*R的逆矩阵。

- 2.将目标图像中的每一个像素点坐标(j,i)，乘以1中求出的逆矩阵iR，转换到摄像机坐标系（_x,_y,_w）,并归一化得到z=1平面下的三维坐标(x,y,1)。

- 3.求出平面模型下像素点对应鱼眼半球模型下的极坐标(r, theta)。

- 4.利用鱼眼畸变模型求出拥有畸变时像素点对应的theta_d。

<p></p>
<div align=center><img src="http://img.blog.csdn.net/20180128134953810?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvdTAxMTQ3NTIxMA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast"/></div>
<p></p>

- 5.利用求出的theta_d值将三维坐标点重投影到二维图像平面得到(u,v)，(u,v)即为目标图像对应的畸变图像中像素点坐标。

- 6.使用cv::Remap（）函数，根据mapx,mapy取出对应坐标位置的像素值赋值给目标图像，一般采用双线性插值法，得到畸变校正后的目标图像。

 

```c
#include <opencv2\opencv.hpp>

void cv::fisheye::initUndistortRectifyMap( InputArray K, InputArray D, InputArray R, InputArray P,
    const cv::Size& size, int m1type, OutputArray map1, OutputArray map2 )
{
    CV_Assert( m1type == CV_16SC2 || m1type == CV_32F || m1type <=0 );
    map1.create( size, m1type <= 0 ? CV_16SC2 : m1type );
    map2.create( size, map1.type() == CV_16SC2 ? CV_16UC1 : CV_32F );

    CV_Assert((K.depth() == CV_32F || K.depth() == CV_64F) && (D.depth() == CV_32F || D.depth() == CV_64F));
    CV_Assert((P.empty() || P.depth() == CV_32F || P.depth() == CV_64F) && (R.empty() || R.depth() == CV_32F || R.depth() == CV_64F));
    CV_Assert(K.size() == Size(3, 3) && (D.empty() || D.total() == 4));
    CV_Assert(R.empty() || R.size() == Size(3, 3) || R.total() * R.channels() == 3);
    CV_Assert(P.empty() || P.size() == Size(3, 3) || P.size() == Size(4, 3));

    //从内参矩阵K中取出归一化焦距fx,fy; cx,cy
    cv::Vec2d f, c;
    if (K.depth() == CV_32F)
    {
        Matx33f camMat = K.getMat();
        f = Vec2f(camMat(0, 0), camMat(1, 1));
        c = Vec2f(camMat(0, 2), camMat(1, 2));
    }
    else
    {
        Matx33d camMat = K.getMat();
        f = Vec2d(camMat(0, 0), camMat(1, 1));
        c = Vec2d(camMat(0, 2), camMat(1, 2));
    }
    //从畸变系数矩阵D中取出畸变系数k1,k2,k3,k4
    Vec4d k = Vec4d::all(0);
    if (!D.empty())
        k = D.depth() == CV_32F ? (Vec4d)*D.getMat().ptr<Vec4f>(): *D.getMat().ptr<Vec4d>();

    //旋转矩阵RR转换数据类型为CV_64F，如果不需要旋转，则RR为单位阵
    cv::Matx33d RR  = cv::Matx33d::eye();
    if (!R.empty() && R.total() * R.channels() == 3)
    {
        cv::Vec3d rvec;
        R.getMat().convertTo(rvec, CV_64F);
        RR = Affine3d(rvec).rotation();
    }
    else if (!R.empty() && R.size() == Size(3, 3))
        R.getMat().convertTo(RR, CV_64F);
    
    //新的内参矩阵PP转换数据类型为CV_64F
    cv::Matx33d PP = cv::Matx33d::eye();
    if (!P.empty())
        P.getMat().colRange(0, 3).convertTo(PP, CV_64F);

    //关键一步：新的内参矩阵*旋转矩阵，然后利用SVD分解求出逆矩阵iR，后面用到
    cv::Matx33d iR = (PP * RR).inv(cv::DECOMP_SVD);

    //反向映射，遍历目标图像所有像素位置，找到畸变图像中对应位置坐标(u,v)，并分别保存坐标(u,v)到mapx和mapy中
    for( int i = 0; i < size.height; ++i)
    {
        float* m1f = map1.getMat().ptr<float>(i);
        float* m2f = map2.getMat().ptr<float>(i);
        short*  m1 = (short*)m1f;
        ushort* m2 = (ushort*)m2f;

        //二维图像平面坐标系->摄像机坐标系
        double _x = i*iR(0, 1) + iR(0, 2),
               _y = i*iR(1, 1) + iR(1, 2),
               _w = i*iR(2, 1) + iR(2, 2);

        for( int j = 0; j < size.width; ++j)
        {
            //归一化摄像机坐标系，相当于假定在Z=1平面上
            double x = _x/_w, y = _y/_w;

            //求鱼眼半球体截面半径r
            double r = sqrt(x*x + y*y);
            //求鱼眼半球面上一点与光心的连线和光轴的夹角Theta
            double theta = atan(r);
            //畸变模型求出theta_d，相当于有畸变的角度值
            double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
            double theta_d = theta * (1 + k[0]*theta2 + k[1]*theta4 + k[2]*theta6 + k[3]*theta8);
            //利用有畸变的Theta值，将摄像机坐标系下的归一化三维坐标，重投影到二维图像平面，得到(j,i)对应畸变图像中的(u,v)
            double scale = (r == 0) ? 1.0 : theta_d / r;
            double u = f[0]*x*scale + c[0];
            double v = f[1]*y*scale + c[1];

            //保存(u,v)坐标到mapx,mapy
            if( m1type == CV_16SC2 )
            {
                int iu = cv::saturate_cast<int>(u*cv::INTER_TAB_SIZE);
                int iv = cv::saturate_cast<int>(v*cv::INTER_TAB_SIZE);
                m1[j*2+0] = (short)(iu >> cv::INTER_BITS);
                m1[j*2+1] = (short)(iv >> cv::INTER_BITS);
                m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE-1))*cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE-1)));
            }
            else if( m1type == CV_32FC1 )
            {
                m1f[j] = (float)u;
                m2f[j] = (float)v;
            }

            //这三条语句是上面 ”//二维图像平面坐标系->摄像机坐标系“的一部分，是矩阵iR的第一列，这样写能够简化计算
            _x += iR(0, 0);
            _y += iR(1, 0);
            _w += iR(2, 0);
        }
    }
}
```
 

### 普通相机模型的畸变校正

&emsp;&emsp;同样建议参考[OpenCV官方文档](https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)阅读代码。

&emsp;&emsp;主要流程和上面Fisheye模型差不多，只有第4部分的畸变模型不一样，普通相机的畸变模型如下：

<p></p>
<div align=center><img src="http://img.blog.csdn.net/20180128135032366?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvdTAxMTQ3NTIxMA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast"/></div>
<p></p> 

&emsp;&emsp;同样把源代码贴上，并加上注解：

```c
#include <opencv2\opencv.hpp>

void cv::initUndistortRectifyMap( InputArray _cameraMatrix, InputArray _distCoeffs,
                              InputArray _matR, InputArray _newCameraMatrix,
                              Size size, int m1type, OutputArray _map1, OutputArray _map2 )
{
    Mat cameraMatrix = _cameraMatrix.getMat(), distCoeffs = _distCoeffs.getMat();
    Mat matR = _matR.getMat(), newCameraMatrix = _newCameraMatrix.getMat();

    if( m1type <= 0 )
        m1type = CV_16SC2;
    CV_Assert( m1type == CV_16SC2 || m1type == CV_32FC1 || m1type == CV_32FC2 );
    _map1.create( size, m1type );
    Mat map1 = _map1.getMat(), map2;
    if( m1type != CV_32FC2 )
    {
        _map2.create( size, m1type == CV_16SC2 ? CV_16UC1 : CV_32FC1 );
        map2 = _map2.getMat();
    }
    else
        _map2.release();

    Mat_<double> R = Mat_<double>::eye(3, 3);
    Mat_<double> A = Mat_<double>(cameraMatrix), Ar;

    if( !newCameraMatrix.empty() )
        Ar = Mat_<double>(newCameraMatrix);
    else
        Ar = getDefaultNewCameraMatrix( A, size, true );

    if( !matR.empty() )
        R = Mat_<double>(matR);

    if( !distCoeffs.empty() )
        distCoeffs = Mat_<double>(distCoeffs);
    else
    {
        distCoeffs.create(14, 1, CV_64F);
        distCoeffs = 0.;
    }

    CV_Assert( A.size() == Size(3,3) && A.size() == R.size() );
    CV_Assert( Ar.size() == Size(3,3) || Ar.size() == Size(4, 3));

    //LU分解求新的内参矩阵Ar与旋转矩阵R乘积的逆矩阵iR
    Mat_<double> iR = (Ar.colRange(0,3)*R).inv(DECOMP_LU);
    const double* ir = &iR(0,0);

    //从旧的内参矩阵中取出光心位置u0,v0,和归一化焦距fx,fy
    double u0 = A(0, 2),  v0 = A(1, 2);
    double fx = A(0, 0),  fy = A(1, 1);

    //尼玛14个畸变系数，不过大多用到的只有(k1,k2,p1,p2)，最多加一个k3，用不到的置为0
    CV_Assert( distCoeffs.size() == Size(1, 4) || distCoeffs.size() == Size(4, 1) ||
               distCoeffs.size() == Size(1, 5) || distCoeffs.size() == Size(5, 1) ||
               distCoeffs.size() == Size(1, 8) || distCoeffs.size() == Size(8, 1) ||
               distCoeffs.size() == Size(1, 12) || distCoeffs.size() == Size(12, 1) ||
               distCoeffs.size() == Size(1, 14) || distCoeffs.size() == Size(14, 1));

    if( distCoeffs.rows != 1 && !distCoeffs.isContinuous() )
        distCoeffs = distCoeffs.t();

    const double* const distPtr = distCoeffs.ptr<double>();
    double k1 = distPtr[0];
    double k2 = distPtr[1];
    double p1 = distPtr[2];
    double p2 = distPtr[3];
    double k3 = distCoeffs.cols + distCoeffs.rows - 1 >= 5 ? distPtr[4] : 0.;
    double k4 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? distPtr[5] : 0.;
    double k5 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? distPtr[6] : 0.;
    double k6 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? distPtr[7] : 0.;
    double s1 = distCoeffs.cols + distCoeffs.rows - 1 >= 12 ? distPtr[8] : 0.;
    double s2 = distCoeffs.cols + distCoeffs.rows - 1 >= 12 ? distPtr[9] : 0.;
    double s3 = distCoeffs.cols + distCoeffs.rows - 1 >= 12 ? distPtr[10] : 0.;
    double s4 = distCoeffs.cols + distCoeffs.rows - 1 >= 12 ? distPtr[11] : 0.;
    double tauX = distCoeffs.cols + distCoeffs.rows - 1 >= 14 ? distPtr[12] : 0.;
    double tauY = distCoeffs.cols + distCoeffs.rows - 1 >= 14 ? distPtr[13] : 0.;

    //tauX,tauY这个是什么梯形畸变，用不到的话matTilt为单位阵
    // Matrix for trapezoidal distortion of tilted image sensor
    cv::Matx33d matTilt = cv::Matx33d::eye();
    cv::detail::computeTiltProjectionMatrix(tauX, tauY, &matTilt);

    for( int i = 0; i < size.height; i++ )
    {
        float* m1f = map1.ptr<float>(i);
        float* m2f = map2.empty() ? 0 : map2.ptr<float>(i);
        short* m1 = (short*)m1f;
        ushort* m2 = (ushort*)m2f;

        //利用逆矩阵iR将二维图像坐标(j,i)转换到摄像机坐标系(_x,_y,_w)
        double _x = i*ir[1] + ir[2], _y = i*ir[4] + ir[5], _w = i*ir[7] + ir[8];

        for( int j = 0; j < size.width; j++, _x += ir[0], _y += ir[3], _w += ir[6] )
        {
            //摄像机坐标系归一化，令Z=1平面
            double w = 1./_w, x = _x*w, y = _y*w;
             //这一部分请看OpenCV官方文档，畸变模型部分
            double x2 = x*x, y2 = y*y;
            double r2 = x2 + y2, _2xy = 2*x*y;
            double kr = (1 + ((k3*r2 + k2)*r2 + k1)*r2)/(1 + ((k6*r2 + k5)*r2 + k4)*r2);
            double xd = (x*kr + p1*_2xy + p2*(r2 + 2*x2) + s1*r2+s2*r2*r2);
            double yd = (y*kr + p1*(r2 + 2*y2) + p2*_2xy + s3*r2+s4*r2*r2);
           //根据求取的xd,yd将三维坐标重投影到二维畸变图像坐标(u,v)
            cv::Vec3d vecTilt = matTilt*cv::Vec3d(xd, yd, 1);
            double invProj = vecTilt(2) ? 1./vecTilt(2) : 1;
            double u = fx*invProj*vecTilt(0) + u0;
            double v = fy*invProj*vecTilt(1) + v0;
            //保存u,v的值到Mapx,Mapy中
            if( m1type == CV_16SC2 )
            {
                int iu = saturate_cast<int>(u*INTER_TAB_SIZE);
                int iv = saturate_cast<int>(v*INTER_TAB_SIZE);
                m1[j*2] = (short)(iu >> INTER_BITS);
                m1[j*2+1] = (short)(iv >> INTER_BITS);
                m2[j] = (ushort)((iv & (INTER_TAB_SIZE-1))*INTER_TAB_SIZE + (iu & (INTER_TAB_SIZE-1)));
            }
            else if( m1type == CV_32FC1 )
            {
                m1f[j] = (float)u;
                m2f[j] = (float)v;
            }
            else
            {
                m1f[j*2] = (float)u;
                m1f[j*2+1] = (float)v;
            }
        }
    }
}
```

**<font color="red" size=3 face="仿宋">希望对需要的人能有所帮助，欢迎订阅、关注、收藏、评论、点赞哦～～(￣▽￣～)～</font>**

**<font color="red" size=3 face="仿宋">完的汪(∪｡∪)｡｡｡zzz</font>**
