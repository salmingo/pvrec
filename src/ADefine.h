/**
 Name        : ADefine.h 常量及宏定义声明文件
 Author      : Xiaomeng Lu
 Version     : 0.1
 Date        : Oct 13, 2012
 Last Date   : Oct 13, 2012
 Description : 天文数字图像处理中常用的常量及宏定义
 **/
#ifndef _ADEFINE_H_
#define _ADEFINE_H_

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>
#include <vector>
#include <string>
#include <algorithm>

#if !defined(WIN32) && !defined(_WINDOWS_)	// Linux平台下使用该类库
#include <unistd.h>
#else
#include <io.h>								// Windows平台使用该类库
#endif

namespace AstroUtil
{
///////////////////////////////////////////////////////////////////////////////
/********************************** 宏定义 **********************************/
#if !defined(BZERO)	// 内存清零
#define BZERO(ptr, szbyte)	memset(ptr, 0, szbyte)
#endif

#if !defined(BYTE)
typedef unsigned char BYTE;
#endif

#if !defined(USHORT)
typedef unsigned short USHORT;
#endif

#if !defined(UINT)
typedef unsigned int UINT;
#endif

#ifndef WORD
typedef unsigned short WORD;
#endif

// 圆周率相关常量
#if !defined(API)	// 圆周率
#define API		3.141592653589793
#endif
#define PI45	0.785398163397448
#define PI90	1.570796326794897
#define PI180	API
#define PI270	4.712388980384690
#define PI360	6.283185307179586
#define GtoR	0.017453292519943		// 使用乘法, 角度转换为弧度的系数
#define RtoG	57.295779513082323		// 使用乘法, 弧度转换为角度的系数
#define StoR	4.848136811095360e-06	// 使用乘法, 角秒转换为弧度的系数
#define RtoS	2.062648062470964e+05	// 使用乘法, 弧度转换为角秒的系数
#define ASEC	StoR			// 角秒对应的弧度
// 时间相关常量
#define MILLISEC		3600000.0		// 1度对应的毫角秒
#define MILLISEC5		18000000.0		// 5度对应的毫角秒
#define MILLISEC90		324000000.0		// 90度对应的毫角秒
#define MILLISEC180		648000000.0		// 180度对应的毫角秒
#define MILLISEC360		1296000000.0	// 360度对应的毫角秒

// 软件中限定最大/最小数
#define AMAX	1E30
#define AEPS 	1E-4

#define cot(x)	(tan(PI90 - (x)))	// 计算弧度对应的余切

///////////////////////////////////////////////////////////////////////////////
// 定义与测站位置有关信息
typedef struct param_mount_site{
	double	lgt;	// 地理经度, 量纲: 弧度
	double	lat;	// 地理纬度, 量纲: 弧度
	double	alt;	// 海拔高度, 量纲: 米
	double	airp;	// 大气压, 量纲: 百帕
	double	temp;	// 环境温度, 量纲: 摄氏度
	int		tmc;	// 时区修正, 所使用本地时的时间与地理位置所在时区的差
}* ptr_param_mntst;

// 定义实数型'点'
typedef struct point_2d {// 两值实数点
	double x;
	double y;

public:
	point_2d() {
		x = y = 0;
	}
	point_2d(double x, double y) {
		this->x = x; this->y = y;
	}
	point_2d& operator=(const point_2d &pt) {
		if (this != &pt) memcpy(this, &pt, sizeof(point_2d));
		return *this;
	}
}POINT2D, * PPOINT2D;

typedef struct point_3d {// 三值实数点
	double x;
	double y;
	double z;

public:
	point_3d() {
		x = y = z = 0;
	}
	point_3d(point_3d &pt) {
		this->x = pt.x;
		this->y = pt.y;
		this->z = pt.z;
	}
	point_3d(double x, double y, double z) {
		this->x = x; this->y = y; this->z = z;
	}
	point_3d& operator=(const point_2d &pt) {
		this->x = pt.x; this->y = pt.y; this->z = 0;
		return *this;
	}
	point_3d& operator=(const point_3d &pt) {
		if (this != &pt) memcpy(this, &pt, sizeof(point_3d));
		return *this;
	}
}POINT3D, * PPOINT3D;

typedef struct hough_point {// Hough数据点
	double r, theta;	//< Hough点极坐标
	double x, y;		//< Hough点XY坐标

public:
	/*!
	 * @brief 直角坐标转换为极坐标
	 */
	void xy2polar() {
		r = sqrt(x * x + y * y);
		theta = atan2(y, x);
	}

	/*!
	 * @brief 极坐标转换为直角坐标
	 */
	void polar2xy() {
		x = r * cos(theta);
		y = r * sin(theta);
	}

	/*!
	 * @brief 由线段计算Hough坐标
	 */
	void line2hough(double x1, double y1, double x2, double y2) {
		theta = atan2(x2 - x1, y1 - y2);
		r = x1 * cos(theta) + y1 * sin(theta);
		polar2xy();
	}

	/*!
	 * @brief 重定义等号
	 */
	hough_point& operator=(const hough_point& other) {
		if (this != &other)
			memcpy(this, &other, sizeof(hough_point));
		return *this;
	}

	/*!
	 * @brief 判断数据点是否在Hough变换确定的直线上
	 * @param x  待定点X坐标
	 * @param y  待定点Y坐标
	 * @param er 极径误差阈值
	 */
	bool online(double x, double y, double er) {
		double dr = x * cos(theta) + y * sin(theta) - r;
		return (-er <= dr && dr <= er);
	}
}HPT, * PHPT;
///////////////////////////////////////////////////////////////////////////////
};
#endif
