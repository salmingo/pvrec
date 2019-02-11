/*
 * @file APVRec.h 类APVRec的声明文件
 * APVRec -- 从按时间或帧编号到达的数据流中, 识别提取位置变化源
 * PVRec: Position Variable Source Recognize
 * @version 0.1
 * @date Sep 23, 2016
 * @author Xiaomeng Lu, lxm@nao.cas.cn
 *
 * @note
 * 工作流程:
 * (1) new_sequence(), 声明开始新的数据处理流程
 * (2) new_frame(), 声明开始处理新的一帧图像数据
 * (3) add_point(), 导入帧数据
 * (4) end_frame(), 声明一帧数据导入完毕
 * (5) get_candidate(), 查看暂时被识别为目标的详细信息
 * (6) end_sequence(), 声明数据处理流程结束
 * (7) get_number(), 查看识别出的目标数量
 * (8) get_object(), 查看某一目标的详细信息
 *
 * @note
 * 遗留问题(2016年9月26日):
 * (1) 单目标被拆分识别为多个目标(文件)  ==> 合并线段, 要做非线性合并, 暂放弃(Sep 26, 2016)
 * (2) 漏点: 中间某些帧中数据未被正确识别并关联  <== 判据 (待采用时间作为帧间判据, 测试决定后续算法)
 * (3) 跳点: OT1中, 构成目标的数据点, 缺中间的某些帧的数据  <== OT1提取算法  ??
 */

#ifndef APVREC_H_
#define APVREC_H_

#include "ADefine.h"
#include "AMath.h"

namespace AstroUtil {
///////////////////////////////////////////////////////////////////////////////
struct param_pv {// PV识别提取判据
	int rframe;			//< 最大帧跨度, 距离越远的点关联性越差, 一般取5. 对于暗弱目标频繁出现跳点, 需要适当增大该值
	double rtheta;		//< 斜率相等判据
	double rrange;		//< 线段长度偏差阈值, 相对: |(r2-r1)|/r2<rrange
	double xmin, ymin;	//< 预测位置, 最小XY坐标, 用于候选体退出机制. 取(-50, -50)
	double xmax, ymax;	//< 预测位置, 最大XY坐标. 取图像宽高加50(w+50,h+50)

public:
	param_pv() {
		rframe = 5;
		rtheta = 2.0 * GtoR;
//		rrange = 0.02;
		rrange = 1.0;
		xmin = ymin = 1.0;
		xmax = ymax = 4000.0;
	}
};

typedef struct pv_point {// 单个目标数据点
	bool related;				//< 关联标志
	int fno;					//< 数据点所在帧的编号
	int fi;						//< 数据点在帧中的编号, 有效范围[1, ∞)
	int year, month, day;		//< 年月日, UTC时间
	double secs;				//< 以当日0时为零点的秒数
	double x, y;				//< 星像质心相对于图像原点的XY坐标
	double ra, dec;				//< 星像质心对应的赤道坐标, 量纲: 角度. 坐标系: 当前
	double mag;					//< 亮度, 量纲: 星等
	int camid;					//< 相机编号

public:
	pv_point& operator=(const pv_point& other) {
		if (this != &other)
			memcpy(this, &other, sizeof(pv_point));
		return *this;
	}
}PVPT, * PPVPT;
typedef std::vector<PPVPT> VECPPVPT;

typedef struct pv_point_link {// 原始数据点链表
	PPVPT pt;				// 原始数据点
	pv_point_link* next;	// 后续点

public:
	pv_point_link() {
		pt   = NULL;
		next = NULL;
	}
}PVPTLNK, * PPVPTLNK;

typedef struct pv_line {// 线段特征
	// 使用投影坐标关联轨迹
	double dx, dy;		//< 线段在XY方向上的位移量
	double theta;		//< 相对X轴倾角
	double len;			//< 线段长度的平方

public:
	void set_points(PPVPT pt1, PPVPT pt2) {
		double dt = pt2->secs - pt1->secs;
		dx = (pt2->x - pt1->x) / dt;
		dy = (pt2->y - pt1->y) / dt;
		len = sqrt(dx * dx + dy * dy);
		theta = atan2(dy, dx);
	}

	pv_line& operator=(const pv_line& other) {
		if (this != &other)
			memcpy(this, &other, sizeof(pv_line));
		return *this;
	}

	void combine(const pv_line* other) {
		dx    = (dx + other->dx) * 0.5;
		dy    = (dy + other->dy) * 0.5;
		len   = (len + other->len) * 0.5;
		theta = (theta + other->theta) * 0.5;
	}
}PVLN, * PPVLN;

typedef struct pv_candidate {// 候选体
	int npt;				//< 构成候选体的点数
	PVPT pt;				//< 候选体的最后一个点
	PVLN line;				//< 构成候选体最后一个线段的特征
	PPVPTLNK pthead;		//< 构成候选体的点的链表
	PPVPTLNK pttail;

public:
	pv_candidate() {
		npt = 0;
		pthead = new PVPTLNK;
		pttail = pthead;
	}

	virtual ~pv_candidate() {
		PPVPTLNK now = pthead;
		PPVPTLNK nxt;
		while(now != NULL) {
			nxt = now->next;
			delete now;
			now = nxt;
		}
		pthead = NULL;
		pttail = NULL;
	}

	/*!
	 * @brief 在候选体末尾添加一个数据点
	 */
	void append_point(PPVPT newpt) {
//		line.set_points(&pt, newpt);
		PVLN nln;
		nln.set_points(&pt, newpt);
		line.combine(&nln);
		pt = *newpt;
		newpt->related = true;
		++npt;

		PPVPTLNK ptr = new PVPTLNK;
		ptr->pt = newpt;
		pttail->next = ptr;
		pttail = ptr;
	}
}PVCAN, * PPVCAN;

typedef struct pv_candidate_link {// 候选体链表
	bool corelated;		//< 本轮是否已经作过关联
	PVCAN can;
	pv_candidate_link* next;

public:
	pv_candidate_link() {
		corelated = false;
		next = NULL;
	}
}PVCANLNK, * PPVCANLNK;

typedef struct pv_object {// 位置变化源
	int npt;			//< 构成目标的数据点数
	PPVPTLNK pthead;

public:
	pv_object() {
		npt = 0;
		pthead = NULL;
	}

	virtual ~pv_object() {
		PPVPTLNK now = pthead;
		PPVPTLNK nxt;
		while(now) {
			nxt = now->next;
			delete now;
			now = nxt;
		}
		pthead = NULL;
	}
}PVOBJ, * PPVOBJ;

typedef struct pv_object_link {// 位置变化源链表
	PVOBJ object;
	pv_object_link* next;

public:
	pv_object_link() {
		next = NULL;
	}
}PVOBJLNK, * PPVOBJLNK;

class APVRec {
public:
	APVRec();
	virtual ~APVRec();

public:
	/*!
	 * @brief 开始处理新的数据流
	 */
	void new_sequence();
	/*!
	 * @brief 开始导入帧编号为fno的数据
	 */
	void new_frame(int fno, double time0);
	/*!
	 * @brief 添加一个原始数据点
	 */
	void add_point(PPVPT pt);
	/*!
	 * @brief 帧编号为fno的数据已经全部导入
	 */
	void end_frame();
	/*!
	 * @brief 结束处理流程
	 */
	void end_sequence();
	/*!
	 * @brief 更改判据
	 */
	void set_param(param_pv* param);
	/*!
	 * @brief 查看判据
	 */
	param_pv* get_param();
	/*!
	 * @brief 查看提取的目标数量
	 */
	int get_number();
	/*!
	 * @brief 查看已提取目标的详细信息
	 */
	PPVOBJLNK get_object();
	/*!
	 * @brief 查看已提取候选体的详细信息
	 */
	PPVCANLNK get_candidate();

protected:
	/*!
	 * @brief 尝试关联数据点和候选体
	 */
	void corelate_candidate();
	/*!
	 * @brief 尝试关联单个候选体与所有数据点
	 */
	int corelate_candidate(PPVCANLNK can);
	/*!
	 * @brief 创建候选体
	 */
	void construct_candidate();
	/*!
	 * @brief 释放链表资源
	 */
	template<typename T> void release_link(T* head);
	/*!
	 * @brief 释放数据点链表
	 * @note
	 * 对数据点链表做特殊处理
	 */
	void release_point(PPVPTLNK* head);
	/*!
	 * @brief 检查帧数据点是否已经与某(些)候选体关联, 若已关联则移出帧链表
	 */
	void related_point();
	/*!
	 * @brief 候选体退出机制. 将符合要求的候选体输出为目标
	 * @note
	 * 候选体输出条件:
	 * (1) 按照运动趋势, 当前帧位置预测越界
	 * (2) 构成候选体的点数不少于5, 即npt>=5
	 */
	void valid_candidate();
	/*!
	 * @brief 将候选体转换为目标
	 */
	void candidate2object();
	void candidate2object(PPVCAN can);

private:
	param_pv m_param;			//< 判据
	int m_fno;					//< 新添加数据所归属的帧编号
	double m_time;				//< 新添加数据对应的天内秒数
	int m_fi;					//< 帧中数据点编号
	PPVPTLNK m_pts;				//< 数据点集合
	PPVPTLNK m_pts_tail;
	PPVPTLNK m_pts_first;		//< 参与后续关联的数据点首地址
	PPVPTLNK m_pts_frame;		//< 单帧图像中的数据点
	PPVPTLNK m_frame_tail;
	PPVCANLNK m_cans;			//< 候选体集合
	PPVCANLNK m_cans_tail;
	PPVOBJLNK m_objs;			//< 目标集合
	PPVOBJLNK m_objs_tail;
};

template<typename T> void APVRec::release_link(T* head) {
	T now = *head;
	T next;

	while(now) {
		next = now->next;
		delete now;
		now = next;
	}
	*head = NULL;
}
///////////////////////////////////////////////////////////////////////////////
} /* namespace AstroUtil */

#endif /* APVREC_H_ */
