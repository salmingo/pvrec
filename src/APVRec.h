/*
 * @file APVRec.h 类APVRec的声明文件
 * APVRec -- 从按时间或帧编号到达的数据流中, 识别提取位置变化源
 * PVRec: Position Variable Source Recognize
 * @version 0.1
 * @date Sep 23, 2016
 * @author Xiaomeng Lu, lxm@nao.cas.cn
 * @version 0.2
 * @date Feb 12, 2019
 *
 * @note
 * 工作流程:
 * (1) NewSequence(),   声明开始新的数据处理流程
 * (3) AddPoint(),      导入数据点
 * (5) GetCandidate(),  查看暂时被识别为目标的详细信息
 * (6) EndSequence(),   声明数据处理流程结束
 * (7) GetNumber(),     查看识别出的目标数量
 * (8) GetObject(),     查看某一目标的详细信息
 *
 * @note
 * 遗留问题(2016年9月26日):
 * (1) 单目标被拆分识别为多个目标(文件)  ==> 合并线段, 要做非线性合并, 暂放弃(Sep 26, 2016)
 * (2) 漏点: 中间某些帧中数据未被正确识别并关联  <== 判据 (待采用时间作为帧间判据, 测试决定后续算法)
 * (3) 跳点: OT1中, 构成目标的数据点, 缺中间的某些帧的数据  <== OT1提取算法  ??
 */

#ifndef APVREC_H_
#define APVREC_H_

#include <string.h>
#include <boost/smart_ptr.hpp>
#include <boost/container/stable_vector.hpp>
#include <boost/container/deque.hpp>
#include "ADefine.h"

namespace AstroUtil {
///////////////////////////////////////////////////////////////////////////////
struct param_pv {// 位置变源关联识别参数
	int    nptmin;	//< 构成PV的最小数据点数量
	double dtmax;	//< 相邻关联数据点的最大时间间隔, 量纲: 天
	double stepmin;	//< 最小步长
	double stepmax;	//< 最大步长
	double dxymax;	//< XY坐标偏差的最大值, 量纲: 像素

public:
	param_pv() {
		nptmin  = 5;
		dtmax   = 60.0 / 86400.0;
		stepmin = 1.0;
		stepmax = 100.0;
		dxymax  = 5.0;
	}
};

typedef struct pv_point {// 单数据点
	int related;	//< 被关联次数
	int fno;		//< 帧编号
	double mjd;		//< 曝光中间时间对应的修正儒略日
	double x, y;	//< 星象质心在模板中的位置
	double ra, dc;	//< 赤道坐标, 量纲: 角度. 坐标系: J2000
	double mag;		//< 星等

public:
	pv_point() {
		memset(this, 0, sizeof(pv_point));
	}

	int inc_rel() {// 增加一次关联次数
		return ++related;
	}

	int dec_rel() {// 减少一次关联次数
		return --related;
	}
}PVPT;
typedef boost::shared_ptr<PVPT> PPVPT;
typedef boost::container::stable_vector<PPVPT> PPVPTVEC;

typedef struct pv_frame {// 单帧数据共性属性及数据点集合
	double mjd;		//< 曝光中间时间对应的修正儒略日
	PPVPTVEC pts;	//< 数据点集合

public:
	pv_frame() {
		mjd = 0.0;
	}

	pv_frame(double Mjd) {
		mjd = Mjd;
	}

	virtual ~pv_frame() {
		pts.clear();
	}
}PVFRM;
typedef boost::shared_ptr<PVFRM> PPVFRM;
typedef boost::container::deque<PPVFRM> PPVFRMDQ;

/*
 * pv_candidate使用流程:
 * 1. 构建对象
 * 2. xy_expect(): 评估输出的xy与数据点之间的偏差是否符合阈值
 * 3. add_point(): 将数据点加入候选体
 * 4. recheck_frame(): 在EndFrame()中评估当前帧数据点是否为候选体提供有效数据
 */
typedef struct pv_candidate {// 候选体
	PPVPTVEC pts;	//< 已确定数据点集合
	PPVPTVEC frmu;	//< 由当前帧加入的不确定数据点
	double vx, vy;	//< XY变化速度
	double lastmjd;	//< 加入候选体的最后一个数据点对应的时间, 量纲: 天; 涵义: 修正儒略日

public:
	PPVPT last_point() {// 构成候选体的最后一个数据点
		return pts[pts.size() - 1];
	}

	bool xy_expect(double mjd, double &x, double &y) {// 由候选体已知(加)速度计算其预测位置
		int n = pts.size();
		if (n >= 2) {
			PPVPT pt = last_point();
			double t = mjd - pt->mjd;
			x = pt->x + vx * t;
			y = pt->y + vy * t;
		}
		return (n >= 2);
	}

	/*!
	 * @brief 将一个数据点加入候选体
	 */
	void add_point(PPVPT pt) {
		if (pts.size() >= 2){// 构成候选体的候选点
			pt->inc_rel();
			frmu.push_back(pt);
		}
		else {// 构成候选体的初始2点
			pts.push_back(pt);
			if (pts.size() == 2) {
				PPVPT prev = pts[0];
				double t = (lastmjd = pt->mjd) - prev->mjd;
				vx = (pt->x - prev->x) / t;
				vy = (pt->y - prev->y) / t;
			}
		}
	}

	PPVPT update() {// 检查/确认来自当前帧的数据点是否加入候选体已确定数据区
		PPVPT pt;
		double x, y;	// 期望位置
		double dx, dy, dx2y2, dx2y2max(1E30);
		if (!frmu.size() || pts.size() < 2) return pt;

		lastmjd = frmu[0]->mjd;
		if (xy_expect(lastmjd, x, y)) {
			/*
			 * 该算法解决: 多点加入一个候选体时带来的混淆
			 */
			for (PPVPTVEC::iterator it = frmu.begin(); it != frmu.end(); ++it) {// 查找与候选体末端最接近的数据
				dx = (*it)->x - x;
				dy = (*it)->y - y;
				dx2y2 = dx * dx + dy * dy;
				if (dx2y2 < dx2y2max) {
					if (pt.use_count()) pt->dec_rel();

					dx2y2max = dx2y2;
					pt = *it;
				}
			}

			if (pt.use_count()) {// 将距离偏差最小的数据点加入候选体
				PPVPT last = last_point();
				double t   = lastmjd - last->mjd;
				vx = (pt->x - last->x) / t;
				vy = (pt->y - last->y) / t;
				pts.push_back(pt);
			}
		}
		frmu.clear();
		return pt;
	}

	virtual ~pv_candidate() {
		pts.clear();
	}
}PVCAN;
typedef boost::shared_ptr<PVCAN> PPVCAN;
typedef boost::container::stable_vector<PPVCAN> PPVCANVEC;

typedef struct pv_object {// PV目标
	PPVPTVEC pts;	//< 已确定数据点集合
}PVOBJ;
typedef boost::shared_ptr<PVOBJ> PPVOBJ;
typedef boost::container::stable_vector<PPVOBJ> PPVOBJVEC;

class APVRec {
public:
	APVRec();
	virtual ~APVRec();

protected:
	param_pv param_;	//< 数据处理参数
	int camid_;			//< 该批次数据使用的相机编号
	int fno_;			//< 最新数据帧编号
	PPVFRM frmprev_;	//< 前一数据帧
	PPVFRM frmlast_;	//< 最新数据帧
	PPVCANVEC cans_;	//< 候选体集合
	PPVOBJVEC objs_;	//< 目标集合

public:
	/*!
	 * @brief 设置数据处理参数
	 */
	void SetParam(param_pv &param);
	/*!
	 * @brief 准备处理一个批次的数据
	 */
	void NewSequence(int camid);
	/*!
	 * @brief 添加一个数据点
	 */
	void AddPoint(PPVPT pt);
	/*!
	 * @brief 结束一个批次数据处理流程
	 */
	void EndSequence();
	/*!
	 * @brief 查看候选体
	 */
	PPVCANVEC& GetCandidate();
	/*!
	 * @brief 查看被识别的目标数量
	 */
	int GetNumber();
	/*!
	 * @brief 查看被识别的目标
	 */
	PPVOBJVEC& GetObject(int &camid);

protected:
	/*!
	 * @brief 准备处理同一帧图像的数据
	 */
	void new_frame(double mjd);
	/*!
	 * @brief 结束同一帧数据
	 */
	void end_frame();
	/*!
	 * @brief 建立新的候选体
	 */
	void create_candidates();
	/*!
	 * @brief 尝试将当前帧数据加入候选体
	 */
	void append_candidates();
	/*!
	 * @brief 检查候选体, 确认其有效性
	 * @note
	 * 判据: 候选体时标与当前帧时标之差是否大于阈值
	 */
	void recheck_candidates();
	/*!
	 * @brief 处理所有候选体
	 */
	void complete_candidates();
	/*!
	 * @brief 将一个候选体转换为目标
	 */
	void candidate2object(PPVCAN can);
};
///////////////////////////////////////////////////////////////////////////////
}

#endif /* APVREC_H_ */
