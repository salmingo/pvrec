/*
 * @file APVrec.cpp 类APVrec的定义文件
 * @version 0.1
 * @date Sep 23, 2016
 * @version 0.2
 * @date Feb 12, 2019
 */
#include <stdio.h>
#include <boost/make_shared.hpp>
#include "APVRec.h"

namespace AstroUtil {
///////////////////////////////////////////////////////////////////////////////
APVRec::APVRec() {
	camid_ = -1;
	fno_   = -1;
}

APVRec::~APVRec() {
	objs_.clear();
}

void APVRec::SetParam(param_pv &param) {
	memcpy(&param_, &param, sizeof(param_pv));
}

void APVRec::NewSequence(int camid) {
	camid_ = camid;
	fno_   = -1;
	objs_.clear();
	cans_.clear();
	frmprev_.reset();
	frmlast_.reset();
}

void APVRec::AddPoint(PPVPT pt) {
	if (fno_ != pt->fno) {
		if (fno_ != -1) end_frame();
		new_frame(pt->mjd);
		fno_ = pt->fno;
	}
	frmlast_->pts.push_back(pt);
}

void APVRec::EndSequence() {
	if (fno_ != -1) {
		recheck_candidates();	// 检查候选体的有效性
		append_candidates(); 	// 尝试将该帧数据加入候选体
		complete_candidates();	// 将所有候选体转换为目标
	}
	cans_.clear();
	frmprev_.reset();
	frmlast_.reset();
}

PPVCANVEC& APVRec::GetCandidate() {
	return cans_;
}

int APVRec::GetNumber() {
	return objs_.size();
}

PPVOBJVEC& APVRec::GetObject(int &camid) {
	camid = camid_;
	return objs_;
}

void APVRec::new_frame(double mjd) {
	frmprev_ = frmlast_;
	frmlast_ = boost::make_shared<PVFRM>(mjd);
}

void APVRec::end_frame() {
	recheck_candidates();	// 检查候选体的有效性, 释放无效候选体
	append_candidates(); 	// 尝试将该帧数据加入候选体
	create_candidates();	// 为未关联数据建立新的候选体
}

void APVRec::create_candidates() {
	if (!(frmprev_.unique() && frmlast_.unique())) return;

	PPVPTVEC &pts1 = frmprev_->pts;
	PPVPTVEC &pts2 = frmlast_->pts;
	double stepmin = param_.stepmin;
	double stepmax = param_.stepmax;
	double dx, dy;
	// 由相邻帧未关联数据构建候选体
	for (PPVPTVEC::iterator it1 = pts1.begin(); it1 != pts1.end(); ++it1) {
		for (PPVPTVEC::iterator it2 = pts2.begin(); it2 != pts2.end(); ++it2) {
			dx = fabs((*it2)->x - (*it1)->x);
			dy = fabs((*it2)->y - (*it1)->y);

			if (stepmin <= dx && dx <= stepmax && stepmin <= dy && dy <= stepmax) {
				PPVCAN can = boost::make_shared<PVCAN>();
				can->add_point(*it1);
				can->add_point(*it2);
				cans_.push_back(can);
			}
		}
	}
}

void APVRec::append_candidates() {
	if (!cans_.size()) return; // 无候选体立即返回

	double stepmin = param_.stepmin;
	double stepmax = param_.stepmax;
	double dxy  = param_.dxymax;
	double mjd = frmlast_->mjd;
	double x1, y1, x2, y2, dx1, dy1, dx2, dy2;
	PPVPTVEC &pts = frmlast_->pts;
	PPVPT pt;
	PPVCAN can;

	// 1. 尝试将帧数据追加至候选体
	for (PPVCANVEC::iterator it = cans_.begin(); it != cans_.end(); ++it) {
		can = *it;
		// 候选体最后一个点的坐标
		pt = can->last_point();
		x1 = pt->x;
		y1 = pt->y;
		for (PPVPTVEC::iterator i = pts.begin(); i != pts.end(); ++i) {// 与当前帧数据交叉比对
			dx1 = fabs(x1 - (*i)->x);
			dy1 = fabs(y1 - (*i)->y);
			if (stepmin <= dx1 && dx1 <= stepmax && stepmin <= dy1 && dy1 <= stepmax) {// 位置变化步长未超出阈值
				if (can->xy_expect(mjd, x2, y2)) {// 预测位置与测量位置偏差未超出阈值
					dx2 = fabs(x2 - (*i)->x);
					dy2 = fabs(y2 - (*i)->y);
					if (dx2 <= dxy && dy2 <= dxy){
						can->add_point(*i);
					}
				}
				else {
					can->add_point(*i);
				}
			}
		}
	}
	// 2. 将确定帧数据加入候选体
	for (PPVCANVEC::iterator it = cans_.begin(); it != cans_.end(); ++it) {
		pt = (*it)->update();

		if (pt.use_count()) {// 通知数据库, 构成弧段的数据点
			if ((*it)->pts.size() == 3) {// 通知数据库, 构成弧段的前两个数据点

			}
		}
	}
	// 3. 剔除已加入候选体的数据点
	for (PPVPTVEC::iterator it = pts.begin(); it != pts.end(); ) {
		if ((*it)->related) it = pts.erase(it);
		else ++it;
	}
	if (!pts.size()) frmlast_.reset();
}

/*
 * 检查候选体的有效性, 判据: 时标偏差
 * - 大于阈值, 1. 数据点多于阈值, 转换为目标; 2. 数据点小于阈值, 剔除
 * - 小于阈值, 保留
 */
void APVRec::recheck_candidates() {
	if (cans_.size()) {
		int    nptmin = param_.nptmin;
		double dtmax  = param_.dtmax;
		double mjd    = frmlast_->mjd;
		double dt;

		for (PPVCANVEC::iterator it = cans_.begin(); it != cans_.end();) {
			dt = mjd - (*it)->lastmjd;
			if (0 < dt && dt <= dtmax) ++it; // 保留. dt > 0: 原始数据未按时间严格排序
			else {// 移出候选体集合
				if ((*it)->pts.size() >= nptmin) candidate2object(*it); // 转换为目标
				it = cans_.erase(it);
			}
		}
	}
}

void APVRec::complete_candidates() {
	int nptmin = param_.nptmin;

	for (PPVCANVEC::iterator it = cans_.begin(); it != cans_.end(); ++it) {
		if ((*it)->pts.size() >= nptmin) candidate2object(*it);
	}
}

void APVRec::candidate2object(PPVCAN can) {
	PPVPTVEC & pts = can->pts;
	PPVOBJ obj = boost::make_shared<PVOBJ>();
	PPVPTVEC &npts = obj->pts;

	for (PPVPTVEC::iterator it = pts.begin(); it != pts.end(); ++it) {
		npts.push_back(*it);
	}
	objs_.push_back(obj);
}
///////////////////////////////////////////////////////////////////////////////
}
