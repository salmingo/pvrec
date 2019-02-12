/*
 * @file APVrec.cpp 类APVrec的定义文件
 * @version 0.1
 * @date Sep 23, 2016
 * @version 0.2
 * @date Feb 12, 2019
 */
#include <boost/make_shared.hpp>
#include "APVRec.h"

namespace AstroUtil {
///////////////////////////////////////////////////////////////////////////////
APVRec::APVRec() {
	camid_ = -1;
}

APVRec::~APVRec() {
}

void APVRec::SetParam(param_pv &param) {
	memcpy(&param_, &param, sizeof(param_pv));
}

void APVRec::NewSequence(int camid) {
	camid_ = camid;
	frms_.clear();
	frmlast_.reset();
	cans_.clear();
	objs_.clear();
}

void APVRec::NewFrame(int fno, double mjd) {
	frmlast_ = boost::make_shared<PVFRM>(fno, mjd);
	frms_.push_back(frmlast_);
}

void APVRec::AddPoint(PPVPT pt) {
	frmlast_->pts.push_back(pt);
}

void APVRec::EndFrame() {
	// 建立关联
	// 释放无效帧
	// 释放无效候选体/有效候选体转为目标
}

void APVRec::EndSequence() {
	// 释放所有帧
	// 有效候选体转为目标/释放无效候选体
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
///////////////////////////////////////////////////////////////////////////////
}
