/*
 * @file APVrec.cpp 类APVrec的定义文件
 * @version 0.1
 * @date Sep 23, 2016
 */

#include "APVRec.h"

namespace AstroUtil {
///////////////////////////////////////////////////////////////////////////////
APVRec::APVRec() {
	m_fno = -1;
	m_time = 0.0;
	m_fi = 0;
	m_pts = m_pts_tail = NULL;
	m_pts_first = NULL;
	m_pts_frame = m_frame_tail = NULL;
	m_cans = m_cans_tail = NULL;
	m_objs = m_objs_tail = NULL;
}

APVRec::~APVRec() {
	if(m_cans) {
		release_link<PPVCANLNK>(&m_cans);
		m_cans_tail = NULL;
	}
	if(m_objs) {
		release_link<PPVOBJLNK>(&m_objs);
		m_objs_tail = NULL;
	}
	if(m_pts) {
		release_point(&m_pts);
		m_pts_tail = NULL;
	}
	if(m_pts_frame) {
		release_point(&m_pts_frame);
		m_frame_tail = NULL;
	}
}

void APVRec::new_sequence() {// 开始新的处理流程
	if(m_objs)
		release_link<PPVOBJLNK>(&m_objs);
	m_objs_tail = m_objs = new PVOBJLNK;
	m_pts_first = NULL;
	if(m_pts_frame)
		release_point(&m_pts_frame);
	m_pts_frame = m_frame_tail = new PVPTLNK;
	if(m_pts)
		release_point(&m_pts);
	m_pts_tail = m_pts = new PVPTLNK;
	if(m_cans)
		release_link<PPVCANLNK>(&m_cans);
	m_cans_tail = m_cans = new PVCANLNK;
	m_fno = -1;
}

void APVRec::new_frame(int fno, double time0) {// 开始导入帧fno数据
	m_fno = fno;
	m_time = time0;
	m_fi = 0;

	/* 调整数据点起始位置 */
	PPVPTLNK now;
	int R = m_param.rframe;
	if (m_pts_first == NULL || (m_fno - m_pts_first->pt->fno) > R) {// 需要调整参与关联的数据点起始位置
		now = m_pts_first != NULL ? m_pts_first->next : m_pts->next;
		while(now != NULL && (m_fno - now->pt->fno) > R)
			now = now->next;

		m_pts_first = now;
	}
}

void APVRec::add_point(PPVPT pt) {// 添加一个原始数据点
	/* 缓存数据点 */
	PPVPTLNK nptlnk = new PVPTLNK;
	PPVPT npt = nptlnk->pt = new PVPT;
	memcpy(npt, pt, sizeof(PVPT));
	npt->fi = ++m_fi;		// 为数据点分配一个编号
	if(m_pts_first != NULL) {
		m_frame_tail->next = nptlnk;
		m_frame_tail = nptlnk;
	}
	else {
		m_pts_tail->next = nptlnk;
		m_pts_tail = nptlnk;
	}
}

void APVRec::end_frame() {// 帧frno数据已经导入完毕
	if (m_fno <= 0 || m_fi == 0 || m_pts_first == NULL)
		return;

	valid_candidate();		// 检查候选体合法性
	corelate_candidate();	// 尝试关联候选体和帧fno的数据点
	related_point();		// 检查数据点已关联性
	construct_candidate();	// 建立候选体, 关联已有数据点和帧fno中未关联数据点

	/* 合并数据点集合 */
	if (m_pts_frame->next != NULL) {
		m_pts_tail->next = m_pts_frame->next;
		m_pts_tail = m_frame_tail;

		m_pts_frame->next = NULL;
		m_frame_tail = m_pts_frame;
	}
}

void APVRec::end_sequence() {// 处理流程结束
	candidate2object(); // 将有效候选体输出为目标

	if(m_cans) {// 释放临时资源
		release_link<PPVCANLNK>(&m_cans);
		m_cans_tail = NULL;
	}
}

void APVRec::set_param(param_pv* param) {// 更改判据
	memcpy(&m_param, param, sizeof(param_pv));
}

param_pv* APVRec::get_param() {// 查看判据
	return &m_param;
}

void APVRec::corelate_candidate() {
	/* 遍历候选体 */
	PPVCANLNK prev = m_cans;
	PPVCANLNK now;

	while((prev = prev->next) != NULL) // 重置关联标记
		prev->corelated = false;

	prev = m_cans;
	while((now = prev->next) != NULL) {
		if (corelate_candidate(now) > 1 && !now->corelated && now != m_cans_tail) {
			now->corelated = true;
			prev->next = now->next;
			now->next = NULL;
			m_cans_tail->next = now;
			m_cans_tail = now;
		}
		else {
			prev = now;
		}
	}
}

int APVRec::corelate_candidate(PPVCANLNK can) {
	/* 选遍历帧数据点, 查找有多少点在该候选体对应的直线上 */
	int npt(0);
	PPVPTLNK ptptr = m_pts_frame;
	PPVPT pt1, pt2, pt;
	PVLN newln;
	PVLN ln;
	double rtheta = m_param.rtheta;
	double rrange = m_param.rrange;
	double dtheta;

	pt1 = can->can.pttail->pt;
	ln = can->can.line;
//	rrange *= ln.len;

	while((ptptr = ptptr->next) != NULL && npt < 2) {
		pt2 = ptptr->pt;
		if (!pt2->related) {
			newln.set_points(pt1, pt2);
			dtheta = newln.theta - ln.theta;
			if (dtheta < -PI180) dtheta += PI360;
			else if (dtheta > PI180) dtheta -= PI360;
			if (dtheta < -rtheta || dtheta > rtheta)
				continue;
			if (fabs(ln.len - newln.len) > rrange)	// 相邻线段长度是否一致
				continue;
			pt = pt2;
			++npt;
		}
	}
//	if (npt > 1) {
//		// 调试输出
//		printf("<%4d : %3d>: candidate has %3d points. %3d points being related\n",
//				pt1->fno, pt1->fi, can->can.npt, npt);
//	}

	/* 将已关联数据点加入候选体 */
	if(npt == 1) {
		can->can.append_point(pt);
	}
	return npt;
}

void APVRec::construct_candidate() {
	PPVPTLNK frmptr = m_pts_frame;
	PPVPTLNK ptshead = m_pts_first;
	PPVPTLNK ptsptr;
	PPVPT pt1, pt2;
	int fno;

	while((frmptr = frmptr->next) != NULL) {// 遍历帧m_fno数据点, 与已有数据点构建候选体
		pt2 = frmptr->pt;
		fno = pt2->fno;
		ptsptr = ptshead;
		while(ptsptr != NULL) {
			pt1 = ptsptr->pt;
			if (pt1->fno < fno && !pt1->related) {
				/* 构建候选体 */
				PPVCANLNK newcan = new PVCANLNK;
				PPVPTLNK pttail = newcan->can.pttail;
				PPVPTLNK ptptr;
				newcan->can.npt = 2;
				newcan->can.pt = *pt2;
				newcan->can.line.set_points(pt1, pt2);
				ptptr = new PVPTLNK;
				ptptr->pt = pt1;
				pttail->next = ptptr;
				pttail = ptptr;
				ptptr = new PVPTLNK;
				ptptr->pt = pt2;
				pttail->next = ptptr;
				newcan->can.pttail = ptptr;
				m_cans_tail->next = newcan;
				m_cans_tail = newcan;
			}
			ptsptr = ptsptr->next;
			/* 调试输出 */
//			printf("%4d, %3d, %6.1f, %6.1f\n", pt1->fno, pt1->fi, pt1->x, pt1->y);
		}
//		printf("%4d, %3d, %6.1f, %6.1f\n\n", pt2->fno, pt2->fi, pt2->x, pt2->y);
	}
}

void APVRec::release_point(PPVPTLNK* head) {// 释放数据点链表
	PPVPTLNK now = *head;
	PPVPTLNK next;

	while(now) {
		next = now->next;
		if (now->pt)
			delete now->pt;
		delete now;
		now = next;
	}
	*head = NULL;
}

void APVRec::related_point() {// 检查帧数据点已关联性
	PPVPTLNK prev = m_pts_frame;
	PPVPTLNK now;

	while((now = prev->next) != NULL) {
		if(now->pt->related) {
			prev->next = now->next;	// 移出帧链表
			now->next = NULL;
			m_pts_tail->next = now;	// 加入全局链表
			m_pts_tail = now;
		}
		else prev = now;
	}
	m_frame_tail = prev;
}

int APVRec::get_number() {
	int n(0);
	PPVOBJLNK obj = m_objs;
	while((obj = obj->next) != NULL) ++n;
	return n;
}

PPVOBJLNK APVRec::get_object() {
	return m_objs;
}

PPVCANLNK APVRec::get_candidate() {
	return m_cans;
}

void APVRec::valid_candidate() {
	double xmin = m_param.xmin;
	double ymin = m_param.ymin;
	double xmax = m_param.xmax;
	double ymax = m_param.ymax;
	int rframe = m_param.rframe;
	double x, y;
	int nfo;
	double dt;
	PPVCANLNK prev = m_cans;
	PPVCANLNK now;
	PPVCAN can;
	// 调试输出
//	int n(0), n1(0), n2(0);

	while((now = prev->next) != NULL) {// 遍历候选体
//		++n;
		can = &now->can;
		nfo = m_fno - can->pt.fno;

		dt = m_time - can->pt.secs;
		x = can->pt.x + can->line.dx * dt;
		y = can->pt.y + can->line.dy * dt;

		if (nfo < rframe && x >= xmin && x <= xmax && y >= ymin && y <= ymax) {// 不符合条件1, 保留候选体
			prev = now;
		}
		else {// 符合条件1, 候选体执行退出
			if (can->npt >= 5) {// 符合条件2, 转换为目标
				candidate2object(can);
//				++n1;
			}
//			else ++n2;
			// 释放候选体资源
			prev->next = now->next;
			now->next = NULL;
			release_link<PPVCANLNK>(&now);
		}
	}
	m_cans_tail = prev;
//	if (n1 > 0 || n2 > 0)
//		printf("<%4d of %4d> candidates are rest. %4d candidates as objects. %4d candidates are erased\n",
//				n - n1 - n2, n, n1, n2);
}

void APVRec::candidate2object() {
	PPVCANLNK now = m_cans;
	PPVCAN can;

	while((now = now->next) != NULL) {
		can = &now->can;
		if (can->npt >= 5) {
			candidate2object(can);
		}
	}
}

void APVRec::candidate2object(PPVCAN can) {
	PPVOBJLNK ptr = new PVOBJLNK;
	ptr->object.npt = can->npt;
	ptr->object.pthead = can->pthead;
	can->pthead = NULL;
	can->pttail = NULL;

	m_objs_tail->next = ptr;
	m_objs_tail = ptr;
}
///////////////////////////////////////////////////////////////////////////////
} /* namespace AstroUtil */
