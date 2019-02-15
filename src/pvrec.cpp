/*============================================================================
 Name        : pvrec.cpp
 Author      : Xiaomeng Lu
 Version     :
 Copyright   : Your copyright notice
 Description : 使用霍夫变换提取位置变化源
 Note        :
 - 使用方法:
   pvrec <parameter> <RAW file / RAW directory> <Result Directory>
   参数列表:
   -F 或缺省: 原始数据格式为文件
   -D      : 原始数据格式为目录, 需遍历处理目录下扩展名为txt的文件
 - 功能:
   关联不同时间的数据点, 从中提取位置变化源

 Bug: 2019-02-13
   (解决)1. 无效星等. mag>20.0, 输出为99.99
 2. 一个轨迹被识别为多个. 原因: (1) 匹配误差阈值; (2) XY应替换为赤经赤纬
   初步确定为：定位误差或时间误差
   (解决)3. 目标目录必须加/作为追加符, 否则无法识别为目录

 Bug: 2019-02-15
 1. 轨迹在靶面非边缘区域突然中断
============================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <strings.h>
#include <sys/time.h>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include "APVRec.h"
#include "ATimeSpace.h"

using std::string;
using namespace AstroUtil;

ATimeSpace ats; // 全局变量, 唯一访问接口

/*
 * @brief 解析存储原始数据的文件中的一行信息
 * 文件行格式为:
 * - 第一行: 注释, 解释每一列的涵义
 * - 第二行至结束, 各列依次为:
 * UTC(精度到秒), 帧编号, X, Y, ra, dec, mag, mag_error, 亚秒(微秒), 天区编号
 */
PPVPT resolve_line(const char* line, int &camid) {
	int iy, im, id, hh, mm, ss, mics;
	double errmag;
	PPVPT pt = boost::make_shared<pv_point>();

	// 格式要求(要求)
	sscanf(line, "%d-%d-%d %d:%d:%d, %d, %lf, %lf, %lf, %lf, %lf, %lf, %d, %d",
			&iy, &im, &id, &hh, &mm, &ss, &pt->fno,
			&pt->x, &pt->y, &pt->ra, &pt->dc,
			&pt->mag, &errmag, &mics, &camid);
	ats.SetUTC(iy, im, id,
			(hh + (mm + (ss + mics * 1E-6 + 5.0) / 60.0) / 60.0) / 24.0);
	pt->mjd = ats.ModifiedJulianDay();
	return pt;
}

void Days2HMS(double fd, int &hh, int &mm, double &ss) {
	hh = (int) fd;
	fd = (fd - hh) * 60.0;
	mm = (int) fd;
	ss = (fd - mm) * 60.0;
}

/*!
 * @brief 输出已关联识别目标
 * @param pvrec  关联识别算法接口
 * @param dirDst 输出数据存储目录
 * @return
 * 导出目标的数量
 */
int OutputObjects(APVRec *pvrec, const char *dirDst) {
	namespace fs = boost::filesystem;
	char filename[50];
	int camid;
	PPVOBJVEC & objs = pvrec->GetObject(camid);
	PPVPT pt;
	int iy, im, id, hh, mm, n(0);
	double ss, fd;
	FILE *fpdst;
	fs::path path;

	for (PPVOBJVEC::iterator it = objs.begin(); it != objs.end(); ++it) {
		PPVOBJ obj = *it;
		PPVPTVEC &pts = obj->pts;

//		// 筛选同步带目标
//		bool is_valid(true);
//		for (PPVPTVEC::iterator i = pts.begin(); i != pts.end() && is_valid; ++i) {
//			is_valid = (*i)->dc > -16.0 && (*i)->dc < 0.0;
//		}
//		if (!is_valid) continue;

		// 生成文件路径
		ats.Mjd2Cal(pts[0]->mjd, iy, im, id, fd);
		sprintf(filename, "%d%02d%02d_%03d_%04d.txt",
				iy, im, id, camid, ++n);
		path = dirDst;
		path /= filename;
		fpdst = fopen(path.c_str(), "w");
		printf(">>>> %s\n", filename);
		// 写入文件内容
		for (PPVPTVEC::iterator i = pts.begin(); i != pts.end(); ++i) {
			pt = *i;
			ats.Mjd2Cal(pt->mjd, iy, im, id, fd);
			Days2HMS(fd * 24.0, hh, mm, ss);
			fprintf(fpdst, "%d %02d %02d %02d %02d %06.3f %4d %9.5f %9.5f ",
					iy, im, id, hh, mm, ss, pt->fno, pt->ra, pt->dc);
			if (pt->mag > 20.0) fprintf(fpdst, "99.99\r\n");
			else fprintf(fpdst, "%5.2f\r\n", pt->mag);
		}

		fclose(fpdst);
	}
	printf("%d objects found\n", n);
	return n;
}

/*
 * @brief 处理一个原始文件
 * @param pathRaw 原始文件路径
 * @param dirDst  结果文件目录
 */
int ProcessFile(const char *pathRaw, const char *dirDst) {
	FILE *fpraw;
	char line[200];
	int objcnt(0), newid(-1), oldid(-1);
	APVRec pvrec;

	if ((fpraw = fopen(pathRaw, "r")) == NULL) {// 打开原始文件
		printf("failed to open file: %s\n", pathRaw);
		return -1;
	}

	fgets(line, 200, fpraw); // 空读一行
	while (!feof(fpraw)) {// 遍历原始数据文件
		if (fgets(line, 200, fpraw) == NULL) continue;
		PPVPT pt = resolve_line(line, newid);

		if (oldid != newid) {
			if (oldid != -1) {
				pvrec.EndSequence();
				objcnt += OutputObjects(&pvrec, dirDst); // 导出关联识别数据
			}
			oldid = newid;
			pvrec.NewSequence(newid);
		}

		pvrec.AddPoint(pt);
	}
	fclose(fpraw); // 关闭原始文件
	// 最好一行原始数据的特殊处理
	pvrec.EndSequence();
	objcnt += OutputObjects(&pvrec, dirDst); // 导出关联识别数据

	return objcnt;
}

/*
 * @brief 处理一个原始文件目录
 * @param dirRaw  原始文件目录
 * @param dirDst  结果文件目录
 */
int ProcessDirectory(const char *dirRaw, const char *dirDst) {
	namespace fs = boost::filesystem;

	int objcnt(0), n;
	fs::path path = dirRaw;
	fs::directory_iterator itend = fs::directory_iterator();
	string extdef = ".txt", extname;
	for (fs::directory_iterator x = fs::directory_iterator(path); x != itend; ++x) {
		extname = x->path().filename().extension().string();
		if (extname == extdef) {
			printf("**** %s ****\n", x->path().filename().c_str());
			n = ProcessFile(x->path().c_str(), dirDst);
			if (n > 0) objcnt += n;
		}
	}

	return objcnt;
}

int main(int argc, char** argv) {
	if (argc < 3 || argc > 4) {
		printf("Usgae: pvrec <param> <path name of raw file> <directory name of result>\n");
		return -1;
	}
	// 解析命令行参数
	string paths[2];
	int pos(0), type(0); // type: 0, File; 1: Directory
	for (int i = 1; i < argc; ++i) {
		if (argv[i][0] == '-') {
			if (strcasecmp(argv[i], "-D") == 0) type = 1;
			else if (strcasecmp(argv[i], "-F") == 0) type = 0;
			else {
				printf("undefined parameter\n");
				return -2;
			}
		}
		else if (pos < 2) {
			paths[pos] = argv[i];
			++pos;
		}
		else {
			printf("too much directories given\n");
			return -3;
		}
	}

	// 检查原始数据是否有效
	namespace fs = boost::filesystem;
	fs::path path = paths[0];
	if (type == 0 && !fs::is_regular_file(path)) {
		printf("RAW file requires file path\n");
		return -4;
	}
	else if (type == 1 && !fs::is_directory(path)) {
		printf("RAW file requires directory path\n");
		return -5;
	}

	fs::path pathdst = paths[1];
	if (!fs::is_directory(pathdst) && !fs::create_directories(pathdst)) {
		printf("failed to create directory for result files\n");
		return -6;
	}

	int n;
	if (type == 0) n = ProcessFile(paths[0].c_str(), paths[1].c_str());
	else n = ProcessDirectory(paths[0].c_str(), paths[1].c_str());
	printf("%d totally being correlated\n", n);
	printf("---------- Over ----------\n");

	return 0;
}
