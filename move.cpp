#include <iostream>
#include "move.h"
#include <string.h>
#include "RRTStar.h"
#include <vector>

#define obstacle_num 15
#define exp_size 20

using namespace std;

int path_len;

//判断两点是否可视
bool is_sight(float** obs, float x1, float y1, float x2, float y2) {
	
	int i;

	if (x1 > x2) {
		float temp;
		temp = x2;
		x2 = x1;
		x1 = temp;
	}

	if (y1 > y2) {
		float temp;
		temp = y2;
		y2 = y1;
		y1 = temp;
	}

	if (x1 == x2) {
		for (i = 0; i < obstacle_num; i++) {
			if (obs[i][1] >= y1 && obs[i][1] <= y2 && obs[i][0] >= (x1 - exp_size) && obs[i][0] <= (x1 + exp_size)) {
				return 0;
				break;
			}
		}
	}

	else if (y1 == y2) {
		for (i = 0; i < obstacle_num; i++) {
			if (obs[i][1] >= (y1- exp_size) && obs[i][1] <= (y1+ exp_size) && obs[i][0] >= x1 && obs[i][0] <= x2) {
				return 0;
				break;
			}
		}
	}

	else {
		float tan = (y2 - y1) / (x2 - x1);
		float dis = sqrt((y2 - y1)*(y2 - y1) + (x2 - x1)*(x2 - x1));
		float sin = (y2 - y1) / dis;
		float cos = (x2 - x1) / dis;
		float x1_n = x1 / cos + (y1 - x1 * tan)*sin;
		float y1_n = y1 / cos - x1_n * tan;
		float x2_n = x2 / cos + (y2 - x2 * tan)*sin;
		float y2_n = y2 / cos - x2_n * tan;

		for (i = 0; i < obstacle_num; i++) {
			float obsx_n = obs[i][0] / cos + (obs[i][1] - obs[i][0] * tan)*sin;
			float obsy_n = obs[i][1] / cos - obsx_n * tan;

			if (obsy_n >= (y1_n - exp_size) && obsy_n <= (y1_n + exp_size) && obsx_n >= x1_n && obsx_n <= x2_n) {
				return 0;
				break;
			}
		}
	}
	return 1;
}

//得到搜索路径，返回路径点二维数组
float **getpath(float bar[][2], float start_x, float start_y, float end_x, float end_y) {
	
	float **m_obs;
	m_obs = new float*[obstacle_num];
	for (int i = 0; i < obstacle_num; i++) {
		m_obs[i] = new float[2];
		m_obs[i][0] = bar[i][0] / 10.0;
		m_obs[i][1] = bar[i][1] / 10.0;
	}

	RRTStar::RRTStar rrt(m_obs, start_x, start_y, end_x, end_y);

	rrt.path_size = rrt.path.size();
	path_len = rrt.path_size;

	float **m_path;
	m_path = new float*[999];
	for (int i = 0; i < path_len; i++) {
		m_path[i] = new float[2];
		m_path[i][0] = rrt.path[i].x;
		m_path[i][1] = rrt.path[i].y;
	}

	return m_path;
}	

//得到路径点数
int pathl() { return path_len; }

//设置路径点数
void set_pathl(int num) { path_len = num; }

//判断得到的路径夹角是否在一定范围内
int is_right_arc(float** c) {
	int i;
	if (path_len <= 2) {
		return 1;
	}
	for (i = 0; i < path_len - 2; i++) {
		double theta1 = atan2(c[i + 1][1] - c[i][1], c[i + 1][0] - c[i][0]);
		double theta2 = atan2(c[i + 2][1] - c[i + 1][1], c[i + 2][0] - c[i + 1][0]);
		if (c[i + 2][1] - c[i + 1][1] < 0) {
			if (3.1415926 - theta1 + theta2 < 1.9) {
				return 0;
			}
		}
		else {
			if (3.1415926 + theta1 - theta2 < 1.9) {
				return 0;
			}
		}
		if ((c[i + 1][0] - c[i][0])*(c[i + 2][0] - c[i + 1][0]) < 0) {
			return 0;
		}
	}
	return 1;
}