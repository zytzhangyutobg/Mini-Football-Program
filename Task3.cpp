#include <iomanip> 
#include <iostream>
#include "Ws2tcpip.h"
#include "Winsock2.h"
#pragma comment(lib,"WS2_32.lib")

#include"zss_cmd.pb.h"
#include"vision_detection.pb.h"
#include"zss_debug.pb.h"
#include "move.h"
#include<string.h>
#include<math.h>

using std::cin;
using std::cout;
using std::cerr;
using std::endl;

int sgn(float d) { return d < 0 ? -1 : 1; }

int main() {
	int control = 4;
	int control_num;

	float finish_x = -250;
	float finish_y = 150;
	float fx = finish_x;
	float fy = finish_y;

	float bar[15][2];

	//设置命令信息
	Robots_Command robots_command;
	Robot_Command *robot_command;
	robot_command = robots_command.add_command();

	robot_command->set_velocity_x(0);
	robot_command->set_velocity_y(0);
	robot_command->set_velocity_r(0);
	robot_command->set_kick(false);
	robot_command->set_power(0);
	robot_command->set_dribbler_spin(10);

	int SOCKADDR_IN_SIZE = sizeof(SOCKADDR_IN);
	int vis_port = 23333;
	const size_t vis_size = 2048;
	const size_t vis_ip_size = 256;

	int cmd_port = 50001;
	char cmd_buf[1024];
	int cmd_size = 1024;

	char debug_buf[1024 * 10];
	int debug_size = 1024 * 10;
	int Send_port = 20001;

	char vis_ip_buf[vis_ip_size];
	char vis_buf[vis_size];
	int vis_val = 0;

	//UDP设置
	WSADATA wsa_data;

	SOCKADDR_IN addr_cmd, addr_vis, addr_clt, addr_debug;
	SOCKET sock_cmd, sock_vis = INVALID_SOCKET, sock_debug;

	WSAStartup(MAKEWORD(2, 2), &wsa_data);

	sock_cmd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	addr_cmd.sin_family = AF_INET;
	addr_cmd.sin_port = htons(cmd_port);
	addr_cmd.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	std::bind(addr_cmd, (SOCKADDR *)&addr_cmd, sizeof(addr_cmd));

	addr_vis.sin_family = AF_INET;
	addr_vis.sin_port = htons(vis_port);
	addr_vis.sin_addr.S_un.S_addr = ADDR_ANY;
	sock_vis = socket(addr_vis.sin_family, SOCK_DGRAM, IPPROTO_UDP);
	vis_val = bind(sock_vis, (SOCKADDR*)&addr_vis, SOCKADDR_IN_SIZE);

	sock_debug = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	addr_debug.sin_family = AF_INET;
	addr_debug.sin_port = htons(Send_port);
	addr_debug.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	std::bind(addr_debug, (SOCKADDR *)&addr_debug, sizeof(addr_debug));

	SecureZeroMemory(vis_buf, vis_size);
	vis_val = recvfrom(sock_vis, vis_buf, vis_size, 0, (SOCKADDR*)&addr_clt, &SOCKADDR_IN_SIZE);
	inet_ntop(addr_clt.sin_family, &addr_clt, vis_ip_buf, vis_ip_size);
	Vision_DetectionFrame current;
	current.ParseFromArray(vis_buf, 1024 * 10);

	int blue_size = current.robots_blue_size();
	int yellow_size = current.robots_yellow_size();

	//读取初始障碍物
	int bar_i = 0;
	for (int blue_i = 0; blue_i < blue_size; blue_i++)
	{
		if (current.robots_blue(blue_i).robot_id() == control) {
			control_num = blue_i;
			robot_command->set_robot_id(control_num);
			continue;
		}
		else
		{
			bar[bar_i][0] = current.robots_blue(blue_i).x();
			bar[bar_i][1] = current.robots_blue(blue_i).y();
			bar_i++;
		}
	}
	for (int yellow_i = 0; yellow_i < yellow_size; yellow_i++)
	{

		bar[bar_i][0] = current.robots_yellow(yellow_i).x();
		bar[bar_i][1] = current.robots_yellow(yellow_i).y();
		bar_i++;

	}

	float x = current.robots_blue(control_num).x() / 10;
	float y = current.robots_blue(control_num).y() / 10;

	float previous_x = x;
	float previous_y = y;

	int len;
	float** cmd_s;

	float vel = 75.0;
	float vel2 = 150;
	float vel3 = 0;

	int k = 1;
	int toward = 1;
	int calu = 0;
	int boom = 0;
	int get = 0;
	bool flag;
	int flag_num = 0;
	int flag_after = 0;

	int goal[7][2] = { {-250,150}, {-220,-80},{220,80},{250,-150},{220,80}, {-220,-80},{-250,150} };

	//循环到达每一个目标点
	while (1)
	{
		//随机采样空白区域
		goal[1][0] = rand() % 160 + 1 - 250;
		goal[2][0] = rand() % 160 + 1 + 100;
		goal[4][0] = rand() % 160 + 1 + 100;
		goal[5][0] = rand() % 160 + 1 - 260;
		goal[1][1] = rand() % 120 + 1 - 170;
		goal[2][1] = rand() % 120 + 1 + 50;
		goal[4][1] = rand() % 120 + 1 + 50;
		goal[5][1] = rand() % 120 + 1 - 170;

		robot_command->set_velocity_x(0);
		robot_command->set_velocity_y(0);
		robot_command->set_velocity_r(0);
		robots_command.SerializeToArray(cmd_buf, cmd_size);
		sendto(sock_cmd, cmd_buf, cmd_size, 0, (SOCKADDR *)&addr_cmd, sizeof(addr_cmd));

		//判断目标点取点	
		if (get == 1) {
			fx = goal[k][0];
			fy = goal[k][1];
			k++;
			if (k == 8) {
				k = 1;
				fx = goal[k][0];
				fy = goal[k][1];
				k++;
			}
			get = 0;
		}

		boom = 0;

		//接收数据
		SecureZeroMemory(vis_buf, vis_size);
		vis_val = recvfrom(sock_vis, vis_buf, vis_size, 0, (SOCKADDR*)&addr_clt, &SOCKADDR_IN_SIZE);
		inet_ntop(addr_clt.sin_family, &addr_clt, vis_ip_buf, vis_ip_size);
		Vision_DetectionFrame current;
		current.ParseFromArray(vis_buf, 1024 * 10);
		blue_size = current.robots_blue_size();
		yellow_size = current.robots_yellow_size();

		//读取实时障碍物数据
		bar_i = 0;
		for (int blue_i = 0; blue_i < blue_size; blue_i++)
		{
			if (current.robots_blue(blue_i).robot_id() == control) {
				control_num = blue_i;
				robot_command->set_robot_id(control_num);
				continue;
			}
			else
			{
				bar[bar_i][0] = current.robots_blue(blue_i).x();
				bar[bar_i][1] = current.robots_blue(blue_i).y();
				bar_i++;
			}
		}
		for (int yellow_i = 0; yellow_i < yellow_size; yellow_i++)
		{
			bar[bar_i][0] = current.robots_yellow(yellow_i).x();
			bar[bar_i][1] = current.robots_yellow(yellow_i).y();
			bar_i++;
		}

		float x = current.robots_blue(control_num).x() / 10;
		float y = current.robots_blue(control_num).y() / 10;

		//RRT规划路径
		cmd_s = getpath(bar, x, y, fx, fy);

		len = pathl() - 1;

		//画出轨迹
		Debug_Msgs debugs;
		for (int debug_i = 0; debug_i < len; debug_i++)
		{

			Debug_Msg *debug = debugs.add_msgs();
			debug->set_type(Debug_Msg::LINE);
			debug->set_color(Debug_Msg::RED);
			Debug_Line *line1 = new Debug_Line;

			Point *startpoint = new Point;

			Point *endpoint = new Point;

			startpoint->set_x(cmd_s[debug_i][0]);
			startpoint->set_y(cmd_s[debug_i][1]);
			endpoint->set_x(cmd_s[debug_i + 1][0]);
			endpoint->set_y(cmd_s[debug_i + 1][1]);
			line1->set_forward(true);
			line1->set_back(false);

			line1->set_allocated_start(startpoint);
			line1->set_allocated_end(endpoint);

			debug->set_allocated_line(line1);

			debugs.SerializeToArray(debug_buf, debug_size);
			sendto(sock_debug, debug_buf, debug_size, 0, (SOCKADDR *)&addr_debug, sizeof(addr_debug));
		}

		float max_vel = 280;
		float mid_vel = 180;
		float mids_vel = 200;
		float min_vel = 25;

		float max_dis = 3600;
		float mid_dis = 1200;
		
		//循环按照path点集行走
		for (int i = 0; i < len; i++)
		{
			//循环从一个点走到另一个点
			while (true)
			{
				//接受实时数据
				SecureZeroMemory(vis_buf, vis_size);
				vis_val = recvfrom(sock_vis, vis_buf, vis_size, 0, (SOCKADDR*)&addr_clt, &SOCKADDR_IN_SIZE);
				inet_ntop(addr_clt.sin_family, &addr_clt, vis_ip_buf, vis_ip_size);
				Vision_DetectionFrame current;
				current.ParseFromArray(vis_buf, 1024 * 10);
				
				float x = current.robots_blue(control_num).x() / 10;
				float y = current.robots_blue(control_num).y() / 10;
				float ori = current.robots_blue(control_num).orientation();

				yellow_size = current.robots_yellow_size();

				bar_i = 0;
				for (int yellow_i = 0; yellow_i < yellow_size; yellow_i++)
				{

					bar[bar_i][0] = current.robots_yellow(yellow_i).x();
					bar[bar_i][1] = current.robots_yellow(yellow_i).y();
					bar_i++;
				}

				float next_x = cmd_s[i + 1][0];
				float next_y = cmd_s[i + 1][1];
				float prev_x = cmd_s[i][0];
				float prev_y = cmd_s[i][1];

				float x_vel_;
				float y_vel_;
				float x_vel;
				float y_vel;

				//到达下一个点跳出循环
				if ((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y) < 80.0)
					break;

				//运动控制
				if (i == len - 1)
				{
					if ((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y) > 4000.0)
					{
						x_vel_ = max_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
						y_vel_ = max_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
					}
					else
					{
						x_vel_ = mid_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
						y_vel_ = mid_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
					}
				}
				else
				{
					float nnext_x = cmd_s[i + 2][0];
					float nnext_y = cmd_s[i + 2][1];
					if (3.14159 - abs(atan2(nnext_y - next_y, nnext_x - next_x) - atan2(next_y - prev_y, next_x - prev_x)) < 3.14159*0.8)
					{
						if ((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y) < 3600.0)
						{
							x_vel_ = mid_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
							y_vel_ = mid_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
						}
						else
						{
							x_vel_ = max_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
							y_vel_ = max_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
						}
					}
					else if (3.14159 - abs(atan2(nnext_y - next_y, nnext_x - next_x) - atan2(next_y - prev_y, next_x - prev_x)) < 3.14159*0.9)
					{
						if ((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y) < 3600.0)
						{
							x_vel_ = mids_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
							y_vel_ = mids_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
						}
						else
						{
							x_vel_ = max_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
							y_vel_ = max_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
						}
					}
					else
					{
						x_vel_ = max_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
						y_vel_ = max_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
					}
				}

				if ((prev_x - x)*(prev_x - x) + (prev_y - y)*(prev_y - y) < 100)
				{
					x_vel_ = mid_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
					y_vel_ = mid_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
				}

				else if ((prev_x - x)*(prev_x - x) + (prev_y - y)*(prev_y - y) < 400)
				{
					x_vel_ = mids_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
					y_vel_ = mids_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
				}

				float x1 = prev_x;
				float y1 = prev_y;
				float x2 = next_x;
				float y2 = next_y;

				//轨迹负反馈
				float k = (y1 - y2) / ((x1 - x2) + 0.0001);
				float theta = atan(k);
				float dis = sgn(fy)*((y1 - y2)*x - (x1 - x2)*y + x1 * y2 - x2 * y1) / sqrt((y1 - y2)*(y1 - y2) + (x1 - x2)*(x1 - x2))*sgn(-k);

				if (abs(dis) >= 30) {
					dis = sgn(dis) * 30;
				}

				float pro = 0.09;//参数

				float ame_x = -pro * dis*dis*dis * sin(theta);
				float ame_y = +pro * dis*dis*dis * cos(theta);

				if (fy*(x2 - x1) < 0 && fy*(y2 - y1) < 0)
				{
					ame_x = -ame_x;
					ame_y = -ame_y;
				}

				if (fy*(x2 - x1) > 0 && fy*(y2 - y1) < 0)
				{
					ame_x = -ame_x;
					ame_y = -ame_y;
				}

				x_vel_ = x_vel_ + ame_x;
				y_vel_ = y_vel_ + ame_y;

				float plus_x = 0;
				float plus_y = 0;

				//人工势场避障
				float bilixishu = 9000;

				for (int bari = 0; bari < bar_i; bari++)
				{
					if ((bar[bari][0] / 10 - x)*(bar[bari][0] / 10 - x) + (bar[bari][1] / 10 - y)*(bar[bari][1] / 10 - y) <= 4000)
					{
						float theta = atan2(y - bar[bari][1] / 10, x - bar[bari][0] / 10);
						float distance = sqrt((bar[bari][0] / 10 - x)*(bar[bari][0] / 10 - x) + (bar[bari][1] / 10 - y)*(bar[bari][1] / 10 - y));

						plus_y += bilixishu / (distance)* sin(theta);
						plus_x += bilixishu / (distance)* cos(theta);
					}
				}

				//陷入局部最小点时消除势场
				if ((x - previous_x)*(x - previous_x) + (y - previous_y)*(y - previous_y) <= 50) {
					flag_num++;
					if (flag_num >= 25) {
						flag = true;
						plus_x = 0;
						plus_y = 0;

					}
					else
						flag = false;
				}
				else
				{
					flag_num = 0;
					previous_x = x;
					previous_y = y;
				}

				x_vel_ += plus_x;
				y_vel_ += plus_y;

				//墙的势场，防止出界
				if (abs(x) >= 80656) {
					if (x > 0)
						x_vel_ -= 300;
					if (x < 0)
						x_vel_ += 300;
				}
				if (abs(y) >= 43681) {
					if (y > 0)
						y_vel_ -= 300;
					if (y < 0)
						y_vel_ += 300;
				}

				//陷入局部最小点跳出速度
				if (flag == true) {
					flag_after = 70;
				}

				if (flag_after != 0) {
					if (y > -0.6*x) {
						x_vel_ = x_vel_ - 150;
						y_vel_ = y_vel_ - 150;
						flag_after--;
					}
					else {
						x_vel_ = x_vel_ + 150;
						y_vel_ = y_vel_ + 150;
						flag_after--;
					}

				}

				x_vel = x_vel_ * cos(current.robots_blue(control_num).orientation()) + y_vel_ * sin(current.robots_blue(control_num).orientation());
				y_vel = y_vel_ * cos(current.robots_blue(control_num).orientation()) - x_vel_ * sin(current.robots_blue(control_num).orientation());

				//发送速度数据
				robot_command->set_velocity_x(x_vel);
				robot_command->set_velocity_y(-y_vel);
				robot_command->set_velocity_r(0);
				robots_command.SerializeToArray(cmd_buf, cmd_size);
				sendto(sock_cmd, cmd_buf, cmd_size, 0, (SOCKADDR *)&addr_cmd, sizeof(addr_cmd));
				
				calu++;
				
				//定时重新规划路径
				if (calu == 15) {
					boom = 1;
					calu = 0;
					break;
				}
			}

			if (boom == 1) {
				break;
			}
		}

		//抵达中间点/终点（判断条件不同，到达终点的要求更加严格）
		if (k == 2 || k == 3 || k == 5 || k == 6) {
			if (sqrt((x - fx)*(x - fx) + (y - fy)*(y - fy)) < 20) {
				get = 1;
			}
		}
		else {
			if (sqrt((x - fx)*(x - fx) + (y - fy)*(y - fy)) < 7) {
				get = 1;
			}
		}
	}

	//结束UDP发送
	closesocket(sock_debug);
	closesocket(sock_cmd);
	closesocket(sock_vis);
	WSACleanup();
	return 0;
}