#include <iomanip> 
#include <iostream>
#include "Ws2tcpip.h"
#include "Winsock2.h"
#pragma comment(lib,"WS2_32.lib")
#pragma comment(lib,"C:/Xiaoxueqi/lib/gmock.lib")
#pragma comment(lib,"C:/Xiaoxueqi/lib/gmock_main.lib")
#pragma comment(lib,"C:/Xiaoxueqi/lib/libprotobuf.lib")
#pragma comment(lib,"C:/Xiaoxueqi/lib/libprotobuf-lite.lib")
#pragma comment(lib,"C:/Xiaoxueqi/lib/libprotoc.lib")

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

#include <time.h> 
#include <stdio.h> 
#include <tchar.h> 
#include <sys/timeb.h> 
#include<Windows.h>

int sgn(float d) { return d < 0 ? -1 : 1; }

int main() {
	int control = 4;
	int control_num;//设置需控制机器人ID

	Robots_Command robots_command;
	Robot_Command *robot_command;
	robot_command = robots_command.add_command();

	robot_command->set_velocity_x(0);
	robot_command->set_velocity_y(0);
	robot_command->set_velocity_r(0);
	robot_command->set_kick(false);
	robot_command->set_power(0);
	robot_command->set_dribbler_spin(10);//对机器人指令进行初始化

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
	std::bind(addr_debug, (SOCKADDR *)&addr_debug, sizeof(addr_debug));//建立UDP通讯连接

	float finish_x = 250;
	float finish_y = -150;
	float fx = finish_x;
	float fy = finish_y;//设置目标点

	SecureZeroMemory(vis_buf, vis_size);
	vis_val = recvfrom(sock_vis, vis_buf, vis_size, 0, (SOCKADDR*)&addr_clt, &SOCKADDR_IN_SIZE);
	inet_ntop(addr_clt.sin_family, &addr_clt, vis_ip_buf, vis_ip_size);
	Vision_DetectionFrame current;
	current.ParseFromArray(vis_buf, 1024 * 10);//接收UDP数据

	int blue_size = current.robots_blue_size();
	int yellow_size = current.robots_yellow_size();
	float bar[15][2];
	int bar_i = 0;
	
	for (int blue_i = 0; blue_i < blue_size; blue_i++)
	{
		if (current.robots_blue(blue_i).robot_id() == control) {
			control_num = blue_i;
			robot_command->set_robot_id(control);
			continue;
		}
		else
		{
			bar[bar_i][0] = current.robots_blue(blue_i).x();
			bar[bar_i][1] = current.robots_blue(blue_i).y();
			bar_i++;
		}
	}//判断控制蓝车ID

	for (int yellow_i = 0; yellow_i < yellow_size; yellow_i++)
	{
		bar[bar_i][0] = current.robots_yellow(yellow_i).x();
		bar[bar_i][1] = current.robots_yellow(yellow_i).y();
		bar_i++;
	}
	for (int ni = 0; ni < bar_i; ni++)
	{
		cout << bar[ni][0] << "                 " << bar[ni][1] << endl;
	}//接收黄车障碍物位置信息

	float x = current.robots_blue(control_num).x() / 10;
	float y = current.robots_blue(control_num).y() / 10;//接收蓝车位置坐标

	int len;
	float **cmd_s;

	while (1)
	{
		robot_command->set_velocity_x(0);
		robot_command->set_velocity_y(0);
		robot_command->set_velocity_r(0);
		robots_command.SerializeToArray(cmd_buf, cmd_size);
		cout << sendto(sock_cmd, cmd_buf, cmd_size, 0, (SOCKADDR *)&addr_cmd, sizeof(addr_cmd)) << endl;

		fx = -fx;
		fy = -fy;

		SecureZeroMemory(vis_buf, vis_size);
		vis_val = recvfrom(sock_vis, vis_buf, vis_size, 0, (SOCKADDR*)&addr_clt, &SOCKADDR_IN_SIZE);
		inet_ntop(addr_clt.sin_family, &addr_clt, vis_ip_buf, vis_ip_size);
		Vision_DetectionFrame current;
		current.ParseFromArray(vis_buf, 1024 * 10);

		blue_size = current.robots_blue_size();
		yellow_size = current.robots_yellow_size();

		bar_i = 0;
		for (int blue_i = 0; blue_i < blue_size; blue_i++)
		{
			if (current.robots_blue(blue_i).robot_id() == control) {
				control_num = blue_i;
				robot_command->set_robot_id(control);
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

		for (int ni = 0; ni < bar_i; ni++)
		{
			cout << bar[ni][0] << "                 " << bar[ni][1] << endl;
		}

		float x = current.robots_blue(control_num).x() / 10;
		float y = current.robots_blue(control_num).y() / 10;//实时接收场上机器人位置信息

		struct timeb startTime, endTime;
		ftime(&startTime);

		while (pathl() > 5) {
			cmd_s = getpath(bar, x, y, fx, fy);
			if (!(is_right_arc(cmd_s))) {
				set_pathl(1000);
			}
		}//getpath函数运用RRT算法获取路径，实现路径规划
		len = pathl() - 1;
		cout << "start" << endl;
		set_pathl(1000);

		ftime(&endTime);
		printf("time: %d ms\n", (endTime.time - startTime.time) * 1000
			+ (endTime.millitm - startTime.millitm));

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
			cout << sendto(sock_debug, debug_buf, debug_size, 0, (SOCKADDR *)&addr_debug, sizeof(addr_debug)) << endl;
		}//将规划出的路径绘制成线显示在屏幕上

		float max_vel = 300;
		float mid_vel = 175;
		float mids_vel = 225;
		float min_vel = 25;

		float max_dis = 3600;
		float mid_dis = 900;//设定各控制速度参数

		for (int i = 0; i < len; i++)
		{
			while (true)
			{
				SecureZeroMemory(vis_buf, vis_size);
				vis_val = recvfrom(sock_vis, vis_buf, vis_size, 0, (SOCKADDR*)&addr_clt, &SOCKADDR_IN_SIZE);
				inet_ntop(addr_clt.sin_family, &addr_clt, vis_ip_buf, vis_ip_size);
				Vision_DetectionFrame current;
				current.ParseFromArray(vis_buf, 1024 * 10);

				cout << "x:" << current.robots_blue(control_num).x() << "y:" << current.robots_blue(control_num).y() << "orientation:" << current.robots_blue(control_num).orientation() << endl;
				
				float x = current.robots_blue(control_num).x() / 10;
				float y = current.robots_blue(control_num).y() / 10;
				float ori = current.robots_blue(control_num).orientation();

				float next_x = cmd_s[i + 1][0];
				float next_y = cmd_s[i + 1][1];
				float prev_x = cmd_s[i][0];
				float prev_y = cmd_s[i][1];

				float x_vel_;
				float y_vel_;
				float x_vel;
				float y_vel;//初始化速度、位置变量

				if ((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y) < 50.0)
					break;//判断是否到达目标点

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

				if ((prev_x - x)*(prev_x - x) + (prev_y - y)*(prev_y - y) < 400)
				{
					x_vel_ = mid_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
					y_vel_ = mid_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
				}

				else if ((prev_x - x)*(prev_x - x) + (prev_y - y)*(prev_y - y) < 900)
				{
					x_vel_ = mids_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
					y_vel_ = mids_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
				}//判断路径折角大小，以在寻出的多条路经中找出最短、无锐角折角路径，进行速度导向

				float x1 = prev_x;
				float y1 = prev_y;
				float x2 = next_x;
				float y2 = next_y;

				float k = (y1 - y2) / ((x1 - x2) + 0.0001);
				float theta = atan(k);
				float dis = sgn(fy)*((y1 - y2)*x - (x1 - x2)*y + x1 * y2 - x2 * y1) / sqrt((y1 - y2)*(y1 - y2) + (x1 - x2)*(x1 - x2))*sgn(-k);

				if (abs(dis) >= 30) {
					dis = sgn(dis) * 30;
				}

				float pro = 0.05;//参数

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
				y_vel_ = y_vel_ + ame_y;//进行速度控制，保证机器人朝向目标点前进

				x_vel = x_vel_ * cos(current.robots_blue(control_num).orientation()) + y_vel_ * sin(current.robots_blue(control_num).orientation());
				y_vel = y_vel_ * cos(current.robots_blue(control_num).orientation()) - x_vel_ * sin(current.robots_blue(control_num).orientation());
				//将世界坐标系所需速度转化为机器人坐标系速度
				robot_command->set_velocity_x(x_vel);
				robot_command->set_velocity_y(-y_vel);
				robot_command->set_velocity_r(0);
				robots_command.SerializeToArray(cmd_buf, cmd_size);
				cout << sendto(sock_cmd, cmd_buf, cmd_size, 0, (SOCKADDR *)&addr_cmd, sizeof(addr_cmd)) << endl;
				//向机器人发送速度指令
			}
		}
	}

	closesocket(sock_debug);
	closesocket(sock_cmd);
	closesocket(sock_vis);
	WSACleanup();
	cout << "server shutdown..." << endl;
	return 0;
	//关闭通讯
}