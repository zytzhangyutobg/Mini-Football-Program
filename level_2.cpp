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

	float bar[15][2];

	int control = 0;
	int control_num;


	Robots_Command robots_command;
	Robot_Command *robot_command;
	robot_command = robots_command.add_command();

	robot_command->set_velocity_x(0);
	robot_command->set_velocity_y(0);
	robot_command->set_velocity_r(0);
	robot_command->set_robot_id(control);
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


	float finish_x = 250;
	float finish_y = -150;
	float fx = finish_x;
	float fy = finish_y;


	SecureZeroMemory(vis_buf, vis_size);
	vis_val = recvfrom(sock_vis, vis_buf, vis_size, 0, (SOCKADDR*)&addr_clt, &SOCKADDR_IN_SIZE);
	inet_ntop(addr_clt.sin_family, &addr_clt, vis_ip_buf, vis_ip_size);
	Vision_DetectionFrame current;
	current.ParseFromArray(vis_buf, 1024 * 10);




	int blue_size = current.robots_blue_size();
	int yellow_size = current.robots_yellow_size();

	int bar_i = 0;
	for (int blue_i = 0; blue_i < blue_size; blue_i++)
	{
		if (current.robots_blue(blue_i).robot_id() == control) {
			control_num = blue_i;
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

	cout << "x:" << current.robots_blue(control_num).x() << "y:" << current.robots_blue(control_num).y() << "orientation:" << current.robots_blue(control_num).orientation() << endl;
	float x = current.robots_blue(control_num).x() / 10;
	float y = current.robots_blue(control_num).y() / 10;


	float previous_x=x;
	float previous_y=y;

	int flag_num = 0;
	bool flag;
	int flag_after = 0;
	
	int len=1;
	
	float cmd_s[2][2] = { {-250,150},{250,-150} };

	float vel = 75.0;
	float vel2 = 150;
	float vel3 = 0;
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

		cmd_s[0][0] *= -1;
		cmd_s[0][1] *= -1;
		cmd_s[1][0] *= -1;
		cmd_s[1][1] *= -1;


		blue_size = current.robots_blue_size();
		yellow_size = current.robots_yellow_size();

		bar_i = 0;
		for (int blue_i = 0; blue_i < blue_size; blue_i++)
		{
			if (current.robots_blue(blue_i).robot_id() == control) {
				
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

		cout << "x:" << current.robots_blue(control_num).x() << "y:" << current.robots_blue(control_num).y() << "orientation:" << current.robots_blue(control_num).orientation() << endl;
		float x = current.robots_blue(control_num).x() / 10;
		float y = current.robots_blue(control_num).y() / 10;

		




	/*	struct timeb startTime, endTime;
		ftime(&startTime);
		//c++;
		while (pathl() > 5) {
			cmd_s = getpath(bar, x, y, fx, fy);
			if (!(is_right_arc(cmd_s))) {
				set_pathl(1000);
			}
		}
		len = pathl() - 1;
		cout << "start" << endl;
		set_pathl(1000);

		ftime(&endTime);
		printf("time: %d ms\n", (endTime.time - startTime.time) * 1000
			+ (endTime.millitm - startTime.millitm));*/

		
		


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
		}



		float max_vel = 250;
		float mid_vel = 100;
		float mids_vel = 105;
		float min_vel = 25;

		/*float max_vel = 200;
		float mid_vel = max_vel;
		float mids_vel = mid_vel;
		float min_vel = mids_vel;*/

		float max_dis = 3600;
		float mid_dis = 900;


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
				float y_vel;

				if ((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y) < 50.0)
					break;



			
			
				/*if ((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y) > 6400.0)
				{*/
					x_vel_ = max_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
					y_vel_ = max_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
				/*}
				else
				{
					x_vel_ = mid_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
					y_vel_ = mid_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
				}*/
				
				/*if (i == len - 1)
				{
					if ((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y) > 2500.0)
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
				
				if ((prev_x - x)*(prev_x - x) + (prev_y - y)*(prev_y - y) < 81)
				{
					x_vel_ = mid_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
					y_vel_ = mid_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
				}

				else if ((prev_x - x)*(prev_x - x) + (prev_y - y)*(prev_y - y) < 225)
				{
					x_vel_ = mids_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
					y_vel_ = mids_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
				}

				*/
				cout << "     x_vel_:    " << x_vel_ << "      y_vel_       " << y_vel_ << endl;



				float x1 = prev_x;
				float y1 = prev_y;
				float x2 = next_x;
				float y2 = next_y;

				float k = (y1 - y2) / ((x1 - x2) + 0.0001);
				float theta = atan(k);
				float dis = sgn(fy)*((y1 - y2)*x - (x1 - x2)*y + x1 * y2 - x2 * y1) / sqrt((y1 - y2)*(y1 - y2) + (x1 - x2)*(x1 - x2))*sgn(-k);

				
				if (abs(dis) >= 20.0) {
					dis = sgn(dis) * 20.0;
				}


				float pro = 0.07;//参数

				float ame_x = -pro * dis*dis*dis * sin(theta);
				float ame_y = +pro * dis*dis*dis * cos(theta);

				if (abs(ame_x) >= 200) {
					ame_x = sgn(ame_x) * 200;
				}

				if (abs(ame_y) >= 200) {
					ame_y = sgn(ame_y) * 200;
				}


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

				

				cout << "     ame_x:    " << ame_x << "      ame_y       " << ame_y << endl;
				


				x_vel_ = x_vel_ + ame_x;
				y_vel_ = y_vel_ + ame_y;
				
				
				/////////////////////
				blue_size = current.robots_blue_size();
				yellow_size = current.robots_yellow_size();

				bar_i = 0;
				for (int blue_i = 0; blue_i < blue_size; blue_i++)
				{
					if (current.robots_blue(blue_i).robot_id() == control) {
						
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
				float x_plus = 0;
				float y_plus = 0;
				float bilixishu = 17000;

				for (int bari = 0; bari < bar_i; bari++)
				{
					if ((bar[bari][0] / 10 - x)*(bar[bari][0] / 10 - x) + (bar[bari][1] / 10 - y)*(bar[bari][1] / 10 - y) <= 12100)
					{
						float theta = atan2(y - bar[bari][1] / 10, x - bar[bari][0] / 10);
						float distance = sqrt((bar[bari][0] / 10 - x)*(bar[bari][0] / 10 - x) + (bar[bari][1] / 10 - y)*(bar[bari][1] / 10 - y));
						y_plus += bilixishu / (distance) * sin(theta);
						x_plus += bilixishu / (distance) * cos(theta);
						cout << "x::" << bilixishu / distance * cos(theta) << "      yy::"<<bilixishu / distance * sin(theta) << endl;
					}
				}

				for (int bari = 0; bari < bar_i; bari++)
				{
					float v_theta = atan2(cmd_s[1][1]-cmd_s[0][1], cmd_s[1][0] - cmd_s[0][0]);
					float bar_theta = atan2(bar[bari][1] / 10 - y, bar[bari][0] / 10 - x + 0.00001);
					float diff = bar_theta - v_theta;
					float d_v_bar = sqrt((bar[bari][0] / 10 - x)*(bar[bari][0] / 10 - x) + (bar[bari][1] / 10 - y)*(bar[bari][1] / 10 - y));
					if (d_v_bar <= 50 && ((diff >= -3.14159 / 6.0 && diff <= -3.14159 / 6.0) || (diff <= -3.14159 * 5.5 / 6.0) || (diff >= 3.14159 * 5.5 / 6.0)))
					{
						x_vel_ = 1.0 / 2.0 * x_vel_;
						y_vel_ = 1.0 / 2.0 * y_vel_;
						cout << "x:::"  << "      yy:::" << endl;
					}
				}

				cout << "     x_vel_:    " << x_vel_ << "      y_vel_       " << y_vel_ << endl;
				if ((x - previous_x)*(x - previous_x) + (y - previous_y)*(y - previous_y) <= 2500.00) {
					flag_num++;
					if (flag_num >= 30) {
						flag = true;
						x_plus = 0;
						y_plus = 0;
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
				////////////////////////////
				if (flag == true) {
					x_vel_ = x_vel_ - sgn(x) * 200;
				}

				/*if (flag_after!=0) {
					x_vel_ = x_vel_ - sgn(x) * 200;
					flag_after--;
				}*/

				/*if ((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y) < 2500.0)
				{
					x_vel_ = mid_vel * (next_x - x) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
					y_vel_ = mid_vel * (next_y - y) / sqrt((next_x - x)*(next_x - x) + (next_y - y)*(next_y - y));
				}*/
				x_vel_ += x_plus;
				y_vel_ += y_plus;

				if (y-x <- 285||y-x>290)
				{
					x_vel_ = 0.35*x_vel_;
					y_vel_ = 0.35*y_vel_;

					cout << "===================================" << endl;
				}



				x_vel = x_vel_ * cos(current.robots_blue(control_num).orientation()) + y_vel_ * sin(current.robots_blue(control_num).orientation());
				y_vel = y_vel_ * cos(current.robots_blue(control_num).orientation()) - x_vel_ * sin(current.robots_blue(control_num).orientation());

				cout << "     x_vel_:    " << x_vel_ << "      y_vel_       " << y_vel_ << endl;





				robot_command->set_velocity_x(x_vel);
				robot_command->set_velocity_y(-y_vel);
				robot_command->set_velocity_r(0);
				robots_command.SerializeToArray(cmd_buf, cmd_size);
				cout << sendto(sock_cmd, cmd_buf, cmd_size, 0, (SOCKADDR *)&addr_cmd, sizeof(addr_cmd)) << endl;







			}

		}





	}

	closesocket(sock_debug);
	closesocket(sock_cmd);
	closesocket(sock_vis);
	WSACleanup();
	cout << "server shutdown..." << endl;
	return 0;

}