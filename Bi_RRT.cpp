#include "RRTStar.h"

#define obs_size 18
#define obs_num 15

//rrt���캯��
RRTStar::RRTStar::RRTStar(float** obstacle, float start_x, float start_y, float end_x, float end_y)
{

	reach_goal = false;

	setmap(300, 225);
	setstepsize(30);
	setnearradius(30);
	setgoalbias(0.07);
	setgoalradius(7.0);
	setmaxiterations(5000);

	Vec2i start, goal;
	start.x = start_x;
	start.y = start_y;
	goal.x = end_x;
	goal.y = end_y;

	for (int i = 0; i < obs_num; i++) {
		Vec2i temp;
		temp.x = obstacle[i][0];
		temp.y = obstacle[i][1];
		Obstacleset.push_back(temp);
	}

	findPath(start, goal);
}

//��㹹�캯��
RRTStar::Vertex::Vertex(Vec2i coordinates_, Vertex *parent_, float cost_)
{
	coordinates = coordinates_;
	parent = parent_;
	cost = cost_;
}

//�趨��ͼ��С
void RRTStar::RRTStar::setmap(float map_width_, float map_height_)
{
	map_width = map_width_;
	map_height = map_height_;
}

//�趨Ŀ��������
void RRTStar::RRTStar::setgoalbias(float goal_bias_)
{
	goal_bias = goal_bias_;
}

//�趨����
void RRTStar::RRTStar::setstepsize(float step_size_)
{
	step_size = step_size_;
}

//�趨����������
void RRTStar::RRTStar::setmaxiterations(int max_iterations_)
{
	max_iterations = max_iterations_;
}

//�趨Ŀ�굽�ﷶΧ
void RRTStar::RRTStar::setgoalradius(float goal_radius_)
{
	goal_radius = goal_radius_;
}

//�趨�ٽ��㷶Χ
void RRTStar::RRTStar::setnearradius(float near_radius_)
{
	near_radius = near_radius_;
}

//����ϰ���
void RRTStar::RRTStar::addobstacle(Vec2i obstacle_)
{
	Obstacleset.push_back(obstacle_);
}

//����������߶����ϰ����Ƿ���ײ
bool RRTStar::RRTStar::isHit(Vec2i coordinates1_, Vec2i coordinates2_)
{
	if (Obstacleset.size() == 0)
	{
		return false;
	}

	for (int i = 0; i < Obstacleset.size(); i++)
	{
		Vec2i bottomleft = { Obstacleset[i].x - obs_size, Obstacleset[i].y - obs_size };
		Vec2i bottomright = { Obstacleset[i].x + obs_size, Obstacleset[i].y - obs_size };
		Vec2i topleft = { Obstacleset[i].x - obs_size, Obstacleset[i].y + obs_size };
		Vec2i topright = { Obstacleset[i].x + obs_size, Obstacleset[i].y + obs_size };

		bool top = islineintersect(coordinates1_, coordinates2_, topleft, topright);
		bool bottom = islineintersect(coordinates1_, coordinates2_, bottomleft, bottomright);
		bool left = islineintersect(coordinates1_, coordinates2_, topleft, bottomleft);
		bool right = islineintersect(coordinates1_, coordinates2_, topright, bottomright);

		if (top || bottom || left || right)
		{
			return true;
		}
	}
	return false;
}

//������߶��Ƿ��ཻ
bool RRTStar::RRTStar::islineintersect(Vec2i line1p1_, Vec2i line1p2_, Vec2i line2p1_, Vec2i line2p2_)
{
	float uA = ((line2p2_.x - line2p1_.x)*(line1p1_.y - line2p1_.y) - (line2p2_.y - line2p1_.y)*
		(line1p1_.x - line2p1_.x)) / ((line2p2_.y - line2p1_.y)*(line1p2_.x - line1p1_.x) -
		(line2p2_.x - line2p1_.x)*(line1p2_.y - line1p1_.y));

	float uB = ((line1p2_.x - line1p1_.x)*(line1p1_.y - line2p1_.y) - (line1p2_.y - line1p1_.y)*
		(line1p1_.x - line2p1_.x)) / ((line2p2_.y - line2p1_.y)*(line1p2_.x - line1p1_.x) -
		(line2p2_.x - line2p1_.x)*(line1p2_.y - line1p1_.y));

	if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1)
	{
		return true;
	}
	return false;
}

//�����Ƿ����ϰ��ﷶΧ��
bool RRTStar::RRTStar::isInObstacle(const Vec2i& coordinates_)
{
	if (Obstacleset.size() == 0)
	{
		return false;
	}

	for (int i = 0; i < Obstacleset.size(); i++)
	{
		if (coordinates_.x >= Obstacleset[i].x - obs_size
			&& coordinates_.x <= Obstacleset[i].x + obs_size
			&& coordinates_.y >= Obstacleset[i].y - obs_size
			&& coordinates_.y <= Obstacleset[i].y + obs_size)
		{
			return true;
		}
	}
	return false;
}

//����Ƿ�ִ�Ŀ���
bool RRTStar::RRTStar::isGoal(Vec2i source_, Vec2i goal_)
{
	float distance = euclidean_dis(source_, goal_);

	if (distance <= goal_radius)
	{
		return true;
	}
	return false;
}

//���������Ƿ����ϰ�������Ϊ·����
bool RRTStar::RRTStar::isValid(Vec2i coordinates_, Vec2i closestvertex_)
{
	if (coordinates_.x > -map_width && coordinates_.y > -map_height
		&& coordinates_.x < map_width && coordinates_.y < map_height
		&& closestvertex_.x > -map_width && closestvertex_.y > -map_height
		&& closestvertex_.x < map_width && closestvertex_.y < map_height
		&& isInObstacle(coordinates_) == false && isInObstacle(closestvertex_) == false
		&& isHit(coordinates_, closestvertex_) == false)
	{
		return true;
	}
	return false;
}

//����������ŷʽ����
float RRTStar::RRTStar::euclidean_dis(Vec2i source_, Vec2i goal_)
{
	float e_distance = sqrt(pow(source_.x - goal_.x, 2) + pow(source_.y - goal_.y, 2));
	return e_distance;
}

//�ڵ�ͼ��Χ�����������
RRTStar::Vec2i RRTStar::RRTStar::GenerateRandomPoint(Vec2i goal_)
{
	Vec2i randompoint;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> x(-map_width, map_width);
	std::uniform_real_distribution<> y(-map_height, map_height);

	randompoint.x = rand() % 600 + 1 - 300;
	randompoint.y = rand() % 450 + 1 - 225;

	bool setgoal = (rand() % 100 + 1)*0.01 <= goal_bias;

	if (setgoal == true)
	{

		return goal_;
	}

	return randompoint;
}

//�õ��㼯�������������Ľڵ�
RRTStar::Vertex* RRTStar::RRTStar::getClosestVertex(std::set<Vertex*>& Vertices_, Vec2i randompoint_)
{
	Vertex* closestvertex = NULL;
	float min_distance = std::numeric_limits<float>::max();

	for (auto vertex : Vertices_) {
		if (euclidean_dis(vertex->coordinates, randompoint_) < min_distance) {
			min_distance = euclidean_dis(vertex->coordinates, randompoint_);
			closestvertex = vertex;
		}
	}
	return closestvertex;
}

//�������ڵ�
bool RRTStar::RRTStar::extend(Vertex* closestvertex_A, Vertex* closestvertex_B, Vec2i randompoint_A, Vec2i randompoint_B, Vec2i source_, Vec2i goal_)
{
	//������������㷽���ƶ���������õ��µĽڵ�
	float thetaA = atan2(randompoint_A.y - closestvertex_A->coordinates.y, randompoint_A.x - closestvertex_A->coordinates.x);
	float thetaB = atan2(randompoint_B.y - closestvertex_B->coordinates.y, randompoint_B.x - closestvertex_B->coordinates.x);
	Vec2i vertextempA, vertextempB;
	vertextempA.x = closestvertex_A->coordinates.x + step_size * cos(thetaA);
	vertextempA.y = closestvertex_A->coordinates.y + step_size * sin(thetaA);
	vertextempB.x = closestvertex_B->coordinates.x + step_size * cos(thetaB);
	vertextempB.y = closestvertex_B->coordinates.y + step_size * sin(thetaB);
	int Afind = 0, Bfind = 0;
	
	//�ж��½ڵ��Ƿ��뵱ǰ�ڵ���ӣ���������������б������ųɹ�
	if (isValid(vertextempA, closestvertex_A->coordinates) == true)
	{
		visitednode.push_back(vertextempA);
		//�������·��
		rewire(closestvertex_A, vertextempA, current_A);
		Afind = 1;
	}
	if (isValid(vertextempB, closestvertex_B->coordinates) == true)
	{
		visitednode.push_back(vertextempB);
		rewire(closestvertex_B, vertextempB, current_B);
		Bfind = 1;
	}
	if (Afind == 1 || Bfind == 1) {
		return true;
	}
	return false;
}

//�������·��
void RRTStar::RRTStar::rewire(Vertex* closestvertex_, Vec2i newvertex_, Vertex* current)
{
	std::set<Vertex*> Nearset;

	for (auto vertex : VertexSet)
	{
		if (euclidean_dis(vertex->coordinates, newvertex_) <= near_radius) {
			Nearset.insert(vertex);
		}
	}

	Vertex* newparent = closestvertex_;

	float mincost = closestvertex_->cost + step_size;
	for (auto nearvertex : Nearset)
	{
		float tempcost = nearvertex->cost + euclidean_dis(nearvertex->coordinates, newvertex_);
		if (tempcost < mincost && isValid(newvertex_, nearvertex->coordinates))
		{
			mincost = tempcost;
			newparent = nearvertex;
		}
	}

	Vertex* newvertex = new Vertex(newvertex_, newparent, mincost);
	
	current = newvertex;

	VertexSet.insert(newvertex);

	for (auto nearvertex : Nearset)
	{
		if (nearvertex != newparent)
		{
			if (isValid(nearvertex->coordinates, newvertex->coordinates) &&
				nearvertex->cost > (newvertex->cost + euclidean_dis(nearvertex->coordinates, newvertex->coordinates)))
			{
				nearvertex->parent = newvertex;
				nearvertex->cost = newvertex->cost + euclidean_dis(nearvertex->coordinates, newvertex->coordinates);
			}
		}
	}
}

//Ѱ·
void RRTStar::RRTStar::findPath(Vec2i source_, Vec2i goal_)
{
	//����ʼ����յ�ֱ���Ϊ�������ĸ��ڵ�
	bool done_flag = false;
	VertexSetA.insert(new Vertex(source_));
	current_A = *VertexSetA.begin();
	VertexSetB.insert(new Vertex(goal_));
	current_B = *VertexSetB.begin();
	int current_iterations = 0;
	//���ϵ���������
	while (done_flag != true && current_iterations < max_iterations)
	{

		Vec2i randompointA, randompointB;
		Vertex* closestvA, *closestvB;
		//A��B���ֱ��ԶԷ��½ڵ�Ϊ��������������ٽ���
		randompointA = GenerateRandomPoint(current_B->coordinates);
		closestvA = getClosestVertex(VertexSetA, randompointA);
		randompointB = GenerateRandomPoint(current_A->coordinates);
		closestvB = getClosestVertex(VertexSetB, randompointB);
		//����һ�������Žڵ�ɹ����ж��������Ƿ����ӳɹ��������ӳɹ���Ѱ·�ɹ��������������
		if (extend(closestvA, closestvB, randompointA, randompointA, source_, goal_) == true)
		{
			Vertex* closestvB = getClosestVertex(VertexSetB, current_A->coordinates);
			Vertex* closestvA = getClosestVertex(VertexSetA, current_B->coordinates);
			if (isValid(current_A->coordinates, closestvB->coordinates) == true &&
				euclidean_dis(current_A->coordinates, closestvB->coordinates) <= step_size)
			{
				done_flag = true;
				current_B = closestvB;
				std::cout << "Found a path ";
			}
			if (isValid(current_B->coordinates, closestvA->coordinates) == true &&
				euclidean_dis(current_B->coordinates, closestvA->coordinates) <= step_size)
			{
				done_flag = true;
				current_A = closestvA;
				std::cout << "Found a path ";
			}

		}
		//������������ﵽ�����Ѱ·ʧ��
		if (current_iterations == max_iterations)
		{
			std::cout << "No path found." << std::endl;
			current_A = NULL;
			current_B = NULL;
			releaseVertices(VertexSetA);
			releaseVertices(VertexSetB);
			return;
		}
		current_iterations++;
	}

	//Ѱ·�ɹ�����A��B���Ľڵ�Ž�path�㼯
	while (current_A != NULL)
	{
		path.push_back(current_A->coordinates);
		current_A = current_A->parent;
	}
	reverse(path.begin(), path.end());
	while (current_B != NULL)
	{
		path.push_back(current_B->coordinates);
		current_B = current_B->parent;
	}

	//���ѭ��ƽ��·��
	for (size_t i = 0; i < 5; i++)
	{
		minsmoothpath(goal_);
		path = smooth_path;
		smooth_path.clear();
	}

	//�ͷ����Ľڵ㼯��
	releaseVertices(VertexSetA);
	releaseVertices(VertexSetB);
}

//·��ƽ����
void RRTStar::RRTStar::minsmoothpath(Vec2i goal_)
{
	if (path.size() <= 2) {
		smooth_path = path;
		return;
	}
	smooth_path.push_back(path[0]);
	int index1 = 0;
	int index2 = 1;
	
	//����㿪ʼѭ�����ж�������Ƿ���ӣ��������ӵ�ֱ�������ӵõ���̵�һ�ο���·��
	while (true)
	{
		if (isValid(path[index1], path[index2]) == true)
		{
			index2++;
		}
		else
		{
			if (index2 - 1 != 0)
			{
				smooth_path.push_back(path[index2 - 1]);

			}
			index1 = index2 - 1;
			index2 = index1 + 1;
		}
		if (index1 + 1 == path.size() || index2 == path.size())
		{
			break;
		}
	}

	if (smooth_path.back().x != goal_.x && smooth_path.back().y != goal_.y)
	{
		smooth_path.push_back(path.back());
	}
}

//�ͷ����ڵ�
void RRTStar::RRTStar::releaseVertices(std::set<Vertex*>& Vertices_)
{
	std::cout << "Visited vertices: " << Vertices_.size() << std::endl;
	for (auto it = Vertices_.begin(); it != Vertices_.end();)
	{
		delete *it;
		it = Vertices_.erase(it);
	}
}
