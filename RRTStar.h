#ifndef _RRTSTAR_H_
#define _RRTSTAR_H_

#include <vector>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <set>
#include <limits>
#include <fstream>
#include <ostream>
#include <random>

namespace RRTStar
{
	struct Vec2i
	{
		float x, y;
	};

	struct Vertex
	{
		Vec2i coordinates;
		Vertex* parent;
		float cost;
		Vertex(Vec2i coordinates, Vertex *parent = NULL, float cost = 0);
	};

	class RRTStar
	{
	private:
		float near_radius;
		float step_size;
		int max_iterations;
		float goal_bias;
		float goal_radius;

		bool reach_goal;

		std::set<Vertex*> VertexSet;

		Vertex* current_A;
		Vertex* current_B;

		std::set<Vertex*> VertexSetA;
		std::set<Vertex*> VertexSetB;

	public:

		RRTStar(float** obstacle, float start_x, float start_y, float end_x, float end_y);
		
		float map_width;
		float map_height;
		
		std::vector<Vec2i> path;
		std::vector<Vec2i> smooth_path;

		int path_size;

		std::vector<Vec2i> Obstacleset;

		Vec2i GenerateRandomPoint(Vec2i goal_);
		Vertex* getClosestVertex(std::set<Vertex*>& Vertices_, Vec2i randompoint_);

		void setmap(float map_width_, float map_height_);
		void setstepsize(float step_size_);
		void setgoalbias(float goal_bias_);
		void setnearradius(float near_radius_);
		void addobstacle(Vec2i obstacle_);
		void setmaxiterations(int max_iterations_);
		void setgoalradius(float goal_radius_);

		void minsmoothpath(Vec2i goal_);
		bool isHit(Vec2i coordinates1_, Vec2i coordinates2_);
		bool islineintersect(Vec2i line1p1, Vec2i line1p2, Vec2i line2p1, Vec2i line2p2);
		bool isInObstacle(const Vec2i& coordinates_);
		bool isGoal(Vec2i source_, Vec2i goal_);
		bool isValid(Vec2i coordinates_, Vec2i closestvertex_);
		float euclidean_dis(Vec2i source_, Vec2i goal_);
		bool extend(Vertex* closestvertex_A, Vertex* closestvertex_B, Vec2i randompoint_A, Vec2i randompoint_B, Vec2i source_, Vec2i goal_);
		void rewire(Vertex* closestvertex_, Vec2i newvertex_, Vertex* current);
		void findPath(Vec2i source_, Vec2i goal_);
		void releaseVertices(std::set<Vertex*>& Vertices_);
	};
}
#endif