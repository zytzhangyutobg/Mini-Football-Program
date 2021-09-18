#ifndef MOVE_H
#define MOVE_H

int pathl();
bool is_sight(float** obs, float x1, float y1, float x2, float y2);
float **getpath(float bar[][2], float start_x, float start_y, float end_x, float end_y);
void set_pathl(int num);
int is_right_arc(float** c);

#endif // !MOVE_H


