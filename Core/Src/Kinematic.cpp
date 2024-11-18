#include "Encoder.h"
#include "Kinematic.h"
#include "mainpp.h"
#include "DataStruct.h"
#include "vector"

using namespace kinematic;

Motor mtr_;
Point2D pos;
Point2D velocity;
Point2D m_out;
std::vector<double>e{0,0,0};
std::vector<double>prev_e{0,0,0};
std::vector<double> v{0,0,0};

extern  STM32 Send;
void Motor::calcOdom(){
	e[0] = (float)Send.Encoder[0];
	e[1] = (float)Send.Encoder[1];
	e[2] = (float)Send.Encoder[2];

	for(int i = 0; i < 3; i++){
		v[i] = e[i] - prev_e[i];
	}

	mtr_.v1 = v[0] / 7500 * M_PI * 10;
//	e1.ResetCounter();
	mtr_.v2 = v[1] / 7500 * M_PI * 10;
//	e2.ResetCounter();
	mtr_.v3 = v[2] / 7500 * M_PI * 10;
//	e3.ResetCounter();
	velocity.x = ((-sqrt(3) * mtr_.v1) + (sqrt(3) * mtr_.v3)) / 3;
	velocity.y = ((-2 * mtr_.v2) + mtr_.v1 + mtr_.v3) / 3;
	velocity.theta = (mtr_.v1 + mtr_.v2 + mtr_.v3) / (3 * 20.8);

	double radian = fmod(pos.theta, M_PI*2);
	if(radian < 0)
		radian += M_PI *2;

	Point2D vel_glob;
	vel_glob.x = (std::cos(radian) * velocity.x) - (std::sin(radian) * velocity.y);
	vel_glob.y = (std::cos(radian) * velocity.y) + (std::sin(radian) * velocity.x);


	pos.x += vel_glob.x;
	pos.y += vel_glob.y;
	pos.theta += velocity.theta;

	prev_e = e;
}

void Motor::inverseKinematic(float x,float y,float z,Point2D &output){
	Point2D vel_world;
	Point2D Vin;
	Point2D Vrad;
	Vin.x = x;
	Vin.y = y;
	Vin.theta = z;

	vel_world.x = cos(pos.theta) * Vin.x + sin(pos.theta) * Vin.y;
	vel_world.y = -sin(pos.theta) * Vin.x + cos(pos.theta) * Vin.y;
	vel_world.theta = Vin.theta;

	Vrad.a = ((-sqrt(3) * vel_world.x / 2) + (vel_world.y / 2) + 20.8 * vel_world.theta) / 5;
	Vrad.b = (-vel_world.y + 20.8 * vel_world.theta) / 5;
	Vrad.c = ((sqrt(3) * vel_world.x / 2) + (vel_world.y / 2) + 20.8 * vel_world.theta) / 5;

	output.a = Vrad.a / (433 * (2* M_PI / 60)) * 225;
	output.b = Vrad.b / (433 * (2* M_PI / 60)) * 225;
	output.c = Vrad.c / (433 * (2* M_PI / 60)) * 225;

}
Point2D Motor::getpos(){
	return pos;
}
