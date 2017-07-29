#include "behavior_fsm.h"
#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Fsm::Fsm()
{
  state = 0;
  state_in_progress = false;
}

Fsm::~Fsm() {}

void Fsm::UpdateState()
{
  // Initial State. Can only change to Stay in Lane.
  if (state == 0) {
    if (car_speed*mph_to_ms > 19.8) {
      state = 1;
      current_state_count = 0;
    }
    return;
  }

  // Stay In Lane. Can only switch to follow car.
  if (state == 1) {
    int id = CarInFront();
    if (id != -1) {
      state = 2;
      current_state_count = 0;
    }
  }

  // If car in front disappears, got to stay in lane.
  if (state == 2) {
    int id = CarInFront();
    if (id == -1) {
      state = 1;
      current_state_count = 0;
    }
    current_state_count += 1;
    cout << "State count " << current_state_count  << endl;
    if (current_state_count > 30) {
      state = 3;
    }
  }



}

void Fsm::AchieveSpeedLimit()

{

  s_path = {car_s+45, car_s+50};
  d_path = {6.0, 6.0};
  //if (speed_diff < 5) speed_diff = 0;
  final_speed = speed_limit*0.95;
  if (car_speed*mph_to_ms > 10) {
    time_to_s_path = ((50.0 / final_speed) + (50.0 / (car_speed*mph_to_ms)))/2;
  } else {
    time_to_s_path = 4;
  }

  return;
}

void Fsm::StayInLane()
{
  int current_lane = FindLane(car_d);

  s_path = {car_s+35, car_s+40};
  d_path = {current_lane*4 + 2.0, current_lane*4 + 2.0};
  final_speed = speed_limit*0.95;
  time_to_s_path = ((40 / final_speed) + (40 / (car_speed*mph_to_ms)))/2;


}

void Fsm::FollowCar()
{
  int id = CarInFront();
  int current_lane = FindLane(car_d);

  double infront_speed = sqrt(sf[id][3]*sf[id][3] + sf[id][4]*sf[id][4]);
  cout << "Infront speed " << infront_speed << endl;

  s_path = {car_s+35, car_s+40};
  d_path = {current_lane*4+2.0, current_lane*4+2.0};


  if (sf[id][5] - car_s < 15) {
    final_speed = infront_speed *0.7;
    cout << "WAY TOO CLOSE" << endl;
  } else if (sf[id][5] - car_s < 35 ) {
    final_speed = infront_speed*0.9;
    cout << "Backing off" << endl;
  } else {
    final_speed = infront_speed;
    cout << "Matching Speed" << endl;
  }
  time_to_s_path = ((40 / final_speed) + (40 / (car_speed*mph_to_ms)))/2;

}

void Fsm::SwitchLanes()
{
  s_path = {car_s+45, car_s+50};
  d_path = {10.0, 10.0};
  final_speed = speed_limit*0.95;
  time_to_s_path = ((50 / final_speed) + (50 / (car_speed*mph_to_ms)))/2;
}

int Fsm::GetState()
{
  return state;
}

bool Fsm::GetStateInProgress()
{
  return state_in_progress;
}

void Fsm::SetStateInProgress(bool set)
{
  state_in_progress = set;
}


// Determine if car is in the same lane. If yes return ID.
int Fsm::CarInFront()
{
  int current_lane = FindLane(car_d);


  int id = - 1;
  double s_dist = 50;
  for (int i = 0; i < sf.size(); i++) {
    int sf_lane = FindLane(sf[i][6]);
    if (sf_lane == current_lane && sf[i][5] > car_s) {
      if (sf[i][5] - car_s < s_dist) {
        s_dist = sf[i][5] - car_s;
        id = i;
      }
    }

  }

  return id;
}

int Fsm::FindLane(double d_in)
{
  int lane; // 0-left, 1-middle, 2-right.
  if (d_in < 4) {
    lane = 0;
  } else if (d_in < 8) {
    lane = 1;
  } else {
    lane = 2;
  }
  return lane;
}

void Fsm::SetLocalizationData(vector<double> l)
{
  car_x = l[0];
  car_y = l[1];
  car_s = l[2];
  car_d = l[3];
  car_yaw = l[4];
  car_speed = l[5];
}

void Fsm::SetSensorFusion(vector<vector<double> > sf_in)
{
  sf = sf_in;
}

void Fsm::SetPrevPath(vector<double> s, vector<double> d)
{
  prev_s.clear();
  prev_s = s;
  prev_d.clear();
  prev_d = d;
}



// I'm having an issue I can't resolve when trying to use
// Eigen's .inverse() so have to do Gaussian elimination instead of
// the easier matrix method.
vector<double> Fsm::JMT(vector< double> start, vector <double> end, double T)
{
  vector<double> answer(6);
  answer[0] = start[0];
  answer[1] = start[1];
  answer[2] = start[2]/2;

  // Calculate RHS of equation.

  vector<double> rhs(3);

  rhs[0] = end[0] - (start[0]+start[1]*T+0.5*start[2]*T*T);
  rhs[1] = end[1] - (start[1]+start[2]*T);
  rhs[2] = end[2] - start[2];

  // Create matrix to perform gaussian elimination.

  MatrixXd m(3,4);
  m(0,0) = pow(T,3);
  m(0,1) = pow(T,4);
  m(0,2) = pow(T,5);
  m(0,3) = rhs[0];

  m(1,0) = pow(T,2) * 3;
  m(1,1) = pow(T,3) * 4;
  m(1,2) = pow(T,4) * 5;
  m(1,3) = rhs[1];

  m(2,0) = T * 6;
  m(2,1) = pow(T,2) * 12;
  m(2,2) = pow(T,3) * 20;
  m(2,3) = rhs[2];

  // Perform Gaussian elimination.
  double r0 = -1 * (m(1,0) / m(0,0));
  m.row(1) = m.row(1) + m.row(0)*r0;
  r0 = -1 * (m(2,0) / m(0,0));
  m.row(2) = m.row(2) + m.row(0) * r0;
  // First column is all zeros except for first.

  // Second column.
  double r1 = -1 * (m(0,1) / m(1,1));
  m.row(0) = m.row(0) + m.row(1) * r1;
  r1 = -1 * (m(2,1) / m(1,1));
  m.row(2) = m.row(2) + m.row(1) * r1;

  // Third Column.
  double r2 = -1 * (m(0,2) / m(2,2));
  m.row(0) = m.row(0) + m.row(2) * r2;
  r2 = -1 * (m(1,2) / m(2,2));
  m.row(1) = m.row(1) + m.row(2) * r2;


  answer[3] = m(0,3) / m(0,0);
  answer[4] = m(1,3) / m(1,1);
  answer[5] = m(2,3) / m(2,2);


  return answer;
}

vector<double> Fsm::GetSPath()
{
  return s_path;
}

vector<double> Fsm::GetDPath()
{
  return d_path;
}

double Fsm::GetTimeToSPath()
{
  return time_to_s_path;
}

double Fsm::GetFinalSpeed()
{
  return final_speed;
}
