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
  return;
  // Still in inital state.
  // Wait for speed limit to be achieved.
  if (state == 0) {
    return;
  }
}

vector<double> Fsm::AchieveSpeedLimit()
{
  vector<double> s;

  // Load previous except for last.

  // Determine starting position for next path.
  double start_s;
  if (prev_s.size() > 0) {
    start_s = prev_s[prev_s.size()-1];
    for (int i = 0; i < prev_s.size()-1; i++) {
      s.push_back(prev_s[i]);
    }
  } else {
    start_s = car_s;
  }


  vector<double> start = {start_s, car_speed*mph_to_ms, 0};
  vector<double> end = {start_s + 300, speed_limit*1, 0};

  cout << "Current Speed " << car_speed << " converted to m per s " << car_speed*mph_to_ms << endl;

  vector<double> a_coeff = JMT(start, end, 14);



  for (int i = 1; i < 150; i++) {
    double T = i*dt;
    double st = a_coeff[0] + a_coeff[1]*T + a_coeff[2]*pow(T,2);
    st += a_coeff[3]*pow(T,3) + a_coeff[4]*pow(T,4) + a_coeff[5]*pow(T,5);
    s.push_back(st);
  }

  return s;
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

void Fsm::SetLocalizationData(vector<double> l)
{
  car_x = l[0];
  car_y = l[1];
  car_s = l[2];
  car_d = l[3];
  car_yaw = l[4];
  car_speed = l[5];
}

void Fsm::SetPrevPath(vector<double> s, vector<double> d)
{
  prev_s = s;
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
