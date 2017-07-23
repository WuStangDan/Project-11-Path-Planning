#ifndef BEHAVIOR_FSM_H
#define BEHAVIOR_FSM_H
#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

class Fsm {
public:
  // Constructor.
  Fsm();
  // Destructor.
  ~Fsm();

  void UpdateState();

  // Initial State
  vector<double> AchieveSpeedLimit();

  // Return current state.
  int GetState();

  bool GetStateInProgress();

  void SetStateInProgress(bool set);

  void SetLocalizationData(vector<double> l);

  void SetPrevPath(vector<double> s, vector<double> d);

  vector<double> JMT(vector< double> start, vector <double> end, double T);




private:
  int state;
  // 0 - Starting state at well below speed limit. Increase to speed limit and don't
  // run into any cars infront.

  // 1 - Keep lane.

  bool state_in_progress;

  // Localization information.
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

  // Previous Path.
  vector<double> prev_s;
  vector<double> prev_d;

  // Constants
  const double speed_limit = 22.3;
  const double mph_to_ms = 0.44704;
  const double dt = 1.0/50.0;
};



#endif
