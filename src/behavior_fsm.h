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

  // FSM Main Functions
  void UpdateState();

  // Initial State
  void AchieveSpeedLimit();

  // Stay in current lane and maintain speed.
  void StayInLane();

  // Follow Car infront.
  void FollowCar();

  void PrepareLaneSwitch();

  // Switch to a new lane.
  void SwitchLanes();

  double TimeToPath(double dist);

  // Return current state.
  int GetState();

  bool GetStateInProgress();

  void SetStateInProgress(bool set);

  // Auxiliary Functions (Used in main functions).
  int CarInFront();

  bool LaneFree(int lane);

  int FindLane(double d_in);

  vector<double> JMT(vector< double> start, vector <double> end, double T);




  // Get and Set Functions.
  void SetLocalizationData(vector<double> l);

  void SetSensorFusion(vector<vector<double> > sf_in);

  void SetPrevPath(vector<double> s, vector<double> d);

  vector<double> GetSPath();

  vector<double> GetDPath();

  double GetTimeToSPath();

  double GetFinalSpeed();




private:
  int state;
  // 0 - Starting state at well below speed limit. Increase to speed limit and don't
  // run into any cars infront.
  // 1 - Stay in lane.
  // 2 - Follow car infront.
  // 3 - Switch Lanes.

  bool state_in_progress;

  // Localization information.
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

  // Sensor Fusion
  vector<vector<double> > sf;

  // Previous Path.
  vector<double> prev_s;
  vector<double> prev_d;

  // State variables.
  int current_state_count = 0;
  int target_lane = -1;

  // New Path
  vector<double> s_path;
  vector<double> d_path;
  double time_to_s_path;
  double final_speed;

  // Constants
  const double speed_limit = 22.3;
  const double mph_to_ms = 0.44704;
  const double dt = 1.0/50.0;
};



#endif
