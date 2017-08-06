#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "behavior_fsm.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}


// My added functions.
void CreatePath(Fsm &fsm, vector<double> &x_vals, vector<double> &y_vals, vector<double> prev_x, vector<double> prev_y, vector<double> local, vector<vector<double> > sensor_fusion);
vector<double> JMT(vector< double> start, vector <double> end, double T); // Jerk Minimizing Trajectory.
vector<double> RectSmooth(vector<double> d, int smooth);


// Global Variables.
// So they can be easily accessed by CreatePath for use in
// getXY and other similar functions without having to pass all these
// vectors into CreatePath.
vector<double> g_map_waypoints_x;
vector<double> g_map_waypoints_y;
vector<double> g_map_waypoints_s;
vector<double> g_map_waypoints_dx;
vector<double> g_map_waypoints_dy;



int main() {
  // Initialize FSM.
  Fsm fsm;

  uWS::Hub h;
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  g_map_waypoints_x = map_waypoints_x;
  g_map_waypoints_y = map_waypoints_y;
  g_map_waypoints_s = map_waypoints_s;
  g_map_waypoints_dx = map_waypoints_dx;
  g_map_waypoints_dy = map_waypoints_dy;


  h.onMessage([&fsm, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
            vector<double> local(6);
            local[0] = car_x;
            local[1] = car_y;
            local[2] = car_s;
            local[3] = car_d;
            local[4] = car_yaw;
            local[5] = car_speed;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            CreatePath(fsm, next_x_vals, next_y_vals, previous_path_x, previous_path_y, local, sensor_fusion);
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });
https://stackoverflow.com/questions/13461538/lambda-of-a-lambda-the-function-is-not-captured
  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}








// Create the path the car should follow.
// Responsible for starting FSM and blending newly generated path
// with existing path to minimize jerk between the two.
void CreatePath(Fsm &fsm, vector<double> &x_vals, vector<double> &y_vals, vector<double> prev_x, vector<double> prev_y, vector<double> local, vector<vector<double> > sensor_fusion)
{
  bool debug = true;

  // Import localization data.
  double car_x = local[0];
  double car_y = local[1];
  double car_s = local[2];
  double car_d = local[3];
  double car_yaw = local[4];
  double car_speed = local[5];

  // Print Sensor fusion.
  //for (int i = 100; i < sensor_fusion.size(); i++) {
  //  cout << "ID: " << sensor_fusion[i][0] << endl;
  //  cout << "X: " << sensor_fusion[i][1] << endl;
  //  cout << "Y: " << sensor_fusion[i][2] << endl;
  //  cout << "S: " << sensor_fusion[i][5] << endl;
  //  cout << "D: " << sensor_fusion[i][6] << endl;
  //  cout << endl;
  //}

  //cout << "Current X, Y, Yaw, S, D " << car_x << ", " << car_y << ", " << car_yaw << ", ";
  //cout << car_s << ", " << car_d << endl;


  const double speed_limit = 22.3;
  const double mph_to_ms = 0.44704;
  const double dt = 1.0/50.0;

  fsm.SetLocalizationData(local);
  fsm.SetSensorFusion(sensor_fusion);

  vector<double> s;
  vector<double> d;
  double final_speed;

  if (fsm.GetStateInProgress() == false) {
    fsm.UpdateState();
    switch (fsm.GetState()) {
      case 0:
        fsm.AchieveSpeedLimit();
        s = fsm.GetSPath();
        d = fsm.GetDPath();
        final_speed = fsm.GetFinalSpeed();
        // xy to sd back to xy conversion isn't perfect since waypoints have
        // significant distance between them.
        cout << "State: Achieve Speed Limit." << endl;
        break;
      case 1:
        fsm.StayInLane();
        s = fsm.GetSPath();
        d = fsm.GetDPath();
        final_speed = fsm.GetFinalSpeed();
        cout << "State: Stay In Lane." << endl;
        break;
      case 2:
        fsm.FollowCar();
        s = fsm.GetSPath();
        d = fsm.GetDPath();
        final_speed = fsm.GetFinalSpeed();
        cout << "State: Follow Car." << endl;
        break;
      case 3:
        fsm.PrepareLaneSwitch();
        s = fsm.GetSPath();
        d = fsm.GetDPath();
        final_speed = fsm.GetFinalSpeed();
        cout << "State: Prepare Lane Switch." << endl;
        break;
      case 4:
        fsm.SwitchLanes();
        s = fsm.GetSPath();
        d = fsm.GetDPath();
        final_speed = fsm.GetFinalSpeed();
        cout << "State: Switch Lanes." << endl;
        break;
    }

    // Calculate xy for end points decided by FSM.
    vector<double> xy1 = getXY(s[0], d[0], g_map_waypoints_s, g_map_waypoints_x, g_map_waypoints_y);
    vector<double> xy2 = getXY(s[1], d[1], g_map_waypoints_s, g_map_waypoints_x, g_map_waypoints_y);
    double heading = atan2(xy2[1] - xy1[1], xy2[0] - xy1[0]);

    // Calculate new time to s based on xy distance.
    double distance = (xy2[1] - car_y)*(xy2[1] - car_y) + (xy2[0] - car_x)*(xy2[0] - car_x);
    distance = sqrt(distance);
    double time_to_s = fsm.TimeToPath(distance);

    // Calculate JMT for X.
    vector<double> x;
    vector<double> start;
    if (prev_x.size() < 10) {
      start = {car_x, car_speed*mph_to_ms*cos(car_yaw * 3.14159 / 180), 0};
    } else {
      distance = (xy2[1] - prev_y[9])*(xy2[1] - prev_y[9]) + (xy2[0] - prev_x[9])*(xy2[0] - prev_x[9]);
      distance = sqrt(distance);
      time_to_s = fsm.TimeToPath(distance);

      double v1 = (prev_x[9] - prev_x[8])*50;
      double v0 = (prev_x[8] - prev_x[7])*50;
      start = {prev_x[9], v1, v1 - v0};
      for (int i = 1; i <= 9; i++) {
        x.push_back(prev_x[i]);
      }
    }
    vector<double> end = {xy2[0], final_speed*cos(heading), 0};

    vector<double> a_coeff = fsm.JMT(start, end, time_to_s);

    int N;
    if (fsm.GetState() == 0 || fsm.GetState() == 4) {
      N = 150;
    } else {
      N = 55;
    }



    for (int i = 1; i < N; i++) {
      double T = i*dt;
      double xt = a_coeff[0] + a_coeff[1]*T + a_coeff[2]*pow(T,2);
      xt += a_coeff[3]*pow(T,3) + a_coeff[4]*pow(T,4) + a_coeff[5]*pow(T,5);
      x.push_back(xt);
    }

    // Calculate JMT for Y.
    vector<double> y;
    if (prev_y.size() < 10) {
      start = {car_y, car_speed*mph_to_ms*sin(car_yaw * 3.14159 / 180), 0};
    } else {
      double v1 = (prev_y[9] - prev_y[8])*50;
      double v0 = (prev_y[8] - prev_y[7])*50;
      start = {prev_y[9], v1, v1 - v0};
      for (int i = 1; i <= 9; i++) {
        y.push_back(prev_y[i]);
      }
    }
    end = {xy2[1], final_speed*sin(heading), 0};

    a_coeff = fsm.JMT(start, end, time_to_s);


    for (int i = 1; i < N; i++) {
      double T = i*dt;
      double yt = a_coeff[0] + a_coeff[1]*T + a_coeff[2]*pow(T,2);
      yt += a_coeff[3]*pow(T,3) + a_coeff[4]*pow(T,4) + a_coeff[5]*pow(T,5);
      y.push_back(yt);
    }

    x_vals = x;
    y_vals = y;

    int smooth_runs = 0;
    while (smooth_runs > 0){
      x_vals = RectSmooth(x_vals, 1);
      y_vals = RectSmooth(y_vals, 1);
      smooth_runs -= 1;
    }

    fsm.SetStateInProgress(true);
  } else {
    //cout << "In Current state" << endl;
    // Enter values into x and y values.
    prev_x.erase(prev_x.begin());
    prev_y.erase(prev_y.begin());

    x_vals = prev_x;
    y_vals = prev_y;

    int vals_size_min;
    if (fsm.GetState() == 0 || fsm.GetState() == 4) {
      vals_size_min = 40;
    } else {
      vals_size_min = 54;
    }
    if (x_vals.size() < vals_size_min) {
      fsm.SetStateInProgress(false);
    }
  }

  return;
}

// Used to smooth x and y values.
// Erases smooth number of points at the end.
vector<double> RectSmooth(vector<double> d, int smooth)
{
  vector<double> out;
  for (int i = 0; i < smooth; i++) {
    out.push_back(d[i]);
  }



  for (int i = smooth; i < d.size()-smooth; i++) {
    double avg = d[i];

    for (int j = 1; j < smooth+1; j++) {
      avg += d[i+j];
      avg += d[i-j];
    }
    out.push_back(avg / (smooth*2+1));
  }
  return out;
}
