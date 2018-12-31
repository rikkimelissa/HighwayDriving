#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "path_planner.h" 
#include "spline.h"
// #include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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

vector<double> JMT(vector<double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.
    */
    
    MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
          3*T*T, 4*T*T*T,5*T*T*T*T,
          6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
          end[1]-(start[1]+start[2]*T),
          end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < C.size(); i++)
  {
      result.push_back(C.data()[i]);
  }
  
    return result;
    
}
double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  // std::cout << theta << " " << heading << endl;

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXYspline(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;
  // double x1, x2, x3, y1, y2, y3;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }
  int wp2 = (prev_wp+1)%maps_x.size();

  if ((s - maps_s[prev_wp]) < (maps_s[wp2] - s)){
    prev_wp = (prev_wp-1)%maps_x.size();;
    wp2 = (wp2-1)%maps_x.size();;
  }

  std::cout << s << " " << maps_s[prev_wp] << " " << maps_s[wp2] << endl;
  int wp3 = (prev_wp+2)%maps_x.size();

  double x1 = (maps_x[prev_wp] + maps_x[wp2])/2.0;
  double x2 = maps_x[wp2];
  double x3 = (maps_x[wp3] + maps_x[wp2])/2.0;

  double y1 = (maps_y[prev_wp] + maps_y[wp2])/2.0;
  double y2 = maps_y[wp2];
  double y3 = (maps_y[wp3] + maps_y[wp2])/2.0;


  MatrixXd A = MatrixXd(3, 3);
  A << 3, x1+x2+x3, x1*x1+x2*x2+x3*x3,
          x1+x2+x3, x1*x1+x2*x2+x3*x3, x1*x1*x1+x2*x2*x2+x3*x3*x3,
          x1*x1+x2*x2+x3*x3, x1*x1*x1+x2*x2*x2+x3*x3*x3, x1*x1*x1*x1+x2*x2*x2*x2+x3*x3*x3*x3;
    
  MatrixXd B = MatrixXd(3,1);     
  B << y1+y2+y3,
          y1*x1+y2*x2+y3*x3,
          y1*x1*x1+y2*x2*x2+y3*x3*x3;
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;

  // result.push_back(C.data()[i]);
  





  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  // double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  // double y = seg_y + d*sin(perp_heading);
  double y = C.data()[0] + C.data()[1]*x + C.data()[2]*x*x;

  return {x,y};

}

vector<double> getSplineWaypoints(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;
  // double x1, x2, x3, y1, y2, y3;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  // int wp0 = (prev_wp+i-2)%maps_x.size();
  // int wp1 = prev_wp;
  // int wp2 = (prev_wp+1)%maps_x.size();
  // int wp3 = (prev_wp+2)%maps_x.size();

  vector<double> points;
  for (int i=0; i<8; i++){
    points.push_back(maps_s[(prev_wp+i-3)%maps_x.size()]);
  }
  for (int i=0; i<8; i++){
    points.push_back(maps_x[(prev_wp+i-3)%maps_x.size()]);
  }  
  for (int i=0; i<8; i++){
    points.push_back(maps_y[(prev_wp+i-3)%maps_x.size()]);
  }
  // vector<double> points = {maps_s[wp0], maps_s[wp1], maps_s[wp2], maps_s[wp3], maps_x[wp0], maps_x[wp1], maps_x[wp2], maps_x[wp3], maps_y[wp0],maps_y[wp1],maps_y[wp2],maps_y[wp3]};

  return points;

}


int main() {
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

  PathPlanner pp;

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

  int lane;

  h.onMessage([&lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
            ValueArray nextVals;

            int nextwp = NextWaypoint(car_x, car_y, deg2rad(car_yaw), map_waypoints_x, map_waypoints_y);

            if (car_d < 4){
              lane = 0;
            }
            else if (car_d < 8){
              lane = 1;
            }
            else {
              lane = 2;
            }
            nextwp++;

            double distance = car_s + .2; //map_waypoints_s[nextwp] - car_s;
            double v_des = 10;

            vector<double> start = {car_s, v_des, 0};
            vector<double> end = {distance, v_des, 0};
            vector<double> resultJMT = JMT(start, end, .02);

            double t;
            double wp_s = car_s;
            double wp_x, wp_y;
            vector<double> nextXY = getXY(wp_s, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> nextWPs = getSplineWaypoints(wp_s, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> S(8), X(8), Y(8);
            for (int i=0; i<8; i++){
              S[i] = nextWPs[i];
              X[i] = nextWPs[i+8];
              Y[i] = nextWPs[i+16]-6;
            }
             tk::spline sX;
             tk::spline sY;
             sX.set_points(S,X);
             sY.set_points(S,Y);  
             double nextX, nextY;

            for (int i=0; i<160; i++){
              t = i*.02;
              wp_s = (car_s + resultJMT[1]*t + resultJMT[3]*pow(t,3) + resultJMT[4]*pow(t,4) + resultJMT[5]*pow(t,5)); 
              
              // wp_s = wp_s + .15;
              // nextXY = getXY(wp_s, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              nextX = sX(wp_s);
              nextY = sY(wp_s);
              // std::cout << nextXY[1] << " " << nextY << endl; //nextXYs[0] << " " << nextXYs[1] << " " << nextXYs[2] << " " << nextXYs[3] << endl;

              // std::cout << nextXY[1] << " " << nextXYs[1] << endl;
              next_x_vals.push_back(nextX);
              // next_y_vals.push_back(nextXY[1] - 6);
              next_y_vals.push_back(nextY);


            }

            // next_x_vals.push_back = 

            // std::cout << "end of path" << endl;
    // convert position to s and d coordinates
    // pick a point 10m ahead
    // plan a jerk/acceleration approved path to get there - find coordinates
    // convert to time based using dt

    // check for nearest car ahead
    // if slower than v_des, slow down
    // if really slow, plan a lane change
    // look for a break in traffic to the left
    // pass the car and move back to right
    // if stopped, don't hit it

              // std::cout << sensor_fusion.size() << std::endl;

              // next_x_vals = nextVals.next_x_vals;
              // next_y_vals = nextVals.next_y_vals;
            // }


            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
