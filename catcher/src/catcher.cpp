#include <cstdlib>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "geometry_msgs/msg/twist.hpp"


class Catcher : public rclcpp::Node {
  public:
    Catcher() : Node("catcher") {
      // CONNECT WITH SPAWN SERVICE
      spawn_cli = this->create_client<turtlesim::srv::Spawn>("spawn");

      while(!spawn_cli->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service \"spawn\" not available, waiting again...");
      }

      // SPAWN FROG
      send_spawn_frog_req();

      // CONNECT WITH SET_PEN SERVICE
      set_pen_frog_cli = this->create_client<turtlesim::srv::SetPen>("frog1/set_pen");

      while(!set_pen_frog_cli->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service \"set_pen\" not available, waiting again...");
      }

      // TURN OFF FROG PEN
      send_turn_off_frog_pen_req();

      // CONNECT WITH TELEPORT SERVICE
      teleport_cli = this->create_client<turtlesim::srv::TeleportAbsolute>("frog1/teleport_absolute");

      while(!teleport_cli->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service \"teleport_absolute\" not available, waiting again...");
      }

      // TELEPORT FROG
      turtle_pose.x = 5.54;
      turtle_pose.y = 5.54;
      send_teleport_frog_req();

      // CREATE SUBSCRIBER TO POSE TOPIC
      turtle_pose_sub = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&Catcher::turtle_pose_sub_callback, this, std::placeholders::_1));

      // CREATE PUBLISHER ON CMD_VEL TOPIC
      turtle_pose_pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

      // RUN GAME LOOP FUNCTION
      timer = this->create_wall_timer(std::chrono::seconds(1 / 30), std::bind(&Catcher::main_loop, this));      

      // NOTIFY USER
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-------------------");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "| Catcher started |");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-------------------");
    }

  private:
    // VARIABLES
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_cli;

    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_frog_cli;

    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_cli;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_sub;
    turtlesim::msg::Pose turtle_pose;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_pose_pub;

    rclcpp::TimerBase::SharedPtr timer;

    double frog_x;
    double frog_y;

    double goal_theta;

    double delta_x = 0;
    double old_delta_x = 0;
    
    double delta_theta = 0;
    double old_delta_theta = 0;

    double I_x = 0;
    double I_theta = 0;

    void send_spawn_frog_req() {
      auto req = std::make_shared<turtlesim::srv::Spawn::Request>();
      
      req->x = 1.0;
      req->y = 1.0;
      req->theta = 0.0;
      req->name = "frog1";

      spawn_cli->async_send_request(req);
    }

    void send_turn_off_frog_pen_req() {
      auto req = std::make_shared<turtlesim::srv::SetPen::Request>();
      
      req->off = 1;

      set_pen_frog_cli->async_send_request(req);
    }

    void send_teleport_frog_req() {
      std::srand(static_cast<unsigned int>(std::time(nullptr)));

      frog_x = (std::rand() % 101) / 10 + 0.5;
      frog_y = (std::rand() % 101) / 10 + 0.5;
      double frog_theta = (std::rand() % 629) / 100;

      auto req = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
      
      req->x = frog_x;
      req->y = frog_y;
      req->theta = frog_theta;

      teleport_cli->async_send_request(req);

      goal_theta = std::atan2(frog_y - turtle_pose.y, frog_x - turtle_pose.x);
    }

    void turtle_pose_sub_callback(const turtlesim::msg::Pose & msg) {
      turtle_pose = msg;
    }

    void main_loop() {
      old_delta_x = delta_x;
      old_delta_theta = delta_theta;

      delta_x = frog_x - turtle_pose.x;
      delta_theta = goal_theta - turtle_pose.theta;
      
      double P_x = delta_x;
      double P_theta = delta_theta;

      I_x = I_x + (1 / 30) * delta_x;
      I_theta = I_theta + (1 / 30) * delta_theta;

      double D_x = (delta_x - old_delta_x) / (1 / 30.0);
      double D_theta = (delta_theta - old_delta_theta) / (1 / 30.0);

      auto msg = geometry_msgs::msg::Twist();
      
      msg.linear.x = 1 * P_x + 1 * I_x + D_x;
      msg.angular.z = 0.1 * P_theta + 0.1 * I_theta + 0.1 * D_theta;

      turtle_pose_pub->publish(msg);

      if(sqrt(pow(delta_x, 2) + pow(frog_y - turtle_pose.y, 2)) < 0.05) {
        send_teleport_frog_req();

        I_x = 0;
        I_theta = 0;
      }
    }
};


// class PID_controller {
//   public:
//     PID_controller(double Kp, double Ki, double Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {};

//     double Kp;
//     double Ki;
//     double Kd;
//     double old_feedback;

//     double control(double feedback) {
//       control = 0.0;

//       old_feedback = feedback;

//       return control;
//     }
// }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Catcher>());
  rclcpp::shutdown();
  return 0;
}