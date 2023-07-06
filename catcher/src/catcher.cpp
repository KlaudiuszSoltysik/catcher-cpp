#include "rclcpp/rclcpp.hpp"


class Catcher : public rclcpp::Node {
  public:
    Catcher() : Node("catcher") {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-------------------");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "| Catcher started |");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-------------------");
    }

  private:
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Catcher>());
  rclcpp::shutdown();
  return 0;
}