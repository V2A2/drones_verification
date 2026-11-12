#include "as2_core/core_functions.hpp"
#include "collision_avoidance_behavior/go_to_collision_avoidance_behavior.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GoToCollisionAvoidanceBehavior>();
  node->preset_loop_frequency(30);
  as2::spinLoop(node);

  rclcpp::shutdown();
  return 0;
}
