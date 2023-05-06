#include "dllnode.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor exec;	
	// Particle filter instance
	auto dll_node = std::make_shared<DLLNode>("dll_node");
	exec.add_node(dll_node);
  	exec.spin();
	// Process data at given rate
	rclcpp::shutdown();
	return 0;
} 




