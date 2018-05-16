#include <ensenso_nx/ensenso_nx_node.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ensenso_nx_node");
	ensenso_nx::EnsensoNxNode ensenso;
	ros::spin();
	return 0;
}