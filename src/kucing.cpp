#include <ros/ros.h>
#include "kucing/libkucing.hpp"
using namespace ros;

int main(int argc, char** argv){
	init(argc, argv, "kucing");

	NodeHandle nodeHandle("~");

    kucing kucing(nodeHandle);

	spin();
	return 0;

}
