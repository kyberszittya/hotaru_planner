/*
 * test_hybridcontrolnode.cpp
 *
 *  Created on: Sep 2, 2020
 *      Author: kyberszittya
 */


#include "rei_construct_elements/rei_hybrid_control_node_ros1.hpp"

#include <gtest/gtest.h>

/*
class DummyHybridControlNode_Ros1: public rei::node::HybridControlNodeRos1
{
private:
public:
	~DummyHybridControlNode_Ros1();
};

TEST(BasicHybridControlTest, BasicControl)
{
	ros::NodeHandle n;
	DummyHybridControlNode_Ros1 control_node;
}
*/

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

