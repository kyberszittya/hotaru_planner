/*
 * tes_port_state_machine.cpp
 *
 *  Created on: Feb 20, 2020
 *      Author: kyberszittya
 */


#include <rei_statemachine_library/testing/test_common.hpp>

constexpr long freq_port_1 = 100*1000000;
constexpr long freq_port_2 = 200*1000000;
constexpr long ref_time_step =   1000000;





TEST(PortMonitorTest, StepOneMessageReceived)
{
	using namespace rei::test_sync_state_machine;
	DummyPortStateMonitor port_state_monitor;
	port_state_monitor.addPort("vehicle_cmd", freq_port_1);
	port_state_monitor.addPort("speed", freq_port_2);
	ASSERT_FALSE(port_state_monitor.isReady());
	port_state_monitor.updateTimestamp("vehicle_cmd", ref_time_step);
	ASSERT_FALSE(port_state_monitor.isReady());
}

TEST(PortMonitorTest, StepAllMessageReceivedSingleStep)
{
	using namespace rei::test_sync_state_machine;
	DummyPortStateMonitor port_state_monitor;
	port_state_monitor.addPort("vehicle_cmd", freq_port_1);
	port_state_monitor.addPort("speed", freq_port_2);
	ASSERT_FALSE(port_state_monitor.isReady());
	port_state_monitor.updateTimestamp("vehicle_cmd", ref_time_step);
	port_state_monitor.updateTimestamp("speed", 2*ref_time_step);
	ASSERT_TRUE(port_state_monitor.isReady());
}

TEST(PortMonitorTest, Step2MessageReceivedTimeout)
{
	using namespace rei::test_sync_state_machine;
	DummyPortStateMonitor port_state_monitor;
	port_state_monitor.addPort("vehicle_cmd", freq_port_1);
	port_state_monitor.addPort("speed", freq_port_2);
	ASSERT_FALSE(port_state_monitor.isReady());
	port_state_monitor.updateTimestamp("vehicle_cmd", ref_time_step);
	port_state_monitor.updateTimestamp("speed", 2*ref_time_step);
	ASSERT_TRUE(port_state_monitor.isReady());
	port_state_monitor.updateTimestamp("vehicle_cmd", 150*ref_time_step);
	std::cout << "--------\n";
	ASSERT_FALSE(port_state_monitor.isReady());
}

TEST(PortMonitorTest, Step2MessageReceivedNoTimeout)
{
	using namespace rei::test_sync_state_machine;
	DummyPortStateMonitor port_state_monitor;
	port_state_monitor.addPort("vehicle_cmd", freq_port_1);
	port_state_monitor.addPort("speed", freq_port_2);
	ASSERT_FALSE(port_state_monitor.isReady());
	port_state_monitor.updateTimestamp("vehicle_cmd", ref_time_step);
	port_state_monitor.updateTimestamp("speed", 10*ref_time_step);
	ASSERT_TRUE(port_state_monitor.isReady());
	port_state_monitor.updateTimestamp("vehicle_cmd", 50*ref_time_step);
	port_state_monitor.updateTimestamp("speed", 70*ref_time_step);
	ASSERT_TRUE(port_state_monitor.isReady());
}

TEST(PortMonitorTest, Step2MessageTimeoutRecover)
{
	using namespace rei::test_sync_state_machine;
	DummyPortStateMonitor port_state_monitor;
	port_state_monitor.addPort("vehicle_cmd", freq_port_1);
	port_state_monitor.addPort("speed", freq_port_2);
	ASSERT_FALSE(port_state_monitor.isReady());
	port_state_monitor.updateTimestamp("vehicle_cmd", ref_time_step);
	port_state_monitor.updateTimestamp("speed", 10*ref_time_step);
	ASSERT_TRUE(port_state_monitor.isReady());
	port_state_monitor.updateTimestamp("speed", 70*ref_time_step);
	port_state_monitor.updateTimestamp("vehicle_cmd", 150*ref_time_step);
	ASSERT_FALSE(port_state_monitor.isReady());
	port_state_monitor.updateTimestamp("vehicle_cmd", 170*ref_time_step);
	ASSERT_TRUE(port_state_monitor.isReady());
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
