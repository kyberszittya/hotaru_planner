#include "interfaceros_hotaru_local_planner.h"

namespace hotaru {

InterfaceRos_Hotarulocalplanner::~InterfaceRos_Hotarulocalplanner() {}

bool InterfaceRos_Hotarulocalplanner::init()
{
	// Initialize state machines
	notifier = std::shared_ptr<rei::RosCommunicationGraphNotifier>(
						new rei::RosCommunicationGraphNotifier("hotaru_local_planner/sync_state/", nh));
	notifier->initialize();
	port_state_monitor= std::shared_ptr<rei::PortStateMonitorRos>(new rei::PortStateMonitorRos());
	std::unique_ptr<rei::RosSyncStateGuard> guard(new rei::RosSyncStateGuard());
	guard->setMonitor(port_state_monitor);
	sync_state_machine = std::shared_ptr<rei::SyncStateMachine>(
						new rei::SyncStateMachine(notifier,
								std::move(guard)));
	sync_sm_sync_state = std::shared_ptr<rei::RosSyncStateMachine>(
			new rei::RosSyncStateMachine(nh,
					sync_state_machine,
					port_state_monitor,
					notifier,
					"hotaru_local_planner/sync_state"));
	if (sync_sm_sync_state!=nullptr)
	{
		if (!sync_sm_sync_state->initialize())
		{
			return false;
		}
		//sync_sm_sync_state->addTopicGuard("/base_waypoints", 0.1);
		sync_sm_sync_state->addTopicGuard("/current_pose", 10.0);
		sync_sm_sync_state->addTopicGuard("/current_velocity", 10.0);
		sync_sm_sync_state->addTopicGuard("/closest_waypoint", 10.0);
	}
	else
	{
		return false;
	}
	/// Initialize internal pubsub state
	pubsubstate = std::unique_ptr<StateHotarulocalplanner>(new StateHotarulocalplanner());
	/// Initialize ROS publishers
	final_waypoints = nh->advertise<autoware_msgs::Lane>("final_waypoints", 10);
	pub_state_machine_output_signal = nh->advertise<rei_monitoring_msgs::ReiStateMachineTransitionSignal>("state_machine_signal", 10);
	/// Initialize ROS subscribers
	sub_base_waypoints = nh->subscribe("base_waypoints", 10, &InterfaceRos_Hotarulocalplanner::cbSub_base_waypoints, this);
	sub_replan_request_sig = nh->subscribe("replan_request_sig", 10, &InterfaceRos_Hotarulocalplanner::cbSub_replan_request_sig, this);
	sub_current_pose = nh->subscribe("current_pose", 10, &InterfaceRos_Hotarulocalplanner::cbSub_current_pose, this);
	sub_current_velocity = nh->subscribe("current_velocity", 10, &InterfaceRos_Hotarulocalplanner::cbSub_current_velocity, this);
	sub_filtered_obstacles = nh->subscribe("filtered_obstacles_poly", 10, &InterfaceRos_Hotarulocalplanner::cbSub_filtered_obstacles, this);
	sub_closest_waypoints = nh->subscribe("closest_waypoint", 10, &InterfaceRos_Hotarulocalplanner::cbSub_closest_waypoints, this);
	if (!initNode())
	{
		return false;
	}

	return true;
}

void InterfaceRos_Hotarulocalplanner::cbSub_base_waypoints(const autoware_msgs::Lane::ConstPtr& msg)
{
	pubsubstate->msg_sub_base_waypoints = *msg;
	// Synchronize with state machine: sync_sm_sync_state
	sm_mutex.lock();
	//sync_sm_sync_state->stepMessageTopic("/base_waypoints", msg->header);
	sm_mutex.unlock();
	//if (sync_sm_sync_state->isReady()){
		executeReconstructWaypoints();
	//}
	
}
void InterfaceRos_Hotarulocalplanner::cbSub_replan_request_sig(const rei_planner_signals::ReplanRequest::ConstPtr& msg)
{
	pubsubstate->msg_sub_replan_request_sig = *msg;
	executeReplanRequest();
}
void InterfaceRos_Hotarulocalplanner::cbSub_current_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	pubsubstate->msg_sub_current_pose = *msg;
	// Synchronize with state machine: sync_sm_sync_state
	sm_mutex.lock();
	sync_sm_sync_state->stepMessageTopic("/current_pose", msg->header);
	sm_mutex.unlock();
	if (sync_sm_sync_state->isReady()){
		executeSynchWithPose();
	}
	
}
void InterfaceRos_Hotarulocalplanner::cbSub_current_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	pubsubstate->msg_sub_current_velocity = *msg;
	// Synchronize with state machine: sync_sm_sync_state
	sm_mutex.lock();
	sync_sm_sync_state->stepMessageTopic("/current_velocity", msg->header);
	sm_mutex.unlock();
	executeUpdateVelocity();
	
}
void InterfaceRos_Hotarulocalplanner::cbSub_filtered_obstacles(const autoware_msgs::DetectedObjectArray::ConstPtr& msg)
{
	pubsubstate->msg_sub_filtered_obstacles = *msg;
	executeUpdateObstacles();
	
}

void InterfaceRos_Hotarulocalplanner::cbSub_closest_waypoints(const std_msgs::Int32::ConstPtr& msg)
{
	pubsubstate->msg_closest_waypoint = *msg;
	sm_mutex.lock();
	sync_sm_sync_state->stepMessageTopic("/closest_waypoint", ros::Time::now());
	sm_mutex.unlock();

	executeUpdateClosestWaypoint();
}

void InterfaceRos_Hotarulocalplanner::publishFinal_waypoints()
{
	final_waypoints.publish(pubsubstate->msg_final_waypoints);
}
void InterfaceRos_Hotarulocalplanner::publishState_machine_signal()
{
	pub_state_machine_output_signal.publish(pubsubstate->msg_pub_state_machine_output_signal);
}



}

