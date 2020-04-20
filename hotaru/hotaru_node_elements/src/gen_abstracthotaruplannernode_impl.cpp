#include <hotaru_node_elements/gen_interfaceros_abstracthotaruplannernode.h>

namespace hotaru {

InterfaceRos_AbstractHotaruPlannerNode::~InterfaceRos_AbstractHotaruPlannerNode() {}

bool InterfaceRos_AbstractHotaruPlannerNode::initTimeoutStateMachine()
{
	// Initialize state machines 
	notifier = std::make_shared<rei::RosCommunicationGraphNotifier>("abstracthotaruplannernode/sync_state/", nh);
	notifier->initialize();
	port_state_monitor= std::make_shared<rei::PortStateMonitorRos>();
	sync_guard = std::make_shared<rei::RosSyncStateGuard>();
	sync_guard->setMonitor(port_state_monitor);
	sync_state_machine = std::make_shared<rei::SyncStateMachine>(notifier, sync_guard);
	// Sync state machine initialization
	sync_sm_planner_state = std::make_shared<rei::RosSyncStateMachine>(nh,
		sync_state_machine, port_state_monitor, notifier, 
		"AbstractHotaruPlannerNode/planner_state");
	if (sync_sm_planner_state!=nullptr)
	{
		if (!sync_sm_planner_state->initialize())
		{
			return false;
		}
		sync_sm_planner_state->addTopicGuard("/current_pose", 0.1+0.5);
		ROS_INFO("Synchronize with topic: current_pose, with estimated freq 10 Hz");
		sync_sm_planner_state->addTopicGuard("/current_velocity", 0.1+0.5);
		ROS_INFO("Synchronize with topic: current_velocity, with estimated freq 10 Hz");
		sync_sm_planner_state->addTopicGuard("/closest_waypoint", 0.1+0.0);
		ROS_INFO("Synchronize with topic: closest_waypoint, with estimated freq 10 Hz");
	}
	else
	{
		return false;
	}
	return true;
}

bool InterfaceRos_AbstractHotaruPlannerNode::initMiddleware(const bool debug)
{
	/// Initialize internal pubsub state
	pubsubstate = std::unique_ptr<StateAbstractHotaruPlannerNode>(new StateAbstractHotaruPlannerNode(debug));
	if (pubsubstate==nullptr)
	{
		return false;
	}
	/// Initialize ROS publishers
	port_refined_trajectory = nh->advertise<hotaru_msgs::RefinedTrajectory>("refined_trajectory", 10);
	port_calc_planner_time = nh->advertise<std_msgs::Float64>("calc_planner_time", 10);
	/// Initialize ROS subscribers
	port_input_trajectory = nh->subscribe("input_trajectory", 10, &InterfaceRos_AbstractHotaruPlannerNode::cbPort_input_trajectory, this);
	port_current_pose = nh->subscribe("current_pose", 10, &InterfaceRos_AbstractHotaruPlannerNode::cbPort_current_pose, this);
	port_current_velocity = nh->subscribe("current_velocity", 10, &InterfaceRos_AbstractHotaruPlannerNode::cbPort_current_velocity, this);
	port_replan_request_sig = nh->subscribe("replan_request_sig", 10, &InterfaceRos_AbstractHotaruPlannerNode::cbPort_replan_request_sig, this);
	port_closests_waypoint = nh->subscribe("closest_waypoint", 10, &InterfaceRos_AbstractHotaruPlannerNode::cbPort_closests_waypoint, this);
	port_poly_obstacle = nh->subscribe("rei_perception_monitor/detected_obstacles", 10, &InterfaceRos_AbstractHotaruPlannerNode::cbPort_poly_obstacle, this);
	return true;
}



void InterfaceRos_AbstractHotaruPlannerNode::cbPort_input_trajectory(const hotaru_msgs::RefinedTrajectory::ConstPtr& msg)
{
	pubsubstate->msg_port_input_trajectory = *msg;
	execute_update_input_trajectory();
	
}
void InterfaceRos_AbstractHotaruPlannerNode::cbPort_current_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	pubsubstate->msg_port_current_pose = *msg;
	// Synchronize with state machine: sync_sm_planner_state
	sm_mutex.lock();
	sync_sm_planner_state->stepMessageTopic("/current_pose", msg->header);
	sm_mutex.unlock();
	if (sync_sm_planner_state->isReady()){
		execute_update_current_pose();
	}
	
}
void InterfaceRos_AbstractHotaruPlannerNode::cbPort_current_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	pubsubstate->msg_port_current_velocity = *msg;
	// Synchronize with state machine: sync_sm_planner_state
	sm_mutex.lock();
	sync_sm_planner_state->stepMessageTopic("/current_velocity", msg->header);
	sm_mutex.unlock();
	if (sync_sm_planner_state->isReady()){
		executeUpdate_velocity();
	}
	
}
void InterfaceRos_AbstractHotaruPlannerNode::cbPort_replan_request_sig(const rei_planner_signals::ReplanRequest::ConstPtr& msg)
{
	pubsubstate->msg_port_replan_request_sig = *msg;
	
}
void InterfaceRos_AbstractHotaruPlannerNode::cbPort_closests_waypoint(const std_msgs::Int32::ConstPtr& msg)
{
	pubsubstate->msg_port_closests_waypoint = *msg;
	// Synchronize with state machine: sync_sm_planner_state
	sm_mutex.lock();
	sync_sm_planner_state->stepMessageTopic("/closest_waypoint", ros::Time::now());
	sm_mutex.unlock();
	if (sync_sm_planner_state->isReady()){
		executeUpdate_closest_waypoint();
	}
	
}
void InterfaceRos_AbstractHotaruPlannerNode::cbPort_poly_obstacle(const rei_monitoring_msgs::DetectedObstacles::ConstPtr& msg)
{
	pubsubstate->msg_port_poly_obstacle = *msg;
	executeUpdate_obstacles();
	
}

void InterfaceRos_AbstractHotaruPlannerNode::publishRefined_trajectory()
{
	port_refined_trajectory.publish(pubsubstate->msg_port_refined_trajectory);
}
void InterfaceRos_AbstractHotaruPlannerNode::publishCalc_planner_time()
{
	port_calc_planner_time.publish(pubsubstate->msg_port_calc_planner_time);
}

}

