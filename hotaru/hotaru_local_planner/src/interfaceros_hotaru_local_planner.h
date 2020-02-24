#ifndef HOTARU_LOCAL_PLANNER_HEADER_HPP
#define HOTARU_LOCAL_PLANNER_HEADER_HPP

#include <ros/ros.h>
/// ROS msgs
#include <geometry_msgs/PoseStamped.h>		
#include <geometry_msgs/TwistStamped.h>		
#include <rei_planner_signals/ReplanRequest.h>		
#include <autoware_msgs/Lane.h>		
#include <rei_monitoring_msgs/ReiStateMachineTransitionSignal.h>		
#include <visualization_msgs/MarkerArray.h>		
#include <rei_statemachine_library/ros/ros_sync_state_machine.hpp>

#include <memory>

namespace hotaru {

struct StateHotarulocalplanner
{
	autoware_msgs::Lane msg_sub_base_waypoints; ///< sub_base_waypoints store to autoware_msgs/Lane
	rei_planner_signals::ReplanRequest msg_sub_replan_request_sig; ///< sub_replan_request_sig store to rei_planner_signals/ReplanRequest
	geometry_msgs::PoseStamped msg_sub_current_pose; ///< sub_current_pose store to geometry_msgs/PoseStamped
	geometry_msgs::TwistStamped msg_sub_current_velocity; ///< sub_current_velocity store to geometry_msgs/TwistStamped
	visualization_msgs::MarkerArray msg_sub_filtered_obstacles; ///< sub_filtered_obstacles store to visualization_msgs/MarkerArray
	/// ROS Publishers
	autoware_msgs::Lane msg_final_waypoints; ///< final_waypoints store to autoware_msgs/Lane
	rei_monitoring_msgs::ReiStateMachineTransitionSignal msg_pub_state_machine_output_signal; ///< pub_state_machine_output_signal store to rei_monitoring_msgs/ReiStateMachineTransitionSignal
};

/**
 *
 * @attribute sub_base_waypoints: subscribes to topic base_waypoints 
 * @attribute sub_replan_request_sig: subscribes to topic replan_request_sig 
 * @attribute sub_current_pose: subscribes to topic current_pose 
 * @attribute sub_current_velocity: subscribes to topic current_velocity 
 * @attribute sub_filtered_obstacles: subscribes to topic filtered_obstacles 
 * @attribute final_waypoints: publishes to topic final_waypoints
 * @attribute pub_state_machine_output_signal: publishes to topic state_machine_signal
 */
class InterfaceRos_Hotarulocalplanner
{
private:
protected:
	/// ROS utils
	std::shared_ptr<ros::NodeHandle> nh;
	/// ROS Subscribers
	ros::Subscriber sub_base_waypoints; ///< sub_base_waypoints subscriber to autoware_msgs/Lane
	ros::Subscriber sub_replan_request_sig; ///< sub_replan_request_sig subscriber to rei_planner_signals/ReplanRequest
	ros::Subscriber sub_current_pose; ///< sub_current_pose subscriber to geometry_msgs/PoseStamped
	ros::Subscriber sub_current_velocity; ///< sub_current_velocity subscriber to geometry_msgs/TwistStamped
	ros::Subscriber sub_filtered_obstacles; ///< sub_filtered_obstacles subscriber to visualization_msgs/MarkerArray
	/// ROS Publishers
	ros::Publisher final_waypoints; ///< final_waypoints publisher to autoware_msgs/Lane
	ros::Publisher pub_state_machine_output_signal; ///< pub_state_machine_output_signal publisher to rei_monitoring_msgs/ReiStateMachineTransitionSignal
	std::unique_ptr<StateHotarulocalplanner> pubsubstate;
	// State machines
	/** TODO: provide it in code generator component
	 *
	 */
	std::shared_ptr<rei::RosSyncStateMachine> sync_sm_sync_state;
	std::shared_ptr<rei::SyncStateMachine> sync_state_machine;
	std::shared_ptr<rei::PortStateMonitorRos> port_state_monitor;
	std::shared_ptr<rei::RosCommunicationGraphNotifier> notifier;
public:
	InterfaceRos_Hotarulocalplanner(std::shared_ptr<ros::NodeHandle> nh): nh(nh){}
	
	virtual ~InterfaceRos_Hotarulocalplanner() = 0;
	/*
	 * @brief: Initializes ROS middleware connection (pub/sub) interfaces
	 * @returns: Initialization successful
	 */
	bool init();
	/*
	 * @brief: descendant specific initialization 
	 */
	virtual bool initNode() = 0;
	
	/**
	 * Callback method for base_waypoints
	 */
	void cbSub_base_waypoints(const autoware_msgs::Lane::ConstPtr& msg); ///< sub_base_waypoints subscriber to autoware_msgs/Lane
	virtual void executeReconstructWaypoints() = 0;
	/**
	 * Callback method for replan_request_sig
	 */
	void cbSub_replan_request_sig(const rei_planner_signals::ReplanRequest::ConstPtr& msg); ///< sub_replan_request_sig subscriber to rei_planner_signals/ReplanRequest
	virtual void executeReplanRequest() = 0;
	/**
	 * Callback method for current_pose
	 */
	void cbSub_current_pose(const geometry_msgs::PoseStamped::ConstPtr& msg); ///< sub_current_pose subscriber to geometry_msgs/PoseStamped
	virtual void executeSynchWithPose() = 0;
	/**
	 * Callback method for current_velocity
	 */
	void cbSub_current_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg); ///< sub_current_velocity subscriber to geometry_msgs/TwistStamped
	/**
	 * Callback method for filtered_obstacles
	 */
	void cbSub_filtered_obstacles(const visualization_msgs::MarkerArray::ConstPtr& msg); ///< sub_filtered_obstacles subscriber to visualization_msgs/MarkerArray
	virtual void executeUpdateObstacles() = 0;
	
	/**
	 * Publish method to publish message to final_waypoints
	 */
	void publishFinal_waypoints();
	/**
	 * Publish method to publish message to state_machine_signal
	 */
	void publishState_machine_signal();
};

}

#endif

