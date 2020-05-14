#include <hotaru_planner_node_teb/gen_interfaceros_hotaruteblocalplannernode.h>

namespace hotaru {

InterfaceRos_HotaruTebLocalPlannerNode::~InterfaceRos_HotaruTebLocalPlannerNode() {}

bool InterfaceRos_HotaruTebLocalPlannerNode::initTimeoutStateMachine()
{
	// Initialize state machines 
	notifier = std::make_shared<rei::RosCommunicationGraphNotifier>("hotaruteblocalplannernode/sync_state/", nh);
	notifier->initialize();
	port_state_monitor= std::make_shared<rei::PortStateMonitorRos>();
	sync_guard = std::make_shared<rei::RosSyncStateGuard>();
	sync_guard->setMonitor(port_state_monitor);
	sync_state_machine = std::make_shared<rei::SyncStateMachine>(notifier, sync_guard);
	// Sync state machine initialization
	return true;
}

bool InterfaceRos_HotaruTebLocalPlannerNode::initMiddleware(const bool debug, const bool bypass_behavior)
{
	/// Initialize internal pubsub state
	pubsubstate = std::unique_ptr<StateHotaruTebLocalPlannerNode>(new StateHotaruTebLocalPlannerNode(debug, bypass_behavior));
	if (pubsubstate==nullptr)
	{
		return false;
	}
	/// Initialize ROS publishers
	/// Initialize ROS subscribers
	return true;
}





}
