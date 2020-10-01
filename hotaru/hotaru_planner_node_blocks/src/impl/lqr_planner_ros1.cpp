/*
 * lqr_planner_ros1.cpp
 *
 *  Created on: Sep 14, 2020
 *      Author: kyberszittya
 */

#include <hotaru_planner_node_blocks/algorithms/dare_calculations.hpp>
#include <hotaru_planner_node_blocks/blocks/hotaru_planner_node_blocks.hpp>

#include <hotaru_planner_msgs/Trajectory.h>

#include <ros/ros.h>

namespace hotaru
{

struct SystemModel
{
	Eigen::MatrixXd A;
	Eigen::VectorXd B;
};

class LqrPlanner: public blocks::PlannerAlgorithm
{
private:
	double dt;
	double max_time;
	double goal_dist;
	/// ROS interface
	ros::Subscriber sub_current_state;
	ros::Subscriber sub_goal_state;
protected:
	/// System model
	std::unique_ptr<SystemModel> model;
	/// LQR paameters
	Eigen::MatrixXd current_X;
	Eigen::MatrixXd current_Kgain;
public:
	LqrPlanner(std::shared_ptr<ros::NodeHandle> nh,
			std::unique_ptr<SystemModel> model,
			double dt, double max_time, double goal_dist):
		blocks::PlannerAlgorithm(nh),
		model(std::move(model)), dt(dt), max_time(max_time), goal_dist(goal_dist)
	{}


	bool lqr_planning(Eigen::VectorXd state, Eigen::VectorXd goal)
	{
		resultant_trajectory.waypoints.clear();
		double time = 0.0;

		Eigen::VectorXd x = state - goal;
		hotaru_planner_msgs::Waypoint wp;
		wp.pose = current_state;
		resultant_trajectory.waypoints.push_back(wp);
		while (time <= max_time)
		{
			hotaru_planner_msgs::Waypoint wp;
			time += dt;
			Eigen::VectorXd u = lqr_control(model->A, model->B, x);
			x = model->A * x + model->B * u;
			// Add as trajectory
			Eigen::VectorXd tr = x + goal;
			wp.pose.pose.position.x = tr.coeff(0);
			wp.pose.pose.position.y = tr.coeff(1);
			wp.pose.pose.orientation.w = 1.0;
			resultant_trajectory.waypoints.push_back(wp);
			Eigen::VectorXd n = x + goal;
			double d = (goal - n).norm();
			if (d < goal_dist)
			{
				return true;
			}

		}
		return false;
	}

	virtual bool calculateTrajectory(const geometry_msgs::PoseStamped& current_state,
			const geometry_msgs::PoseStamped& goal_state) override
	{
		Eigen::VectorXd _cs(2);
		_cs << current_state.pose.position.x, current_state.pose.position.y;
		Eigen::VectorXd _gs(2);
		_gs << goal_state.pose.position.x, goal_state.pose.position.y;
		return lqr_planning(_cs, _gs);
	}


	void dlqr(Eigen::MatrixXd A, Eigen::MatrixXd B,
			Eigen::MatrixXd Q, Eigen::MatrixXd R)
	{
		current_X = algorithm::dareCalculation(A, B, Q, R);
		double invK = (B.transpose() * current_X * B + R).inverse();
		current_Kgain =  invK * (B.transpose() * current_X * A);

	}

	Eigen::VectorXd lqr_control(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::VectorXd x)
	{
		dlqr(A, B, Eigen::MatrixXd::Identity(A.rows(), A.rows()), Eigen::MatrixXd(B.cols(), B.cols()));
		return -current_Kgain * x;
	}

	// TODO: get system model from shared source
	// TODO: get robot state from shared source

	/**
	 * @section ROS interface
	 */

	void initRosInterfaces(const double planner_timer_period)
	{
		init_ros_interface_timer(planner_timer_period);
		sub_current_state = nh->subscribe("/current_pose", 1, &LqrPlanner::cbCurrentPose, this);
		sub_goal_state = nh->subscribe("/goal_state", 1, &LqrPlanner::cbGoalState, this);
		planner_timer.start();
	}

	/**
		 * @fn void cbCurrentPose(const geometry_msgs::PoseStamped::ConstPtr&)
	 * @brief
	 *
	 * @pre
	 * @post
	 * @param pose
	 */
	void cbCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		current_state = *msg;
	}


	void cbGoalState(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		goal_state = *msg;
	}

};

} // namespace hotaru

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lqr_planner");
	std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
	// System model
	std::unique_ptr<hotaru::SystemModel> model = std::unique_ptr<hotaru::SystemModel>(new hotaru::SystemModel());
	double dt = 0.1;
	model->A = Eigen::MatrixXd(2,2);
	model->A << dt, 1,
			0, dt;
	model->B = Eigen::VectorXd(2);
	model->B << 0, 1;
	hotaru::LqrPlanner planner(nh, std::move(model), dt, 200.0, 0.1);
	Eigen::VectorXd goal =  Eigen::VectorXd(2);
	Eigen::VectorXd state = Eigen::VectorXd(2);
	//goal << 20,3;
	//state << 0, 0;
	planner.initRosInterfaces(0.05);
	ros::spin();
	return 0;
}


