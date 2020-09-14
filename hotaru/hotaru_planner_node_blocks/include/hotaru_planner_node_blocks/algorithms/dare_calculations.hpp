/*
 * dare_calculations.hh
 *
 *  Created on: Sep 14, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_ALGORITHMS_DARE_CALCULATIONS_HPP_
#define INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_ALGORITHMS_DARE_CALCULATIONS_HPP_


#include <Eigen/Dense>

namespace hotaru
{

namespace algorithm
{

constexpr double DARE_EPSILON = 0.01;

Eigen::MatrixXd dareCalculation(const Eigen::MatrixXd& A, const Eigen::VectorXd& B, const Eigen::MatrixXd& Q, double R,
		unsigned int MAX_ITER=1000)
{
	Eigen::MatrixXd X = Q;
	Eigen::MatrixXd Xn = Q;

	for (int i = 0; i < MAX_ITER; i++)
	{
		double inv_ = (B.transpose() * X * B + R);

		Xn = A.transpose() * X * A - A.transpose() * X * B * 1/inv_ * B.transpose() * X * A + Q;

		Eigen::MatrixXd err = (Xn - X).cwiseAbs();

		if (err.maxCoeff() < DARE_EPSILON)
		{
			break;
		}
		X = Xn;
	}
	return Xn;
}


} // namespace algorithm

} // namespace hotaru

#endif /* INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_ALGORITHMS_DARE_CALCULATIONS_HPP_ */
