/*
 * test_dare.cpp
 *
 *  Created on: Sep 14, 2020
 *      Author: kyberszittya
 */


#include <gtest/gtest.h>

#include <hotaru_planner_node_blocks/algorithms/dare_calculations.hpp>


TEST(TestBasicCalculations, DareCalculation)
{
	Eigen::MatrixXd A(2,2);
	A << -0.9, -0.3,
			0.7, 0.1;
	Eigen::VectorXd B(2);
	B << 1, 1;
	Eigen::MatrixXd Q(2,2);
	Q << 1, 0,
		0, 3;
	double R = 0.1;
	Eigen::MatrixXd X = hotaru::algorithm::dareCalculation(A, B, Q, R);
	std::cout << "Resulting matrix: " << X << '\n';
	ASSERT_NEAR(X.coeff(0,0), 4.76776,  0.0001);
	ASSERT_NEAR(X.coeff(0,1), 0.943602, 0.0001);
	ASSERT_NEAR(X.coeff(1,0), 0.943602, 0.0001);
	ASSERT_NEAR(X.coeff(1,1), 3.23687,  0.0001);
}


int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
