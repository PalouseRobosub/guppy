// from proxsuite examples
#include <iostream>
#include <Eigen/Core>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <chrono>

using namespace proxsuite::proxqp;
using namespace std::chrono;


int main() {
	dense::isize motors = 8;

	Eigen::MatrixXd A = Eigen::MatrixXd(6, motors);
	A << 1, 1, 1, 1.5, 1, 2, 3, 4, \
		 4, 0, 0, 2,1, 2, 3, 4, \
		 4, 0, 0, 2,1, 2, 3, 4, \
		 4, 0, 0, 2, 3, 4, 5,\
		 4, 0, 0, 2,6, 7, 8, 9, \
		 3.5, 3, 3, 2.5, 1, 1, 1, 1;

	Eigen::MatrixXd b = Eigen::MatrixXd(6, 1);
	b << 10,
		 1,
		 0,
		 2,
		 3,
		 4;

	Eigen::MatrixXd H = A.transpose() * A;

	Eigen::VectorXd g = - ( A.transpose() * b );

	Eigen::MatrixXd C = Eigen::MatrixXd::Identity(motors, motors);
	Eigen::VectorXd l = Eigen::VectorXd(motors); l << -1, -1, -1, -1, -1, -1, -1, -1;
	Eigen::VectorXd u = Eigen::VectorXd(motors); u << 1, 1, 1, 1, 1, 1, 1, 1;
	dense::QP<double> qp(motors, 0, motors);

	qp.settings.eps_abs = 1e-2; // convergence amount
	qp.settings.initial_guess = InitialGuessStatus::NO_INITIAL_GUESS;
	qp.settings.verbose = false;

	qp.init(H, g, std::nullopt, std::nullopt, C, l, u);
	auto start = high_resolution_clock::now();
	qp.solve();
	auto stop = high_resolution_clock::now();

	auto duration = duration_cast<microseconds>(stop - start);
	std::cout << duration.count() << " us" << std::endl;

	std::cout << qp.results.x << std::endl;
	std::cout << "optimal z: " << qp.results.z << std::endl;
	return 0;
}
