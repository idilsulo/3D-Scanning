#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function
struct SurfaceCostFunction
{

};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// Read 3D surface data points and define the parameters of the problem
	const std::string file_path = "../data/points_surface.txt";
	const auto points = read_points_from_file<Point3D>(file_path);
	
	const double a_initial = 1.0;
	const double b_initial = 1.0;
	const double c_initial = 1.0;
	
	double a = a_initial;
	double b = b_initial;
	double c = c_initial;
	
	ceres::Problem problem;

	// TODO: For each data point create one residual block
	

	ceres::Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;
	
	// Output the final values of the parameters
	std::cout << "Initial a: " << a_initial << "\tb: " << b_initial << "\tc: " << c_initial << std::endl;
	std::cout << "Final a: " << a << "\tb: " << b << "\tc: " << c << std::endl;

	system("pause");
	return 0;
}
