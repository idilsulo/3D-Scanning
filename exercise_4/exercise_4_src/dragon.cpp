#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function
struct RegistrationCostFunction
{
    RegistrationCostFunction(const Point2D& _p, const Point2D& _q, const Weight& _weight): p(_p), q(_q), weight(_weight){}

    template<typename T>
    bool operator()(const T* const angle, const T* const tx, const T* const ty, T* residual) const{

        auto x_diff = (cos(angle[0]) * p.x) - (sin(angle[0]) * p.y) + tx[0] - q.x;
        auto y_diff = (sin(angle[0]) * p.x) + (cos(angle[0]) * p.y) + ty[0] - q.y;
        auto squaredNorm = pow(x_diff, 2.0) + pow(y_diff, 2.0);
        residual[0] = weight.w * squaredNorm;
        return true;

    }

private:
    const Point2D p;
    const Point2D q;
    const Weight weight;
};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// Read data points and the weights, and define the parameters of the problem
	const std::string file_path_1 = "../data/points_dragon_1.txt";
	const auto points1 = read_points_from_file<Point2D>(file_path_1);
	
	const std::string file_path_2 = "../data/points_dragon_2.txt";
	const auto points2 = read_points_from_file<Point2D>(file_path_2);
	
	const std::string file_path_weights = "../data/weights_dragon.txt";
	const auto weights = read_points_from_file<Weight>(file_path_weights);
	
	const double angle_initial = 0.0;
	const double tx_initial = 0.0;
	const double ty_initial = 0.0;
	
	double angle = angle_initial;
	double tx = tx_initial;
	double ty = ty_initial;

	ceres::Problem problem;

	// TODO: For each weighted correspondence create one residual block
    for(int i = 0 ; i < points1.size(); i++){
        ceres::CostFunction* cost_func = new ceres::AutoDiffCostFunction<RegistrationCostFunction,1,1,1,1>(new RegistrationCostFunction(points1[i],points2[i],weights[i]));
        problem.AddResidualBlock(cost_func, nullptr, &angle, &tx, &ty);
    }

	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	// Output the final values of the translation and rotation (in degree)
	std::cout << "Initial angle: " << angle_initial << "\ttx: " << tx_initial << "\tty: " << ty_initial << std::endl;
	std::cout << "Final angle: " << std::fmod(angle * 180 / M_PI, 360.0) << "\ttx: " << tx << "\tty: " << ty << std::endl;

	system("pause");
	return 0;
}
