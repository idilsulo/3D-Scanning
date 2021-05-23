#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function
struct RegistrationCostFunction
{
    RegistrationCostFunction(const Point2D& pointP_, const Point2D& pointQ_, const Weight& weight_)
        : pointP(pointP_), pointQ(pointQ_), weight(weight_)
    {
    }

    template<typename T>
    bool operator()(const T* const theta, const T* const t_x, const T* const t_y, T* error) const
    {
        /*
        Eigen::Vector2d p(pointP.x, pointP.y);
        Eigen::Vector2d q(pointQ.x, pointQ.y);
        Eigen::Matrix2d rotationMatrix;
        rotationMatrix(0, 0) = cos(theta[0]);
        rotationMatrix(0, 1) = (-1) * sin(theta[0]);
        rotationMatrix(1, 0) = sin(theta[0]);
        rotationMatrix(1, 1) = cos(theta[0]);
        Eigen::Vector2d translation(t_x[0], t_y[0]);
       // auto transformation;
        error[0] = weight.w * (rotationMatrix * p + translation - q).squaredNorm();
        */
        auto transformation_x = (cos(theta[0]) * pointP.x) - (sin(theta[0]) * pointP.y) + t_x[0] - pointQ.x;
        auto transformation_y = (sin(theta[0]) * pointP.x) + (cos(theta[0]) * pointP.y) + t_y[0] - pointQ.y;
        auto squaredNorm = pow(transformation_x, 2.0) + pow(transformation_y, 2.0);
        error[0] = weight.w * squaredNorm;
                return true;

    }

private:
    const Point2D pointP;
    const Point2D pointQ;
    const Weight  weight;
};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// Read data points and the weights, and define the parameters of the problem
	const std::string file_path_1 = "../../data/points_dragon_1.txt";
	const auto points1 = read_points_from_file<Point2D>(file_path_1);
	
	const std::string file_path_2 = "../../data/points_dragon_2.txt";
	const auto points2 = read_points_from_file<Point2D>(file_path_2);
	
	const std::string file_path_weights = "../../data/weights_dragon.txt";
	const auto weights = read_points_from_file<Weight>(file_path_weights);
	
	const double angle_initial = 0.0;
	const double tx_initial = 0.0;
	const double ty_initial = 0.0;
	
	double angle = angle_initial;
	double tx = tx_initial;
	double ty = ty_initial;

	ceres::Problem problem;

	// TODO: For each weighted correspondence create one residual block
	auto N = 0;
	auto sizePoints1 = points1.size();
    auto sizePoints2 = points2.size();
    auto sizeWeights = weights.size();
    if (sizePoints1 != sizePoints2 || sizeWeights != sizePoints1 || sizeWeights != sizePoints2)
    {
        N = std::min({sizePoints1, sizePoints2, sizeWeights});
    }
    else
    {
        N = sizePoints1;
    }

    for (int i = 0; i < N; i++)
    {
        problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<RegistrationCostFunction, 1, 1, 1, 1>(
                        new RegistrationCostFunction(points1[i], points2[i], weights[i])
                ),
                nullptr, &angle, &tx, &ty
        );
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
