#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);
		
		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with  .block(start_row, start_col, num_rows, num_cols)= elements

		Matrix4f estimatedPose = Matrix4f::Identity();

        estimatedPose.block(0, 0, 3, 3) = rotation;
        estimatedPose.block(0, 3, 3, 1) = translation;
		estimatedPose(3,3) = 1;

        return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.

		Vector3f mean = Vector3f::Zero();
        auto N = points.size();

        for (int i = 0; i < N; i++) {
            mean[0] += points[i][0];
            mean[1] += points[i][1];
            mean[2] += points[i][2];
        }

        mean[0] /= N;
        mean[1] /= N;
        mean[2] /= N;

        return mean;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Important: The covariance matrices should contain mean-centered source/target points.

        Matrix3f rotation = Matrix3f::Identity();

        ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same.");
        int N = sourcePoints.size();

        MatrixXf X(3,N);
        MatrixXf Y(3,N);

        for (int i = 0; i < N; ++i) {
            // Mean-centering in order to remove translation
            Vector3f point_x;
            Vector3f point_y;
            point_x = sourcePoints[i] - sourceMean;
            X.block(0, i,3,1) = point_x;
            point_y = targetPoints[i] - targetMean;
            Y.block(0, i,3,1) = point_y;
        }


        auto X_Y_t = X * (Y.transpose());

        JacobiSVD<MatrixXf> svd(X_Y_t, ComputeThinU | ComputeThinV);
        svd.compute(X_Y_t);

        auto U = svd.matrixU();
        auto V = svd.matrixV();
        rotation = V * U.transpose();
        return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target points.
		
        Vector3f translation = Vector3f::Zero();

        // Recover the translation from the version where translation is removed
        translation = (targetMean - (rotation * sourceMean));
        return translation;
	}
};