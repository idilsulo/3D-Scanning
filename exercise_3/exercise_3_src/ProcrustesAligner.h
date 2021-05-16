#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
	    std::cout << "source size: " << sourcePoints.size() << " target size: " << targetPoints.size() << std::endl;
		ASSERT((sourcePoints.size() == targetPoints.size()) && ("The number of source and target points should be the same, since every source point is matched with corresponding target point."));


		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);
		
		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements

		Matrix4f estimatedPose = Matrix4f::Identity();
		estimatedPose.block(0, 0, 3, 3) = rotation;
		estimatedPose.block(0, 3, 3, 1) = translation;
		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.

		Vector3f mean = Vector3f::Zero();
		int size = points.size();
		for (Vector3f point : points)
        {
		    mean += point;
        }
		mean /= size;
        return mean;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Important: The covariance matrices should contain mean-centered source/target points.


        ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same.");
        Matrix3f rotation = Matrix3f::Identity();
        Matrix3f crossCovariance = Matrix3f::Identity();

        int sizeSourcePoints = sourcePoints.size();
        MatrixXf X(sizeSourcePoints, 3);
        MatrixXf XHat(sizeSourcePoints, 3);
        for (int i = 0; i < sizeSourcePoints; i++)
        {
            Vector3f x_i = sourcePoints[i] - sourceMean;
            Vector3f xHat_i = targetPoints[i] - targetMean;

            X.block(i, 0, 1, 3) = x_i.transpose();
            XHat.block(i, 0, 1, 3) = xHat_i.transpose();
        }

        crossCovariance = X.transpose() * XHat;

        JacobiSVD<MatrixXf> svd(crossCovariance, ComputeThinU | ComputeThinV);
        svd.compute(crossCovariance);

        rotation = svd.matrixV() * svd.matrixU().transpose();
        Matrix3f mirror = Matrix3f::Identity();
        mirror(2, 2) = -1;
        if (rotation.determinant() == -1)
        {
            rotation = (svd.matrixV()) * mirror * (svd.matrixU().transpose());
        }
        return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target points.
        Vector3f translation = Vector3f::Zero();
        translation = (-1)*(rotation * sourceMean) + targetMean;
        return translation;
	}
};