#ifndef SLAM3D_ROCK_ODOMETRY_HPP
#define SLAM3D_ROCK_ODOMETRY_HPP

#include <slam3d/Odometry.hpp>
#include <transformer/Transformer.hpp>

namespace slam3d
{
	class RockOdometry : public Odometry
	{
	public:
		RockOdometry(transformer::Transformation& tf, Logger* logger);
		~RockOdometry();
		
		Transform getOdometricPose(timeval stamp);
		Eigen::Affine3d getOdometricPose(base::Time t);
		TransformWithCovariance getRelativePose(timeval last, timeval next);
		Covariance calculateCovariance(const Transform &tf);

	private:
		transformer::Transformation& mTransformation;
	};
}

#endif
