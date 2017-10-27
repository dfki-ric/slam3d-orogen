#include "RockOdometry.hpp"
#include <boost/format.hpp>
using namespace slam3d;

RockOdometry::RockOdometry(transformer::Transformation& tf, Logger* logger)
 : Odometry(logger), mTransformation(tf)
{
}

RockOdometry::~RockOdometry()
{
}

Transform RockOdometry::getOdometricPose(timeval stamp)
{
	base::Time ts = base::Time::fromSeconds(stamp.tv_sec, stamp.tv_usec);
	Eigen::Affine3d affine = getOdometricPose(ts);
	
	Transform tf;
	if((affine.matrix().array() == affine.matrix().array()).all())
	{
		tf.linear() = affine.linear();
		tf.translation() = affine.translation();
	}else
	{
		mLogger->message(ERROR, "Odometry sample contained invalid data!");
		throw OdometryException();
	}
	return tf;
}

Eigen::Affine3d RockOdometry::getOdometricPose(base::Time t)
{
	Eigen::Affine3d odom;
	bool res;
	try
	{
		res = mTransformation.get(t, odom, false);
	}catch(std::exception& e)
	{
		mLogger->message(ERROR, e.what());
		throw OdometryException();
	}
	
	if(!res)
	{
		transformer::TransformationStatus s = mTransformation.getStatus();
		mLogger->message(ERROR, (boost::format("Transformation not available! [success: %1%, no chain: %2%, no sample: %3%, interpolation failed: %4%]") % s.generated_transformations % s.failed_no_chain % s.failed_no_sample % s.failed_interpolation_impossible).str());
		throw OdometryException();
	}
	
	return odom;
}

TransformWithCovariance RockOdometry::getRelativePose(timeval last, timeval next)
{
	TransformWithCovariance twc;
	twc.transform = Transform::Identity();
	twc.covariance = Covariance::Identity();
	return twc;
}

Covariance RockOdometry::calculateCovariance(const Transform &tf)
{
	return Covariance::Identity() * 100;
}
