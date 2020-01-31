#include "RockOdometry.hpp"

#include <slam3d/core/Graph.hpp>
#include <boost/format.hpp>
using namespace slam3d;

RockOdometry::RockOdometry(const std::string& name, Graph* graph, Solver* solver, Logger* logger, transformer::Transformation& tf)
 : PoseSensor(name, graph, logger), mTransformation(tf), mSolver(solver)
{
	mLastVertex = 0;
	mLastOdometricPose = Transform::Identity();
	mCurrentOdometricPose = Transform::Identity();
	mGravityReference = Direction::UnitZ();
}

RockOdometry::~RockOdometry()
{
}

void RockOdometry::handleNewVertex(IdType vertex)
{
	// Add odometry transform to previous vertex
	if(mLastVertex > 0)
	{
		TransformWithCovariance twc;
		twc.transform = mLastOdometricPose.inverse() * mCurrentOdometricPose;
		twc.covariance = calculateCovariance(twc.transform);
		SE3Constraint::Ptr se3(new SE3Constraint(mName, twc));
		mGraph->addConstraint(mLastVertex, vertex, se3);
		mGraph->setCorrectedPose(vertex, mGraph->getVertex(mLastVertex).corrected_pose * twc.transform);
	}
	
	// Add a gravity vector to this vertex
	Eigen::Quaterniond state(mCurrentOdometricPose.rotation());
	Direction upVector = state.inverse() * Eigen::Vector3d::UnitZ();	
	GravityConstraint::Ptr grav(new GravityConstraint(mName, upVector, mGravityReference, Covariance<2>::Identity()));
	mGraph->addConstraint(vertex, 0, grav);
	
	mLastVertex = vertex;
	mLastOdometricPose = mCurrentOdometricPose;
}

Transform RockOdometry::getPose(timeval stamp)
{
	base::Time ts = base::Time::fromSeconds(stamp.tv_sec, stamp.tv_usec);
	getPose(ts);
	return mCurrentOdometricPose;
}

Eigen::Affine3d RockOdometry::getPose(base::Time t)
{
	Eigen::Affine3d affine;
	bool res;
	try
	{
		res = mTransformation.get(t, affine, false);
	}catch(std::exception& e)
	{
		throw InvalidPose(e.what());
	}
	
	if(!res)
	{
		transformer::TransformationStatus s = mTransformation.getStatus();
		throw InvalidPose(
			(boost::format("Transformation not available! [success: %1%, no chain: %2%, no sample: %3%, interpolation failed: %4%]")
			% s.generated_transformations
			% s.failed_no_chain
			% s.failed_no_sample
			% s.failed_interpolation_impossible).str());
	}
	
	if((affine.matrix().array() == affine.matrix().array()).all())
	{
		mCurrentOdometricPose.linear() = affine.linear();
		mCurrentOdometricPose.translation() = affine.translation();
	}else
	{
		throw InvalidPose("Odometry sample contained invalid data.");
	}
	
	return affine;
}

Covariance<6> RockOdometry::calculateCovariance(const Transform &tf)
{
	return Covariance<6>::Identity() * 100;
}
