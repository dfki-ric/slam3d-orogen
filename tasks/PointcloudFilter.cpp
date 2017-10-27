#include "PointcloudFilter.hpp"

#include <base-logging/Logging.hpp>
#include <base/samples/Pointcloud.hpp>

using namespace slam3d;

PointcloudFilter::PointcloudFilter(std::string const& name, TaskCore::TaskState initial_state)
 : PointcloudFilterBase(name, initial_state)
{
}

PointcloudFilter::PointcloudFilter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
 : PointcloudFilterBase(name, engine, initial_state)
{
}

PointcloudFilter::~PointcloudFilter()
{
}

bool PointcloudFilter::configureHook()
{
	if (! PointcloudFilterBase::configureHook())
		return false;
	
	mMinHeight = _min_height.get();
	mMaxHeight = _max_height.get();
	mSqMinDistance = _min_distance.get() * _min_distance.get();
	mSqMaxDistance = _max_distance.get() * _max_distance.get();
	mPassRate = _pass_rate.get();
	mSkipCount = 0;
	return true;
}

bool PointcloudFilter::startHook()
{
	if (! PointcloudFilterBase::startHook())
		return false;
	return true;
}

void PointcloudFilter::updateHook()
{
	PointcloudFilterBase::updateHook();
	
	// Read the scan from the port
	base::samples::Pointcloud cloud;
	while(_input.read(cloud, false) == RTT::NewData)
	{
		mSkipCount++;
		if(mSkipCount < mPassRate)
		{
			continue;
		}
		mSkipCount = 0;
		base::samples::Pointcloud filtered_cloud;
		for(std::vector<base::Vector3d>::const_iterator it = cloud.points.begin(); it < cloud.points.end(); ++it)
		{
			double sq_dist = ((*it)[0] * (*it)[0]) + ((*it)[1] * (*it)[1]) + ((*it)[2] * (*it)[2]);
			if((*it)[2] < mMaxHeight && (*it)[2] > mMinHeight && sq_dist < mSqMaxDistance && sq_dist > mSqMinDistance)
				filtered_cloud.points.push_back(*it);
		}
		filtered_cloud.time = cloud.time;
		_output.write(filtered_cloud);
	}
}

void PointcloudFilter::errorHook()
{
	PointcloudFilterBase::errorHook();
}

void PointcloudFilter::stopHook()
{
	PointcloudFilterBase::stopHook();
}

void PointcloudFilter::cleanupHook()
{
	PointcloudFilterBase::cleanupHook();
}
