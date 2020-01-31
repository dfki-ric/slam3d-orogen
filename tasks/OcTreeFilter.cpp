#include "OcTreeFilter.hpp"
#include "BaseLogger.hpp"
#include "Common.hpp"

#include <base/samples/Pointcloud.hpp>
#include <slam3d/core/FileLogger.hpp>

#include <pcl/filters/voxel_grid.h>
#include <boost/format.hpp>

using namespace slam3d;

octomap::OcTree* initOcTree(const OctoMapConfiguration &conf);

OcTreeFilter::OcTreeFilter(std::string const& name)
 : OcTreeFilterBase(name)
{
}

OcTreeFilter::OcTreeFilter(std::string const& name, RTT::ExecutionEngine* engine)
 : OcTreeFilterBase(name, engine)
{
}

OcTreeFilter::~OcTreeFilter()
{
}

bool OcTreeFilter::setLog_level(boost::int32_t value)
{
	switch(value)
	{
	case 4:
		mLogger->setLogLevel(FATAL);
		break;
	case 3:
		mLogger->setLogLevel(ERROR);
		break;
	case 2:
		mLogger->setLogLevel(WARNING);
		break;
	case 0:
		mLogger->setLogLevel(DEBUG);
		break;
	default:
		mLogger->setLogLevel(INFO);
	}
	return true;
}

PointCloud::Ptr OcTreeFilter::downsample(PointCloud::Ptr source, float leaf_size)
{
	PointCloud::Ptr downsampled(new PointCloud);
	pcl::VoxelGrid<PointType> grid;
	grid.setLeafSize (leaf_size, leaf_size, leaf_size);
	grid.setInputCloud(source);
	grid.filter(*downsampled);
	return downsampled;
}

void OcTreeFilter::inputTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &input_sample)
{
	// Get laser pose
	timeval start = mClock->now();
	Eigen::Affine3d laser2odom;
	try
	{
		_laser2odometry.get(ts, laser2odom, true);
	}catch(std::exception &e)
	{
		mLogger->message(WARNING, e.what());
	}
	
	// Add to accumulated cloud and octree
	octomap::Pointcloud octoCloud;
	PointCloud::Ptr t1(new PointCloud);
	for(std::vector<base::Vector3d>::const_iterator it = input_sample.points.begin(); it < input_sample.points.end(); ++it)
	{
		double sq_dist = ((*it)[0] * (*it)[0]) + ((*it)[1] * (*it)[1]) + ((*it)[2] * (*it)[2]);
		if((*it)[2] < mMaxHeight && (*it)[2] > mMinHeight && sq_dist < mSqMaxDistance && sq_dist > mSqMinDistance)
		{
			t1->push_back(PointType((*it)[0], (*it)[1], (*it)[2]));
		}
	}
	
	PointCloud::Ptr t2 = downsample(t1, mOctoConfig.resolution);
	for(PointCloud::iterator p = t2->begin(); p != t2->end(); ++p)
	{
		base::Vector3d v = laser2odom * base::Vector3d(p->x, p->y, p->z);
		mPointcloud.push_back(PointType(v[0], v[1], v[2]));
		octoCloud.push_back(octomap::point3d(v[0], v[1], v[2]));
	}
	
	mOcTree->insertPointCloud(octoCloud, octomap::point3d(), mOctoConfig.rangeMax, true, true);
	
	// Accumulate scans before updating
	mScanCount++;
	if(mScanCount < mPassRate)
	{
		mLogger->message(DEBUG, (boost::format("Added scan in %1% ms.") % timevaldiff(start, mClock->now())).str());
		return;
	}
	mScanCount = 0;
	
	// Check each point, if it is in free OctoMap voxel
	PointCloud::Ptr filtered_cloud(new PointCloud);
	for(PointCloud::iterator p = mPointcloud.begin(); p != mPointcloud.end(); ++p)
	{
		octomap::OcTreeNode* node = mOcTree->search(p->x, p->y, p->z);
		if(!node || mOcTree->isNodeOccupied(node))
		{
			filtered_cloud->push_back(*p);
		}
	}
	
	// Downsample
	PointCloud::Ptr downsampled = downsample(filtered_cloud, mOctoConfig.resolution);
	
	// Transform to current pose
	base::samples::Pointcloud result;
	for(PointCloud::iterator p = downsampled->begin(); p != downsampled->end(); ++p)
	{
		base::Vector3d vec = laser2odom.inverse() * base::Vector3d(p->x, p->y, p->z);
		result.points.push_back(vec);
	}
	result.time = ts;
	_output.write(result);

	// Write octree binary to view with 'octovis'
	if(_log_level.get() == 0)
	{
		mOcTree->writeBinary("slam3d_filter_result.bt");
	}
	
	// Reset everything
	delete mOcTree;
	mOcTree = initOcTree(mOctoConfig);
	mPointcloud.clear();
	mLogger->message(DEBUG, (boost::format("Created output in %1% ms.") % timevaldiff(start, mClock->now())).str());
}

bool OcTreeFilter::configureHook()
{
	if (! OcTreeFilterBase::configureHook())
		return false;
	
	mClock = new Clock();
	switch(_log_type)
	{
	case 0:
		mLogger = new Logger(*mClock);
		break;
	case 1:
		mLogger = new BaseLogger();
		break;
	case 2:
		mLogger = new FileLogger(*mClock, "slam3d_filter.log");
		break;
	default:
		mLogger = new Logger(*mClock);
		mLogger->message(WARNING, "Invalid logger type, using standard logger.");
	}

	setLog_level(_log_level);
	
	mMinHeight = _min_height.get();
	mMaxHeight = _max_height.get();
	mSqMinDistance = _min_distance.get() * _min_distance.get();
	mSqMaxDistance = _max_distance.get() * _max_distance.get();
	mOctoConfig = _octo_map_config.get();
	mPassRate = _pass_rate.get();
	mOcTree = initOcTree(mOctoConfig);
	
	mScanCount = 0;
	return true;
}

bool OcTreeFilter::startHook()
{
	if (! OcTreeFilterBase::startHook())
		return false;
	return true;
}

void OcTreeFilter::updateHook()
{
	OcTreeFilterBase::updateHook();
}

void OcTreeFilter::errorHook()
{
	OcTreeFilterBase::errorHook();
}

void OcTreeFilter::stopHook()
{
	OcTreeFilterBase::stopHook();
}

void OcTreeFilter::cleanupHook()
{
	delete mLogger;
	delete mClock;
	OcTreeFilterBase::cleanupHook();
}
