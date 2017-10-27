#include "ScanConverter.hpp"

#include <velodyne_lidar/MultilevelLaserScan.h>
#include <velodyne_lidar/pointcloudConvertHelper.hpp>

#include <base-logging/Logging.hpp>
#include <base/samples/Pointcloud.hpp>

using namespace slam3d;

ScanConverter::ScanConverter(std::string const& name, TaskCore::TaskState initial_state)
    : ScanConverterBase(name, initial_state)
{
}

ScanConverter::ScanConverter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : ScanConverterBase(name, engine, initial_state)
{
}

ScanConverter::~ScanConverter()
{
}

void ScanConverter::copyPointCloud(const std::vector< Eigen::Vector3d >& pc_eigen, std::vector< base::Point >& pc_base) const
{
	pc_base.resize(pc_eigen.size());
	for(unsigned i = 0; i < pc_eigen.size(); i++)
		pc_base[i] = pc_eigen[i];
}

bool ScanConverter::configureHook()
{
	if (! ScanConverterBase::configureHook())
		return false;
	return true;
}

bool ScanConverter::startHook()
{
	if (! ScanConverterBase::startHook())
		return false;
	return true;
}

void ScanConverter::updateHook()
{
	ScanConverterBase::updateHook();

	if(_scan.connected() && _depth_map.connected())
	{
		LOG_ERROR("ScanConverter allows a connection to either scan or depth_map port");
		state(MULTIPLE_INPUT_CONNECTIONS);
	}

	// Get scans from any of the input ports
	base::samples::Pointcloud cloud;

	// Deprecated velodyne output format
	velodyne_lidar::MultilevelLaserScan scan;
	while(_scan.read(scan, false) == RTT::NewData)
	{
		// Convert to PointCloud
		cloud.points.clear();
		std::vector<Eigen::Vector3d> pc;
		velodyne_lidar::ConvertHelper::convertScanToPointCloud(scan, pc);
		copyPointCloud(pc, cloud.points);
		cloud.time = scan.time;
		_cloud.write(cloud);
	}

	base::samples::DepthMap depthMap;
	while(_depth_map.read(depthMap, false) == RTT::NewData)
	{
		depthMap.convertDepthMapToPointCloud(cloud.points, true);
		cloud.time = depthMap.time;
		_cloud.write(cloud);
	}
}

void ScanConverter::errorHook()
{
	ScanConverterBase::errorHook();
}

void ScanConverter::stopHook()
{
	ScanConverterBase::stopHook();
}

void ScanConverter::cleanupHook()
{
	ScanConverterBase::cleanupHook();
}
