#include "LineScanConverter.hpp"

#include <base-logging/Logging.hpp>
#include <base/samples/LaserScan.hpp>
#include <base/samples/Pointcloud.hpp>

using namespace slam3d;

LineScanConverter::LineScanConverter(std::string const& name, TaskCore::TaskState initial_state)
    : LineScanConverterBase(name, initial_state)
{
}

LineScanConverter::LineScanConverter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : LineScanConverterBase(name, engine, initial_state)
{
}

LineScanConverter::~LineScanConverter()
{
}

bool LineScanConverter::configureHook()
{
	if (! LineScanConverterBase::configureHook())
		return false;
	return true;
}

bool LineScanConverter::startHook()
{
	if (! LineScanConverterBase::startHook())
		return false;
	return true;
}

void LineScanConverter::updateHook()
{
	LineScanConverterBase::updateHook();
	
	base::samples::LaserScan scan;
	while(_scan.read(scan, false) == RTT::NewData)
	{
		std::vector<Eigen::Vector3d> points;
		scan.convertScanToPointCloud(points);
		if(points.size() == 0)
		{
			LOG_ERROR("Convertion to pointcloud returned no points!");
			return;
		}else
		{
			LOG_DEBUG("Converted to pointcloud with %d points.", points.size());
		}
		
		base::samples::Pointcloud cloud;
		for(std::vector<Eigen::Vector3d>::iterator it = points.begin(); it < points.end(); ++it)
		{
			cloud.points.push_back(*it);
		}
		cloud.time = scan.time;
		_cloud.write(cloud);
	}
}

void LineScanConverter::errorHook()
{
	LineScanConverterBase::errorHook();
}

void LineScanConverter::stopHook()
{
	LineScanConverterBase::stopHook();
}

void LineScanConverter::cleanupHook()
{
	LineScanConverterBase::cleanupHook();
}
