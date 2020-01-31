#include "PointcloudToBinary.hpp"

#include <base-logging/Logging.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <velodyne_lidar/pointcloudConvertHelper.hpp>

using namespace slam3d;

PointcloudToBinary::PointcloudToBinary(std::string const& name, TaskCore::TaskState initial_state)
    : PointcloudToBinaryBase(name, initial_state)
{
}

PointcloudToBinary::PointcloudToBinary(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : PointcloudToBinaryBase(name, engine, initial_state)
{
}

PointcloudToBinary::~PointcloudToBinary()
{
}

bool PointcloudToBinary::configureHook()
{
    if (! PointcloudToBinaryBase::configureHook())
        return false;
    return true;
}
bool PointcloudToBinary::startHook()
{
    if (! PointcloudToBinaryBase::startHook())
        return false;
		
    mScanNumber = 0;
	mTransformLog = fopen("transforms.log", "wb");
	mPointcloudLog = fopen("pcl.log", "wb");
    return true;
}

void PointcloudToBinary::handlePointcloud(const std::vector<Eigen::Vector3d>& points, std::vector<float> remission)
{
	int num = points.size();
	int pointer = 0;
	float* data = new float[num*4];
	
	for(int point = 0; point < num; ++point)
	{
		data[pointer] = (float)(points.at(point)[0]); pointer++;
		data[pointer] = (float)(points.at(point)[1]); pointer++;
		data[pointer] = (float)(points.at(point)[2]); pointer++;
		data[pointer] = remission.at(point);          pointer++;
	}
	
	try
	{
		char filename[20];
		sprintf(filename, "%06d.bin", mScanNumber);
		FILE* stream = fopen(filename, "wb");
		fwrite(data, sizeof(float), 4*num, stream);
		fclose(stream);
		mScanNumber++;
	}catch(std::exception &e)
	{
		LOG_ERROR("Error writing file: %s", e.what());
	}
	delete data;
}

void PointcloudToBinary::updateHook()
{
    PointcloudToBinaryBase::updateHook();
	
	// Read odometry data
	base::samples::RigidBodyState rbs;
	double* pose_data = new double[7];
	while(_odometry.read(rbs, false) == RTT::NewData)
	{
		int64_t stamp = rbs.time.microseconds;
		pose_data[0] = rbs.position[0];
		pose_data[1] = rbs.position[1];
		pose_data[2] = rbs.position[2];
		pose_data[3] = rbs.orientation.x();
		pose_data[4] = rbs.orientation.y();
		pose_data[5] = rbs.orientation.z();
		pose_data[6] = rbs.orientation.w();
		fwrite(&stamp, sizeof(int64_t), 1, mTransformLog);
		fwrite(pose_data, sizeof(double), 7, mTransformLog);
	}

	// Read pointcloud from the port
	base::samples::Pointcloud cloud;
	while(_pcl.read(cloud, false) == RTT::NewData)
	{
		// Write timestamp to log file
		int64_t stamp = cloud.time.microseconds;
		fwrite(&stamp, sizeof(int64_t), 1, mPointcloudLog);
		fwrite(&mScanNumber, sizeof(int), 1, mPointcloudLog);
		
		// Write pointcloud to vector
		std::vector<Eigen::Vector3d> points;
		std::vector<float> remission;
		for(std::vector<base::Vector3d>::const_iterator it = cloud.points.begin(); it < cloud.points.end(); ++it)
		{
			Eigen::Vector3d p;
			p[0] = (*it)[0];
			p[1] = (*it)[1];
			p[2] = (*it)[2];
			points.push_back(p);
			remission.push_back(1.0);
		}
		handlePointcloud(points, remission);
	}
}

void PointcloudToBinary::errorHook()
{
    PointcloudToBinaryBase::errorHook();
}

void PointcloudToBinary::stopHook()
{
    PointcloudToBinaryBase::stopHook();
	fclose(mTransformLog);
	fclose(mPointcloudLog);
}

void PointcloudToBinary::cleanupHook()
{
    PointcloudToBinaryBase::cleanupHook();
}
