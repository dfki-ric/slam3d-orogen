#ifndef SLAM3D_POINTCLOUDMAPPER_TASK_HPP
#define SLAM3D_POINTCLOUDMAPPER_TASK_HPP

#include <slam3d/PointcloudMapperBase.hpp>
#include <slam3d/GraphMapper.hpp>
#include <slam3d/PointCloudSensor.hpp>
#include <slam3d/Solver.hpp>

#include <envire/maps/MLSGrid.hpp>
#include <envire/maps/Pointcloud.hpp>

#include <queue>
#include <boost/thread/shared_mutex.hpp>

#include "GridConfiguration.hpp"

namespace slam3d
{	
	class RockOdometry;
	
	class PointcloudMapper : public PointcloudMapperBase
	{
	friend class PointcloudMapperBase;
	protected:

		// Operations
		virtual bool generate_cloud();
		virtual bool generate_map();
		virtual bool optimize();
		virtual bool force_add();
		virtual bool write_graph();
		virtual bool write_envire();
		virtual bool write_ply(const std::string& folder);
		
		// Callbacks
		virtual bool setLog_level(boost::int32_t value);
		void transformerCallback(const base::Time &time);

		// Internal methods
		PointCloud::Ptr buildPointcloud(const VertexObjectList& vertices);
		void sendPointcloud(const VertexObjectList& vertices);
		virtual void handleNewScan(const VertexObject& scan);
		virtual void addScanToMap(PointCloudMeasurement::Ptr scan, const Transform& pose);
		virtual void clearMap();
		virtual void rebuildMap(const VertexObjectList& vertices);
		virtual void sendMap();
		bool loadPLYMap(const std::string& path);
	
		// Members
		slam3d::Clock* mClock;
		slam3d::Logger* mLogger;
		slam3d::GraphMapper* mMapper;
		slam3d::PointCloudSensor* mPclSensor;
		slam3d::Solver* mSolver;
		RockOdometry* mOdometry;
		boost::shared_mutex mGraphMutex;

		std::string mRobotName;
		std::string mRobotFrame;
		std::string mOdometryFrame;
		std::string mMapFrame;
		
		int mScansAdded;
		bool mForceAdd;
		
		// Parameters for creation of map-pcl
		double mScanResolution;
		bool mUseColorsAsViewpoints;

		// Parameters for creation of MLS
		envire::Environment mEnvironment;
		envire::MLSGrid* mMultiLayerMap;
		envire::Pointcloud* mPointcloud;
		GridConfiguration mGridConf;
		
		// Current state of transformations
		Eigen::Affine3d mCurrentOdometry;
		Eigen::Affine3d mCurrentDrift;
		base::Time mCurrentTime;

	public:
		PointcloudMapper(std::string const& name = "slam3d::PointcloudMapper");
		PointcloudMapper(std::string const& name, RTT::ExecutionEngine* engine);
		~PointcloudMapper();

		slam3d::PointCloud::Ptr createFromRockMessage(const base::samples::Pointcloud& cloud);
		void createFromPcl(slam3d::PointCloud::ConstPtr pcl_cloud, base::samples::Pointcloud& base_cloud);
		PointCloudMeasurement::Ptr castToPointcloud(Measurement::Ptr m);

		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif
