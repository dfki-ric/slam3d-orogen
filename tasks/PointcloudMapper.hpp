#ifndef SLAM3D_POINTCLOUDMAPPER_TASK_HPP
#define SLAM3D_POINTCLOUDMAPPER_TASK_HPP

#include <slam3d/PointcloudMapperBase.hpp>
#include <slam3d/core/Mapper.hpp>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>

#include <envire_core/items/Item.hpp>
#include <maps/grid/MLSMap.hpp>

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
		slam3d::Graph* mGraph;
		slam3d::Mapper* mMapper;
		slam3d::PointCloudSensor* mPclSensor;
		slam3d::Solver* mSolver;
		RockOdometry* mOdometry;
		boost::shared_mutex mGraphMutex;
		boost::shared_mutex mMapMutex;

		std::string mRobotName;
		std::string mRobotFrame;
		std::string mOdometryFrame;
		std::string mMapFrame;
		
		int mScansAdded;
		bool mForceAdd;
		
		// Parameters for creation of map-pcl
		double mScanResolution;

		// Parameters for creation of MLS
		envire::core::Item<maps::grid::MLSMapKalman> mMultiLayerMap;
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
