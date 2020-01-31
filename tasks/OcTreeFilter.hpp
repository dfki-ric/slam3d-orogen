#ifndef SLAM3D_OCTREEFILTER_TASK_HPP
#define SLAM3D_OCTREEFILTER_TASK_HPP

#include "slam3d/OcTreeFilterBase.hpp"

#include <base/samples/Pointcloud.hpp>
#include <octomap/OcTree.h>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>

#include "OctoMapConfiguration.hpp"

namespace slam3d
{
	class OcTreeFilter : public OcTreeFilterBase
	{
	friend class OcTreeFilterBase;
	
	protected:
	
		// Callbacks
		virtual void inputTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &scan_sample);
		virtual bool setLog_level(boost::int32_t value);

		// Internal methods
		PointCloud::Ptr downsample(PointCloud::Ptr source, float leaf_size);

		// Members
		slam3d::PointCloud mPointcloud;
		octomap::OcTree* mOcTree;
		OctoMapConfiguration mOctoConfig;
		unsigned mScanCount;
		Clock* mClock;
		Logger* mLogger;

		// Parameters
		double mMinHeight;
		double mMaxHeight;
		double mSqMinDistance;
		double mSqMaxDistance;
		unsigned mPassRate;

	public:
		OcTreeFilter(std::string const& name = "slam3d::OcTreeFilter");
		OcTreeFilter(std::string const& name, RTT::ExecutionEngine* engine);
		~OcTreeFilter();
		
		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif

