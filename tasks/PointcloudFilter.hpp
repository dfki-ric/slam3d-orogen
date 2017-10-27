#ifndef SLAM3D_POINTCLOUDFILTER_TASK_HPP
#define SLAM3D_POINTCLOUDFILTER_TASK_HPP

#include "slam3d/PointcloudFilterBase.hpp"

namespace slam3d
{
	class PointcloudFilter : public PointcloudFilterBase
	{
	friend class PointcloudFilterBase;
	
	protected:
		double mMinHeight;
		double mMaxHeight;
		double mSqMinDistance;
		double mSqMaxDistance;
		unsigned mPassRate;
		
		unsigned mSkipCount;

	public:
		PointcloudFilter(std::string const& name = "slam3d::PointcloudFilter", TaskCore::TaskState initial_state = Stopped);
		PointcloudFilter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);
		~PointcloudFilter();
		
		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif

