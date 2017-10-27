#ifndef SLAM3D_SCANCONVERTER_TASK_HPP
#define SLAM3D_SCANCONVERTER_TASK_HPP

#include "slam3d/ScanConverterBase.hpp"

namespace slam3d
{
	class ScanConverter : public ScanConverterBase
	{
	friend class ScanConverterBase;
	
	protected:
		void copyPointCloud(const std::vector< Eigen::Vector3d >& pc_eigen, std::vector< base::Point >& pc_base) const;

	public:
		ScanConverter(std::string const& name = "slam3d::ScanConverter", TaskCore::TaskState initial_state = Stopped);
		ScanConverter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);
		~ScanConverter();
		
		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif

