#ifndef SLAM3D_LINESCANCONVERTER_TASK_HPP
#define SLAM3D_LINESCANCONVERTER_TASK_HPP

#include "slam3d/LineScanConverterBase.hpp"

namespace slam3d
{
	class LineScanConverter : public LineScanConverterBase
	{
	friend class LineScanConverterBase;
	protected:

	public:
		LineScanConverter(std::string const& name = "slam3d::LineScanConverter", TaskCore::TaskState initial_state = Stopped);
		LineScanConverter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);
		~LineScanConverter();

		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
    };
}

#endif

