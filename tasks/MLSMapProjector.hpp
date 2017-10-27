#ifndef SLAM3D_MLS_RENDERER_TASK_HPP
#define SLAM3D_MLS_RENDERER_TASK_HPP

#include "slam3d/MLSMapProjectorBase.hpp"

#include <envire/Orocos.hpp>
#include <envire/operators/MLSProjection.hpp>

namespace slam3d
{
	class MLSMapProjector : public MLSMapProjectorBase
	{
	friend class MLSMapProjectorBase;
	
	protected:
		typedef std::vector<envire::BinaryEvent> EnvireEvents;
		typedef RTT::extras::ReadOnlyPointer<EnvireEvents> EnvirePointer;
	
	protected:
		envire::Environment mEnvironment;
		envire::Pointcloud* mPointcloud;
		envire::MultiLevelSurfaceGrid* mMultiLayerMap;
		envire::MLSProjection* mProjection;

	public:
		MLSMapProjector(std::string const& name = "slam3d::MLSMapProjector", TaskCore::TaskState initial_state = Stopped);
		MLSMapProjector(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);
		~MLSMapProjector();
		
		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif
