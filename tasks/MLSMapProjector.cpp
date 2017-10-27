#include "MLSMapProjector.hpp"
#include "GridConfiguration.hpp"

#include <base-logging/Logging.hpp>
#include <envire/Orocos.hpp>

using namespace slam3d;

MLSMapProjector::MLSMapProjector(std::string const& name, TaskCore::TaskState initial_state)
    : MLSMapProjectorBase(name, initial_state)
{
}

MLSMapProjector::MLSMapProjector(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : MLSMapProjectorBase(name, engine, initial_state)
{
}

MLSMapProjector::~MLSMapProjector()
{
}

bool MLSMapProjector::configureHook()
{
	if (! MLSMapProjectorBase::configureHook())
		return false;

	// Initialize MLS-Map
	GridConfiguration grid = _grid_config.get();
	size_t x_size = (grid.max_x - grid.min_x) / grid.resolution;
	size_t y_size = (grid.max_y - grid.min_y) / grid.resolution;
	
	mMultiLayerMap = new envire::MultiLevelSurfaceGrid(x_size, y_size, grid.resolution, grid.resolution, grid.min_x, grid.min_y);
	mMultiLayerMap->setUniqueId("/slam3d-proj-mls");
	mMultiLayerMap->getConfig() = _mls_config.get();
	
	mPointcloud = new envire::Pointcloud();
	mProjection = new envire::MLSProjection();

	// add pointcloud to environment
	envire::FrameNode* cloud_node = new envire::FrameNode();
	mEnvironment.addChild(mEnvironment.getRootNode(), cloud_node);
	mEnvironment.setFrameNode(mPointcloud, cloud_node);
	
	// add mls to environment
	envire::FrameNode* mls_node = new envire::FrameNode();
	mEnvironment.addChild(mEnvironment.getRootNode(), mls_node);
	mEnvironment.setFrameNode(mMultiLayerMap, mls_node);
	
	mProjection->setAreaOfInterest(grid.min_x, grid.max_x, grid.min_y, grid.max_y, grid.min_z, grid.max_z);
	if(!mEnvironment.addInput(mProjection, mPointcloud))
	{
		LOG_ERROR("Failed to add Input to Envire!");
		return false;
	}
	if(!mEnvironment.addOutput(mProjection, mMultiLayerMap))
	{
		LOG_ERROR("Failed to add Output to Envire!");
		return false;
	}

	return true;
}

bool MLSMapProjector::startHook()
{
    if (! MLSMapProjectorBase::startHook())
        return false;
    return true;
}

void MLSMapProjector::updateHook()
{
    MLSMapProjectorBase::updateHook();
	
	// Read point cloud from the port
	base::samples::Pointcloud cloud;
	while(_cloud.read(cloud, false) == RTT::NewData)
	{
		mPointcloud->vertices.clear();
		for(std::vector<base::Vector3d>::const_iterator it = cloud.points.begin(); it < cloud.points.end(); ++it)
		{
			mPointcloud->vertices.push_back(*it);
		}
		mMultiLayerMap->clear();
		mProjection->updateAll();
	}
	
	// Publish the MLS-Map	
	envire::OrocosEmitter emitter(&mEnvironment, _envire_map);
	emitter.flush();
}

void MLSMapProjector::errorHook()
{
    MLSMapProjectorBase::errorHook();
}

void MLSMapProjector::stopHook()
{
    MLSMapProjectorBase::stopHook();
}

void MLSMapProjector::cleanupHook()
{
    MLSMapProjectorBase::cleanupHook();
}
