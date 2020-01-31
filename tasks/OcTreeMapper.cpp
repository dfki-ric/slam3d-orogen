#include "OcTreeMapper.hpp"

#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <pcl/common/transforms.h>

using namespace slam3d;

octomap::OcTree* initOcTree(const OctoMapConfiguration &conf)
{
	octomap::OcTree* tree = new octomap::OcTree(conf.resolution);
	tree->setOccupancyThres(conf.occupancyThres);
	tree->setProbHit(conf.probHit);
	tree->setProbMiss(conf.probMiss);
	tree->setClampingThresMin(conf.clampingThresMin);
	tree->setClampingThresMax(conf.clampingThresMax);
	return tree;
}

OcTreeMapper::OcTreeMapper(std::string const& name)
    : OcTreeMapperBase(name)
{
}

OcTreeMapper::OcTreeMapper(std::string const& name, RTT::ExecutionEngine* engine)
    : OcTreeMapperBase(name, engine)
{
}

OcTreeMapper::~OcTreeMapper()
{
}

void OcTreeMapper::addScanToMap(PointCloudMeasurement::Ptr scan, const Transform& pose)
{
	PointCloud::Ptr tempCloud(new PointCloud);
	Transform sensor = pose * scan->getSensorPose();
	pcl::transformPointCloud(*(scan->getPointCloud()), *tempCloud, sensor.matrix());

	octomap::Pointcloud octoCloud;
	for(PointCloud::iterator it = tempCloud->begin(); it < tempCloud->end(); ++it)
	{
		octoCloud.push_back(octomap::point3d(it->x, it->y,it->z));
	}
	Eigen::Vector3d origin = sensor.translation();
	mOcTree->insertPointCloud(octoCloud, octomap::point3d(origin(0), origin(1), origin(2)), mOctreeConf.rangeMax, true, true);
}

void OcTreeMapper::clearMap()
{
	// Reset OctoMap (OcTree::clear is currently not working)
	delete mOcTree;
	mOcTree = initOcTree(mOctreeConf);
}

void OcTreeMapper::buildMLS()
{
	mLogger->message(WARNING, "Generating MLS from OcTree is not implemented.");
/*
	mLogger->message(DEBUG, "Generating MLS from OcTree.");
	mMultiLayerMap->clear();
	for(octomap::OcTree::leaf_iterator leaf = mOcTree->begin_leafs() ; leaf != mOcTree->end_leafs(); ++leaf)
	{
		if(!mOcTree->isNodeOccupied(*leaf))
		{
			continue;
		}
		double half_size = leaf.getSize() / 2.0;
		double half_res = mGridConf.resolution / 2.0;
		double x_min = leaf.getX() - half_size + half_res;
		double y_min = leaf.getY() - half_size + half_res;
		double x_max = leaf.getX() + half_size;
		double y_max = leaf.getY() + half_size;
		
		for(double y = y_min; y < y_max; y += mGridConf.resolution)
		{
			for(double x = x_min; x < x_max; x += mGridConf.resolution)
			{
				double z_low  = std::max(mGridConf.min_z, leaf.getZ() - half_size);
				double z_high = std::min(mGridConf.max_z, leaf.getZ() + half_size);
				if(z_high > z_low)
				{
					envire::SurfacePatch patch(z_high, 0, z_high - z_low, envire::SurfacePatch::VERTICAL);
					mMultiLayerMap->update(Eigen::Vector2d(x, y) , patch);
				}
			}
		}
	}
*/
}

void OcTreeMapper::sendMap()
{
	// Build and publish the MLS-Map
	mOcTree->updateInnerOccupancy();
	mOcTree->writeBinaryConst("slam3d_octomap.bt");
	buildMLS();
	mMultiLayerMap.setTime(mCurrentTime);
	_mls.write(mMultiLayerMap.asSpatioTemporal());
}

bool OcTreeMapper::remove_dynamic_objects()
{
	mLogger->message(INFO, "Requested dynamic object removal.");
	timeval start = mClock->now();
	VertexObjectList vertices = mGraph->getVerticesFromSensor(mPclSensor->getName());
	unsigned deleted = 0;
	
	boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);
	for(VertexObjectList::iterator v = vertices.begin(); v != vertices.end(); ++v)
	{
		// Cast to PointCloudMeasurement
		PointCloudMeasurement::Ptr m = boost::dynamic_pointer_cast<PointCloudMeasurement>(v->measurement);
		if(!m)
		{
			mLogger->message(WARNING, "Vertex given to remove_dynamic_objects is not a Pointcloud!");
			continue;
		}
		
		// Check each point, if it is in free OctoMap voxel
		PointCloud::Ptr cloud = m->getPointCloud();
		for(PointCloud::iterator p = cloud->begin(); p != cloud->end();)
		{
			base::Vector3d p_tf;
			p_tf[0] = p->x;
			p_tf[1] = p->y;
			p_tf[2] = p->z;
			p_tf = v->corrected_pose * m->getSensorPose() * p_tf;
			octomap::OcTreeNode* node = mOcTree->search(p_tf[0], p_tf[1], p_tf[2]);
			if(node && !mOcTree->isNodeOccupied(node))
			{
				deleted++;
				p = cloud->erase(p);
			}else
			{
				p++;
			}
		}
	}
	int duration = mClock->now().tv_sec - start.tv_sec;
	mLogger->message(INFO, (boost::format("Removed %1% dynamic points in %2% seconds.") % deleted % duration).str());
	return true;
}

bool OcTreeMapper::configureHook()
{
	if (! OcTreeMapperBase::configureHook())
		return false;
		
	mOctreeConf  = _octo_map_config.get();
	mOcTree = initOcTree(mOctreeConf);
	
	mLogger->message(INFO, " = OctoMap - Parameters =");
	mLogger->message(INFO, (boost::format("resolution:       %1%") % mOctreeConf.resolution).str());
	mLogger->message(INFO, (boost::format("occupancyThres:   %1%") % mOctreeConf.occupancyThres).str());
	mLogger->message(INFO, (boost::format("probHit:          %1%") % mOctreeConf.probHit).str());
	mLogger->message(INFO, (boost::format("probMiss:         %1%") % mOctreeConf.probMiss).str());
	mLogger->message(INFO, (boost::format("clampingThresMin: %1%") % mOctreeConf.clampingThresMin).str());
	mLogger->message(INFO, (boost::format("clampingThresMax: %1%") % mOctreeConf.clampingThresMax).str());
	mLogger->message(INFO, (boost::format("rangeMax:         %1%") % mOctreeConf.rangeMax).str());
	
	return true;
}

bool OcTreeMapper::startHook()
{
	if (! OcTreeMapperBase::startHook())
		return false;
	return true;
}

void OcTreeMapper::updateHook()
{
	OcTreeMapperBase::updateHook();
}

void OcTreeMapper::errorHook()
{
	OcTreeMapperBase::errorHook();
}

void OcTreeMapper::stopHook()
{
	OcTreeMapperBase::stopHook();
}

void OcTreeMapper::cleanupHook()
{
	OcTreeMapperBase::cleanupHook();
	delete mOcTree;
}
