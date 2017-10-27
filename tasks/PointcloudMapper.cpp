#include "PointcloudMapper.hpp"
#include "RockOdometry.hpp"
#include "BaseLogger.hpp"
#include "Common.hpp"

#include <base/samples/Pointcloud.hpp>
#include <base/samples/RigidBodyState.hpp>

#include <slam3d/BoostMapper.hpp>
#include <slam3d/FileLogger.hpp>
#include <slam3d/G2oSolver.hpp>

#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include <envire/Orocos.hpp>

using namespace slam3d;

PointcloudMapper::PointcloudMapper(std::string const& name)
    : PointcloudMapperBase(name)
{
}

PointcloudMapper::PointcloudMapper(std::string const& name, RTT::ExecutionEngine* engine)
    : PointcloudMapperBase(name, engine)
{
}

PointcloudMapper::~PointcloudMapper()
{
}

bool PointcloudMapper::optimize()
{
	mLogger->message(INFO, "Requested global optimization.");
	try
	{
		boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);
		if(mMapper->optimize())
		{
			return true;
		}
	}catch (boost::lock_error &e)
	{
		mLogger->message(WARNING, "Could not access the pose graph for optimization! Is another operation still running?");
	}
	return false;
}

bool PointcloudMapper::generate_cloud()
{
	// Publish accumulated cloud
	mLogger->message(INFO, "Requested pointcloud generation.");
	VertexObjectList vertices = mMapper->getVertexObjectsFromSensor(mPclSensor->getName());
	boost::thread projThread(&PointcloudMapper::sendPointcloud, this, vertices);
	return true;
}

bool PointcloudMapper::generate_map()
{
	mLogger->message(INFO, "Requested map generation.");
	if(mMapper->optimized())
	{
		VertexObjectList vertices = mMapper->getVertexObjectsFromSensor(mPclSensor->getName());
		boost::thread projThread(&PointcloudMapper::rebuildMap, this, vertices);
	}else
	{
		sendMap();
	}
	return true;
}

bool PointcloudMapper::force_add()
{
	mForceAdd = true;
	return true;
}

bool PointcloudMapper::write_envire()
{
	mEnvironment.serialize(_envire_path.get());
	return true;
}

bool PointcloudMapper::write_graph()
{
	mMapper->writeGraphToFile("slam3d_graph");
	return true;
}

bool PointcloudMapper::write_ply(const std::string& folder)
{
	mLogger->message(INFO, "Write pointcloud to PLY file.");
	VertexObjectList vertices = mMapper->getVertexObjectsFromSensor(mPclSensor->getName());
	PointCloud::Ptr accCloud;
	try
	{
		accCloud = buildPointcloud(vertices);
	}
	catch (boost::lock_error &e)
	{
		mLogger->message(ERROR, "Could not access the pose graph to build Pointcloud!");
		return false;
	}

	boost::filesystem::path ply_path(folder);
	boost::filesystem::create_directories(ply_path);
	ply_path += "pointcloud-";
	ply_path += base::Time::now().toString(base::Time::Seconds, "%Y%m%d-%H%M");
	ply_path += ".ply";

	pcl::PLYWriter ply_writer;
	return ply_writer.write(ply_path.string(), *accCloud) >= 0;
}

PointCloud::Ptr PointcloudMapper::buildPointcloud(const VertexObjectList& vertices)
{
	timeval start = mClock->now();
	PointCloud::Ptr accumulated;

	boost::shared_lock<boost::shared_mutex> guard(mGraphMutex);
	accumulated = mPclSensor->getAccumulatedCloud(vertices);

	PointCloud::Ptr cleaned = mPclSensor->removeOutliers(accumulated, _map_outlier_radius, _map_outlier_neighbors);
	PointCloud::Ptr downsampled = mPclSensor->downsample(cleaned, _map_resolution);

	timeval finish = mClock->now();
	int duration = finish.tv_sec - start.tv_sec;
	mLogger->message(INFO, (boost::format("Generated Pointcloud from %1% scans in %2% seconds.") % vertices.size() % duration).str());
	return downsampled;
}

void PointcloudMapper::sendPointcloud(const VertexObjectList& vertices)
{
	PointCloud::Ptr accCloud;
	try
	{
		accCloud = buildPointcloud(vertices);
	}catch (boost::lock_error &e)
	{
		mLogger->message(ERROR, "Could not access the pose graph to build Pointcloud!");
		return;
	}
	base::samples::Pointcloud mapCloud;
	mPointcloud->vertices.clear();
	for(PointCloud::iterator it = accCloud->begin(); it < accCloud->end(); ++it)
	{
		base::Vector3d vec;
		vec[0] = it->x;
		vec[1] = it->y;
		vec[2] = it->z;
		mapCloud.points.push_back(vec);
		mPointcloud->vertices.push_back(vec);
	}
	mapCloud.time = base::Time::fromMicroseconds(accCloud->header.stamp);
	_cloud.write(mapCloud);	
}

PointCloudMeasurement::Ptr PointcloudMapper::castToPointcloud(Measurement::Ptr m)
{
	PointCloudMeasurement::Ptr pcm = boost::dynamic_pointer_cast<PointCloudMeasurement>(m);
	if(!pcm)
	{
		mLogger->message(ERROR, "Measurement is not a point cloud!");
		throw BadMeasurementType();
	}
	return pcm;
}

void PointcloudMapper::handleNewScan(const VertexObject& scan)
{
	addScanToMap(castToPointcloud(scan.measurement), scan.corrected_pose);
}

void PointcloudMapper::addScanToMap(PointCloudMeasurement::Ptr scan, const Transform& pose)
{	
	PointCloud::ConstPtr pcl = scan->getPointCloud();
	Transform sensor_pose = pose * scan->getSensorPose();
	for(PointCloud::const_iterator it = pcl->begin(); it != pcl->end(); ++it)
	{
		Eigen::Vector3d p = sensor_pose * Eigen::Vector3d(it->x, it->y, it->z);
		if(p[2] >= mGridConf.min_z && p[2] <= mGridConf.max_z)
		{
			envire::MLSGrid::SurfacePatch patch( p[2], 0.1 );
			mMultiLayerMap->update(Eigen::Vector2d(p[0], p[1]) , patch );
		}
	}
}

void PointcloudMapper::clearMap()
{
	mMultiLayerMap->clear();
}

void PointcloudMapper::rebuildMap(const VertexObjectList& vertices)
{
	timeval start = mClock->now();
	clearMap();

	boost::shared_lock<boost::shared_mutex> guard(mGraphMutex);
	for(VertexObjectList::const_iterator v = vertices.begin(); v != vertices.end(); ++v)
	{
		addScanToMap(castToPointcloud(v->measurement), v->corrected_pose);
	}
	timeval finish = mClock->now();
	int duration = finish.tv_sec - start.tv_sec;
	mLogger->message(INFO, (boost::format("Completely rebuild map from %1% scans in %2% seconds.") % vertices.size() % duration).str());

	sendMap();
}

void PointcloudMapper::sendMap()
{
	// Publish the MLS-Map	
	envire::OrocosEmitter emitter(&mEnvironment, _envire_map);
	emitter.setTime(mCurrentTime);
	emitter.flush();
}

bool PointcloudMapper::loadPLYMap(const std::string& path)
{
	PointCloud::Ptr pcl_cloud(new PointCloud());
	pcl::PLYReader ply_reader;
	if(ply_reader.read(path, *pcl_cloud) >= 0)
	{
		Transform pc_tr(pcl_cloud->sensor_orientation_.cast<ScalarType>());
		pc_tr.translation() = pcl_cloud->sensor_origin_.block(0,0,3,1).cast<ScalarType>();
		PointCloudMeasurement::Ptr initial_map(new PointCloudMeasurement(pcl_cloud, mRobotName, mPclSensor->getName(), pc_tr));
		try
		{
			VertexObject root_node = mMapper->getVertex(0);
			mMapper->addExternalReading(initial_map, root_node.measurement->getUniqueId(), Transform::Identity(), Covariance::Identity(), "none");
			addScanToMap(initial_map, Transform::Identity());
			return true;
		}
		catch(std::exception& e)
		{
			mLogger->message(ERROR, (boost::format("Adding initial point cloud failed: %1%") % e.what()).str());
		}
	}else
	{
		mLogger->message(ERROR, (boost::format("Failed to load a-priori PLY map %1%") % path).str());
	}
	return false;
}

bool PointcloudMapper::setLog_level(boost::int32_t value)
{
	switch(value)
	{
	case 4:
		mLogger->setLogLevel(FATAL);
		break;
	case 3:
		mLogger->setLogLevel(ERROR);
		break;
	case 2:
		mLogger->setLogLevel(WARNING);
		break;
	case 0:
		mLogger->setLogLevel(DEBUG);
		break;
	default:
		mLogger->setLogLevel(INFO);
	}
	return true;
}

bool PointcloudMapper::configureHook()
{	
	if (! PointcloudMapperBase::configureHook())
		return false;
		
	mClock = new Clock();
	switch(_log_type)
	{
	case 0:
		mLogger = new Logger(*mClock);
		break;
	case 1:
		mLogger = new BaseLogger();
		break;
	case 2:
		mLogger = new FileLogger(*mClock, "slam3d.log");
		break;
	default:
		mLogger = new Logger(*mClock);
		mLogger->message(WARNING, "Invalid logger type, using standard logger.");
	}

	setLog_level(_log_level);

	mLogger->message(INFO, "=== Configure PointCloudMapper ===");

	mPclSensor = new PointCloudSensor("pointcloud", mLogger, Transform::Identity());
	GICPConfiguration conf = _gicp_config.get();
	mPclSensor->setFineConfiguaration(conf);
	mLogger->message(INFO, " = GICP - Parameters =");
	mLogger->message(INFO, (boost::format("correspondence_randomness:    %1%") % conf.correspondence_randomness).str());
	mLogger->message(INFO, (boost::format("euclidean_fitness_epsilon:    %1%") % conf.euclidean_fitness_epsilon).str());
	mLogger->message(INFO, (boost::format("max_correspondence_distance:  %1%") % conf.max_correspondence_distance).str());
	mLogger->message(INFO, (boost::format("max_fitness_score:            %1%") % conf.max_fitness_score).str());
	mLogger->message(INFO, (boost::format("max_sensor_distance:          %1%") % conf.max_sensor_distance).str());
	mLogger->message(INFO, (boost::format("maximum_iterations:           %1%") % conf.maximum_iterations).str());
	mLogger->message(INFO, (boost::format("maximum_optimizer_iterations: %1%") % conf.maximum_optimizer_iterations).str());
	mLogger->message(INFO, (boost::format("orientation_sigma:            %1%") % conf.orientation_sigma).str());
	mLogger->message(INFO, (boost::format("point_cloud_density:          %1%") % conf.point_cloud_density).str());
	mLogger->message(INFO, (boost::format("position_sigma:               %1%") % conf.position_sigma).str());
	mLogger->message(INFO, (boost::format("rotation_epsilon:             %1%") % conf.rotation_epsilon).str());
	mLogger->message(INFO, (boost::format("transformation_epsilon:       %1%") % conf.transformation_epsilon).str());

	conf = _gicp_coarse_config.get();
	mPclSensor->setCoarseConfiguaration(conf);
	mLogger->message(INFO, " = GICP - Coarse Parameters =");
	mLogger->message(INFO, (boost::format("correspondence_randomness:    %1%") % conf.correspondence_randomness).str());
	mLogger->message(INFO, (boost::format("euclidean_fitness_epsilon:    %1%") % conf.euclidean_fitness_epsilon).str());
	mLogger->message(INFO, (boost::format("max_correspondence_distance:  %1%") % conf.max_correspondence_distance).str());
	mLogger->message(INFO, (boost::format("max_fitness_score:            %1%") % conf.max_fitness_score).str());
	mLogger->message(INFO, (boost::format("max_sensor_distance:          %1%") % conf.max_sensor_distance).str());
	mLogger->message(INFO, (boost::format("maximum_iterations:           %1%") % conf.maximum_iterations).str());
	mLogger->message(INFO, (boost::format("maximum_optimizer_iterations: %1%") % conf.maximum_optimizer_iterations).str());
	mLogger->message(INFO, (boost::format("orientation_sigma:            %1%") % conf.orientation_sigma).str());
	mLogger->message(INFO, (boost::format("point_cloud_density:          %1%") % conf.point_cloud_density).str());
	mLogger->message(INFO, (boost::format("position_sigma:               %1%") % conf.position_sigma).str());
	mLogger->message(INFO, (boost::format("rotation_epsilon:             %1%") % conf.rotation_epsilon).str());
	mLogger->message(INFO, (boost::format("transformation_epsilon:       %1%") % conf.transformation_epsilon).str());
	
	mSolver = new G2oSolver(mLogger);
	mMapper = new BoostMapper(mLogger);

	mLogger->message(INFO, " = GraphMapper - Parameters =");
	mLogger->message(INFO, (boost::format("use_odometry:           %1%") % _use_odometry.get()).str());	
	if(_use_odometry.get())
	{
		mOdometry = new RockOdometry(_robot2odometry, mLogger);
		mMapper->setOdometry(mOdometry, _add_odometry_edges.get());
		_robot2odometry.registerUpdateCallback(boost::bind(&PointcloudMapper::transformerCallback, this, _1));
		mLogger->message(INFO, (boost::format("add_odometry_edges:     %1%") % _add_odometry_edges.get()).str());
	}else
	{
		mOdometry = NULL;
	}
	mCurrentOdometry = Eigen::Affine3d::Identity();
	
	double min_translation = _min_translation.get();
	double min_rotation = _min_rotation.get();
	mLogger->message(INFO, (boost::format("min_pose_distance:      %1% / %2%") % min_translation % min_rotation).str());
	mMapper->setMinPoseDistance(min_translation, min_rotation);
	
	double neighbor_radius = _neighbor_radius.get();
	mLogger->message(INFO, (boost::format("neighbor_radius:        %1%") % neighbor_radius).str());
	int max_neighbor_links = _max_neighbor_links.get();
	mLogger->message(INFO, (boost::format("max_neighbor_links:     %1%") % max_neighbor_links).str());
	mMapper->setNeighborRadius(neighbor_radius, max_neighbor_links);
	
	unsigned range = _patch_building_range.get();
	mLogger->message(INFO, (boost::format("patch_building_range:   %1%") % range).str());
	mMapper->setPatchBuildingRange(range);
	
	base::Pose pose = _start_pose.get();
	base::Position p = pose.position;
	base::Orientation o = pose.orientation;
	mLogger->message(INFO, (boost::format("start_pose:             pos: (%1%, %2%, %3%) / quat: (%4%, %5%, %6%, %7%)")
	 % p[0] % p[1] % p[2] % o.x() % o.y() % o.z() % o.w()).str());
	mMapper->setCurrentPose(pose2transform(pose));
	
	mMapper->useOdometryHeading(_use_odometry_heading.get());
	mLogger->message(INFO, (boost::format("use_odometry_heading:   %1%") % _use_odometry_heading.get()).str());
	
	mScanResolution = _scan_resolution.get();
	mLogger->message(INFO, (boost::format("scan_resolution:        %1%") % mScanResolution).str());
	
	mLogger->message(INFO, (boost::format("map_resolution:         %1%") % _map_resolution).str());
	mLogger->message(INFO, (boost::format("map_outlier_radius:     %1%") % _map_outlier_radius).str());
	mLogger->message(INFO, (boost::format("map_outlier_neighbors:  %1%") % _map_outlier_neighbors).str());
	mLogger->message(INFO, (boost::format("map_publish_rate:       %1%") % _map_publish_rate).str());
	mLogger->message(INFO, (boost::format("optimization_rate:      %1%") % _optimization_rate).str());
	
	mRobotName = _robot_name.get();
	mLogger->message(INFO, (boost::format("robot_name:             %1%") % mRobotName).str());
	
	mLogger->message(INFO, (boost::format("laser_frame:            %1%") % _laser_frame.get()).str());
	
	mRobotFrame = _robot_frame.get();
	mLogger->message(INFO, (boost::format("robot_frame:            %1%") % mRobotFrame).str());
	
	mOdometryFrame = _odometry_frame.get();
	mLogger->message(INFO, (boost::format("odometry_frame:         %1%") % mOdometryFrame).str());

	mMapFrame = _map_frame.get();
	mLogger->message(INFO, (boost::format("map_frame:              %1%") % mMapFrame).str());

	mUseColorsAsViewpoints = _use_colors_as_viewpoints.get();
	mLogger->message(INFO, (boost::format("use_viewpoints:         %1%") % mUseColorsAsViewpoints).str());

	mMapper->registerSensor(mPclSensor);
	mMapper->setSolver(mSolver);
	
	mScansAdded = 0;
	mForceAdd = false;
	
	// Initialize MLS-Map
	mGridConf = _grid_config.get();
	mLogger->message(INFO, " = Grid - Parameters =");
	mLogger->message(INFO, (boost::format("min_x: %1%") % mGridConf.min_x).str());
	mLogger->message(INFO, (boost::format("max_x: %1%") % mGridConf.max_x).str());
	mLogger->message(INFO, (boost::format("min_y: %1%") % mGridConf.min_y).str());
	mLogger->message(INFO, (boost::format("max_y: %1%") % mGridConf.max_y).str());
	mLogger->message(INFO, (boost::format("min_z: %1%") % mGridConf.min_z).str());
	mLogger->message(INFO, (boost::format("max_z: %1%") % mGridConf.max_z).str());
	mLogger->message(INFO, (boost::format("resolution: %1%") % mGridConf.resolution).str());
	
	size_t x_size = (mGridConf.max_x - mGridConf.min_x) / mGridConf.resolution;
	size_t y_size = (mGridConf.max_y - mGridConf.min_y) / mGridConf.resolution;
	
	mMultiLayerMap = new envire::MultiLevelSurfaceGrid(x_size, y_size, mGridConf.resolution, mGridConf.resolution, mGridConf.min_x, mGridConf.min_y);
	mMultiLayerMap->setUniqueId("/slam3d-mls");
	mMultiLayerMap->getConfig() = _grid_mls_config.get();
	
	// Add MLS to Environment
	envire::FrameNode* mls_node = new envire::FrameNode();
	mEnvironment.addChild(mEnvironment.getRootNode(), mls_node);
	mEnvironment.setFrameNode(mMultiLayerMap, mls_node);
	
	// Add point cloud to environment
	mPointcloud = new envire::Pointcloud();
	mPointcloud->setUniqueId("slam3d-pointcloud");
	envire::FrameNode* cloud_node = new envire::FrameNode();
	mEnvironment.addChild(mEnvironment.getRootNode(), cloud_node);
	mEnvironment.setFrameNode(mPointcloud, cloud_node);

	// load a-priori map file
	if(!_apriori_ply_map.value().empty() && loadPLYMap(_apriori_ply_map.value()))
	{
		mScansAdded++;
	}
	
	return true;
}

bool PointcloudMapper::startHook()
{
	if(!PointcloudMapperBase::startHook())
		return false;
	return true;
}

PointCloud::Ptr PointcloudMapper::createFromRockMessage(const base::samples::Pointcloud& cloud_in)
{
	PointCloud::Ptr cloud_out(new PointCloud);
	cloud_out->header.stamp = cloud_in.time.toMicroseconds();
	cloud_out->reserve(cloud_in.points.size());
	
	if(mUseColorsAsViewpoints)
	{
		if(cloud_in.colors.size() != cloud_in.points.size())
		{
			mLogger->message(WARNING, "Color vector from pointcloud has invalid size!");
			mUseColorsAsViewpoints = false;
		}
	}
	
	unsigned numPoints = cloud_in.points.size();
	for(unsigned i = 0; i < numPoints; ++i)
	{
		PointType p;
		p.x = cloud_in.points[i][0];
		p.y = cloud_in.points[i][1];
		p.z = cloud_in.points[i][2];
		
		if(mUseColorsAsViewpoints)
		{
			p.vp_x = cloud_in.colors[i][0];
			p.vp_y = cloud_in.colors[i][1];
			p.vp_z = cloud_in.colors[i][2];
		}
		
		cloud_out->push_back(p);
	}
	return cloud_out;
}

void PointcloudMapper::createFromPcl(PointCloud::ConstPtr pcl_cloud, base::samples::Pointcloud& base_cloud)
{
	base_cloud.time.fromMicroseconds(pcl_cloud->header.stamp);
	base_cloud.points.reserve(pcl_cloud->size());
	for(PointCloud::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++)
	{
		base::Point p;
		p[0] = it->x;
		p[1] = it->y;
		p[2] = it->z;
		base_cloud.points.push_back(p);
	}
}

void PointcloudMapper::transformerCallback(const base::Time &time)
{
	try
	{
		mCurrentOdometry = mOdometry->getOdometricPose(time);

		// Send the current pose
		base::samples::RigidBodyState rbs;
		rbs.invalidateCovariances();
		rbs.time = time;
		rbs.sourceFrame = mRobotFrame;
		rbs.targetFrame = mMapFrame;
		rbs.setTransform(mCurrentDrift * mCurrentOdometry);
		_robot2map.write(rbs);
	}
	catch(OdometryException &e)
	{
		mLogger->message(ERROR, e.what());
	}
}

void PointcloudMapper::updateHook()
{
	PointcloudMapperBase::updateHook();

	base::samples::Pointcloud scan_sample;
	while(_scan.read(scan_sample, false) == RTT::NewData)
	{
		// Get laser pose
		Transform laserPose = Transform::Identity();
		try
		{
			Eigen::Affine3d affine;
			if(!_laser2robot.get(scan_sample.time, affine, false) || !affine.matrix().allFinite())
			{
				mLogger->message(ERROR, (boost::format("Failed to receive a valid transform from '%1%' to '%2%'!")
					% _laser_frame % _robot_frame).str());
				continue;
			}
			laserPose.linear() = affine.linear();
			laserPose.translation() = affine.translation();
		}
		catch(std::exception &e)
		{
			mLogger->message(ERROR, e.what());
			continue;
		}

		// Transform base::samples::Pointcloud --> Pointcloud
		PointCloud::Ptr cloud = createFromRockMessage(scan_sample);
		
		// Downsample and add to map
		PointCloudMeasurement::Ptr measurement;
		try
		{
			if(mScanResolution > 0)
			{
				PointCloud::Ptr downsampled_cloud = mPclSensor->downsample(cloud, mScanResolution);
				mLogger->message(DEBUG, (boost::format("Downsampled cloud has %1% points.") % downsampled_cloud->size()).str());
				measurement = PointCloudMeasurement::Ptr(new PointCloudMeasurement(downsampled_cloud, mRobotName, mPclSensor->getName(), laserPose));
			}else
			{
				measurement = PointCloudMeasurement::Ptr(new PointCloudMeasurement(cloud, mRobotName, mPclSensor->getName(), laserPose));
			}

			if(mMapper->addReading(measurement, mForceAdd))
			{
				mScansAdded++;
				mForceAdd = false;
				handleNewScan(mMapper->getLastVertex());
				if(_optimization_rate > 0 && (mScansAdded % _optimization_rate) == 0)
				{
					optimize();
				}
				if(_map_publish_rate > 0 && (mScansAdded % _map_publish_rate) == 0)
				{
					generate_map();
				}
			}else
			{
				addScanToMap(measurement, mMapper->getCurrentPose());
			}
			mCurrentTime = scan_sample.time;
			
			// Send the calculated transform
			base::samples::RigidBodyState rbs;
			rbs.invalidateCovariances();
			rbs.targetFrame = mMapFrame;
			rbs.time = mCurrentTime;
			
			if(mOdometry)
			{
				mCurrentDrift = mMapper->getCurrentPose() * mCurrentOdometry.inverse();
				rbs.sourceFrame = mOdometryFrame;
				rbs.setTransform(mCurrentDrift);
				_odometry2map.write(rbs);
			}else
			{
				rbs.sourceFrame = mRobotFrame;
				rbs.setTransform(mMapper->getCurrentPose());
				_robot2map.write(rbs);
			}
		}catch(std::exception& e)
		{
			mLogger->message(ERROR, (boost::format("Adding scan to map failed: %1%") % e.what()).str());
		}
	}
}

void PointcloudMapper::errorHook()
{
	PointcloudMapperBase::errorHook();
}

void PointcloudMapper::stopHook()
{
	PointcloudMapperBase::stopHook();
}

void PointcloudMapper::cleanupHook()
{
	PointcloudMapperBase::cleanupHook();
	delete mMapper;
	delete mPclSensor;
	if(mOdometry)
		delete mOdometry;
	delete mSolver;
	delete mLogger;
	delete mClock;
}
