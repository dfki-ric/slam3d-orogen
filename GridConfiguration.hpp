#ifndef SLAM3D_GRID_CONFIGURATION_HPP
#define SLAM3D_GRID_CONFIGURATION_HPP

namespace slam3d
{
	struct GridConfiguration
	{
		double min_x;
		double max_x;
		double min_y;
		double max_y;
		double min_z;
		double max_z;
		double resolution;
		
		GridConfiguration() : min_x(-10), max_x(10),
		                      min_y(-10), max_y(10),
		                      min_z(-10), max_z(10),
		                      resolution(0.1) {}
	};
}

#endif
