#ifndef SLAM3D_COMMON_HPP
#define SLAM3D_COMMON_HPP

#include <sys/time.h>
#include <base/Pose.hpp>
#include <base/Time.hpp>
#include <slam3d/Sensor.hpp>

namespace slam3d
{
	long timevaldiff(const timeval& start, const timeval& end);
	
	base::Time timeval2time(const timeval& tv);
	
	Transform pose2transform(const base::Pose& pose);
}

#endif
