#ifndef SLAM3D_BASE_LOGGER_HPP
#define SLAM3D_BASE_LOGGER_HPP

#include <slam3d/core/Logger.hpp>

namespace slam3d
{
	class BaseLogger : public Logger
	{
	public:
		BaseLogger();
		~BaseLogger();
		
		virtual void message(LOG_LEVEL lvl, const std::string& msg);
	};
}

#endif
