#include "BaseLogger.hpp"

#include <base-logging/Logging.hpp>

using namespace slam3d;

BaseLogger::BaseLogger() : Logger(Clock())
{
}

BaseLogger::~BaseLogger()
{	
}

void BaseLogger::message(LOG_LEVEL lvl, const std::string& msg)
{
	switch(lvl)
	{
		case DEBUG:
			LOG_DEBUG("%s", msg.c_str());
			break;
		case INFO:
			LOG_INFO("%s", msg.c_str());
			break;
		case WARNING:
			LOG_WARN("%s", msg.c_str());
			break;
		case ERROR:
			LOG_ERROR("%s", msg.c_str());
			break;
		case FATAL:
			LOG_FATAL("%s", msg.c_str());
			break;
	}
}