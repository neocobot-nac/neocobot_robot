#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <log4cplus/logger.h>
#include <log4cplus/configurator.h> 
#include <log4cplus/layout.h> 
#include <log4cplus/loggingmacros.h> 
#include <log4cplus/helpers/stringhelper.h> 

using namespace std;
using namespace log4cplus;
using namespace log4cplus::helpers;

#ifdef _WIN32

#define CONFIG_FILE_DIR "C:/2.workspace/pycharm/2.Project/7.Server/NeoServer/OdinServer/test/QtTest/OdinServerQtTest/OdinServerQtTest/logconfig.properties"
#else

#define CONFIG_FILE_DIR "/home/eaibot/logconfig.properties"
#endif

class MyLogger
{
public:
	static MyLogger * getInstance();
	Logger logger;

private:
	MyLogger();
	~MyLogger();
	static MyLogger * my_logger;

};

#endif
