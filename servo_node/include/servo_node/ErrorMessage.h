#ifndef ERRORMESSAGE_HH
#define ERRORMESSAGE_HH

#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <pthread.h>

#include "Timer.h"

#define FANCY_ERROR_MESSAGE

#ifdef FANCY_ERROR_MESSAGE

#define PRINT_INFO(msg)     { std::ostringstream msgStream; msgStream << "[" << __PRETTY_FUNCTION__ << " : " << __LINE__ << "] : " << msg << std::endl; Upenn::ErrorMessage::PrintInfo(msgStream.str()); }
#define PRINT_INFO_RAW(msg) { std::ostringstream msgStream; msgStream << "[" << __PRETTY_FUNCTION__ << " : " << __LINE__ << "] : " << msg << std::endl; Upenn::ErrorMessage::PrintInfo(msgStream.str()); }
#define PRINT_WARNING(msg)  { std::ostringstream msgStream; msgStream << "[" << __PRETTY_FUNCTION__ << " : " << __LINE__ << "] : " << msg << std::endl; Upenn::ErrorMessage::PrintWarning(msgStream.str()); }
#define PRINT_ERROR(msg)    { std::ostringstream msgStream; msgStream << "[" << __PRETTY_FUNCTION__ << " : " << __LINE__ << "] : " << msg << std::endl; Upenn::ErrorMessage::PrintError(msgStream.str()); }

//#define PRINT_INFO( msg )    { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; Upenn::ErrorMessage::PrintInfo(msgStream.str()); }
//#define PRINT_INFO_RAW( msg )    { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; Upenn::ErrorMessage::PrintInfo(msgStream.str()); }
//#define PRINT_WARNING( msg ) { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; Upenn::ErrorMessage::PrintWarning(msgStream.str()); }
//#define PRINT_ERROR( msg )   { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; Upenn::ErrorMessage::PrintError(msgStream.str()); }

#define LOG_MESSAGE( msg ) { }

#define PRINT_INFO_ON Upenn::ErrorMessage::PrintInfoOn()
#define PRINT_INFO_OFF Upenn::ErrorMessage::PrintInfoOff()
#define PRINT_WARNINGS_ON Upenn::ErrorMessage::PrintWarningsOn()
#define PRINT_WARNINGS_OFF Upenn::ErrorMessage::PrintWarningsOff()
#define PRINT_ERRORS_ON Upenn::ErrorMessage::PrintErrorsOn()
#define PRINT_ERRORS_OFF Upenn::ErrorMessage::PrintErrorsOff()

#define LOG_INFO_ON Upenn::ErrorMessage::LogInfoOn()
#define LOG_INFO_OFF Upenn::ErrorMessage::LogInfoOff()
#define LOG_WARNINGS_ON Upenn::ErrorMessage::LogWarningsOn()
#define LOG_WARNINGS_OFF Upenn::ErrorMessage::LogWarningsOff()
#define LOG_ERRORS_ON Upenn::ErrorMessage::LogErrorsOn()
#define LOG_ERRORS_OFF Upenn::ErrorMessage::LogErrosOff()

#else

#define PRINT_INFO( msg ) { std::cout << msg <<std::endl; }
#define PRINT_INFO_RAW( msg ) { std::cout << msg <<std::endl; }
#define PRINT_WARNING( msg ) { std::cout << msg <<std::endl; }
#define PRINT_ERROR( msg ) { std::cout << msg <<std::endl; }

#define PRINT_INFO_ON
#define PRINT_INFO_OFF
#define PRINT_WARNINGS_ON
#define PRINT_WARNINGS_OFF
#define PRINT_ERRORS_ON
#define PRINT_ERRORS_OFF

#define LOG_INFO_ON
#define LOG_INFO_OFF
#define LOG_WARNINGS_ON
#define LOG_WARNINGS_OFF
#define LOG_ERRORS_ON
#define LOG_ERRORS_OFF

#endif

namespace Upenn {

class ErrorMessage
{
public:

    //constructor
    ErrorMessage();

    //destructor
    ~ErrorMessage();

    //print out info message
    static int PrintInfo(std::string msg);

    //print out warning message
    static int PrintWarning(std::string msg);

    //print out error message
    static int PrintError(std::string msg);

    static int PrintInfoOn();
    static int PrintInfoOff();
    static int PrintWarningsOn();
    static int PrintWarningsOff();
    static int PrintErrorsOn();
    static int PrintErrorsOff();

    static int LogInfoOn();
    static int LogInfoOff();
    static int LogWarningsOn();
    static int LogWarningsOff();
    static int LogErrorsOn();
    static int LogErrorsOff();

private:

    //initialize
    static int Initialize();

    static int LockErrorMutex();
    static int UnlockErrorMutex();

    static int InitializeInfoLog();
    static int InitializeWarningLog();
    static int InitializeErrorLog();

    static bool logInfo;
    static bool logWarnings;
    static bool logErrors;
    static bool printInfo;
    static bool printWarnings;
    static bool printErrors;

    static bool initialized;
    static bool initializedInfoLog;
    static bool initializedWarningLog;
    static bool initializedErrorLog;
    static std::string logInfoFileName;
    static std::string logWarningFileName;
    static std::string logErrorFileName;

    static std::ofstream * logInfoStream;
    static std::ofstream * logWarningStream;
    static std::ofstream * logErrorStream;
    static bool threadSafe;
    static pthread_mutex_t errorMutex;
    static Upenn::Timer timer0;
};

} // namespace Upenn

#endif //ERRORMESSAGE_HH
