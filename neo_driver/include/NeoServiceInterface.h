#include <dlfcn.h>
#include "NeoServiceMetaType.h"

typedef NEOCStatus(*serviceLoginType)(const char*, short int);
typedef NEOCStatus(*serviceLogoutType)();
typedef NEOCStatus(*robotSetupType)(const char*, SetupParam, SetupMode);
typedef NEOCStatus(*robotShutdownType)();
typedef NEOCStatus(*robotMoveToAnglesType)(size_t, const int*, const double*, float, float, RelativeMode);
typedef NEOCStatus(*robotMoveToPoseType)(Pose, float, float, RelativeMode);
typedef NEOCStatus(*robotWaitForJointType)(size_t, const int*);
typedef NEOCStatus(*robotMoveJType)(size_t, Pose*, float, float, float, TrajCloseMode);
typedef NEOCStatus(*robotMoveLType)(size_t, Pose*, const double*, float, float, float, TrajCloseMode);
typedef NEOCStatus(*robotMovePType)(size_t, Pose*, float, float, float, TrajCloseMode);
typedef NEOCStatus(*robotReleaseType)(size_t, const int*);
typedef NEOCStatus(*robotHoldType)(size_t, const int*);
typedef NEOCStatus(*robotStopType)();
typedef NEOCStatus(*robotRecoverType)();
typedef NEOCStatus(*robotCalibrateType)();
typedef NEOCStatus(*robotResetType)();
typedef NEOCStatus(*robotGetJointIdsType)(size_t&, int*);
typedef NEOCStatus(*robotGetAnglesType)(size_t, const int*, double*);
typedef NEOCStatus(*robotGetVelocityType)(size_t, const int*, double*);
typedef NEOCStatus(*robotGetCurrentType)(size_t, const int*, double*);
typedef NEOCStatus(*robotGetPoseType)(Pose&);
typedef NEOCStatus(*robotForwardKinematicType)(const double*, Pose&);
typedef NEOCStatus(*robotInverseKinematicType)(Pose , double*);
typedef NEOCStatus(*robotGetInputType)(size_t, const int*, int*);
typedef NEOCStatus(*robotSetOutputType)(size_t, const int*, const int*);
typedef NEOCStatus(*robotSetEOATActionType)(const char*, const char*);

class NeoServiceInterface
{
private:
    const char* libname = "libneoserviceinterface.so";
    void* libcb = NULL;

public:
    serviceLoginType            serviceLoginFunc;
    serviceLogoutType           serviceLogoutFunc;
    robotSetupType              robotSetupFunc;
    robotShutdownType           robotShutdownFunc;
    robotMoveToAnglesType       robotMoveToAnglesFunc;
    robotMoveToPoseType         robotMoveToPoseFunc;
    robotWaitForJointType       robotWaitForJointFunc;
    robotMoveJType              robotMoveJFunc;
    robotMoveLType              robotMoveLFunc;
    robotMovePType              robotMovePFunc;
    robotReleaseType            robotReleaseFunc;
    robotHoldType               robotHoldFunc;
    robotStopType               robotStopFunc;
    robotRecoverType            robotRecoverFunc;
    robotCalibrateType          robotCalibrateFunc;
    robotResetType              robotResetFunc;
    robotGetJointIdsType        robotGetJointIdsFunc;
    robotGetAnglesType          robotGetAnglesFunc;
    robotGetVelocityType        robotGetVelocityFunc;
    robotGetCurrentType         robotGetCurrentFunc;
    robotGetPoseType            robotGetPoseFunc;
    robotForwardKinematicType   robotForwardKinematicFunc;
    robotInverseKinematicType   robotInverseKinematicFunc;
    robotGetInputType           robotGetInputFunc;
    robotSetOutputType          robotSetOutputFunc;
    robotSetEOATActionType      robotSetEOATActionFunc;

public:
    NeoServiceInterface(){}
    ~NeoServiceInterface(){}
    bool load();
    void unload();
};

bool NeoServiceInterface::load()
{
    libcb = dlopen(libname, RTLD_NOW);
    if((libcb = dlopen(libname, RTLD_NOW)) == NULL)
    {
        return false;
    }

    serviceLoginFunc = (serviceLoginType)dlsym(libcb, "serviceLogin");
    if(serviceLoginFunc == NULL)
    {
        return false;
    }

    serviceLogoutFunc = (serviceLogoutType)dlsym(libcb, "serviceLogout");
    if(serviceLogoutFunc == NULL)
    {
        return false;
    }

    robotSetupFunc = (robotSetupType)dlsym(libcb, "robotSetup");
    if(robotSetupFunc == NULL)
    {
        return false;
    }

    robotShutdownFunc = (robotShutdownType)dlsym(libcb, "robotShutdown");
    if(robotShutdownFunc == NULL)
    {
        return false;
    }

    robotMoveToAnglesFunc = (robotMoveToAnglesType)dlsym(libcb, "robotMoveToAngles");
    if(robotMoveToAnglesFunc == NULL)
    {
        return false;
    }

    robotMoveToPoseFunc = (robotMoveToPoseType)dlsym(libcb, "robotMoveToPose");
    if(robotMoveToPoseFunc == NULL)
    {
        return false;
    }

    robotWaitForJointFunc = (robotWaitForJointType)dlsym(libcb, "robotWaitForJoint");
    if(robotWaitForJointFunc == NULL)
    {
        return false;
    }

    robotMoveJFunc = (robotMoveJType)dlsym(libcb, "robotMoveJ");
    if(robotMoveJFunc == NULL)
    {
        return false;
    }

    robotMoveLFunc = (robotMoveLType)dlsym(libcb, "robotMoveL");
    if(robotMoveLFunc == NULL)
    {
        return false;
    }

    robotMovePFunc = (robotMovePType)dlsym(libcb, "robotMoveP");
    if(robotMovePFunc == NULL)
    {
        return false;
    }

    robotReleaseFunc = (robotReleaseType)dlsym(libcb, "robotRelease");
    if(robotReleaseFunc == NULL)
    {
        return false;
    }

    robotHoldFunc = (robotHoldType)dlsym(libcb, "robotHold");
    if(serviceLoginFunc == NULL)
    {
        return false;
    }

    robotStopFunc = (robotStopType)dlsym(libcb, "robotStop");
    if(serviceLoginFunc == NULL)
    {
        return false;
    }

    robotRecoverFunc = (robotRecoverType)dlsym(libcb, "robotRecover");
    if(serviceLoginFunc == NULL)
    {
        return false;
    }

    robotCalibrateFunc = (robotCalibrateType)dlsym(libcb, "robotCalibrate");
    if(robotCalibrateFunc == NULL)
    {
        return false;
    }

    robotResetFunc = (robotResetType)dlsym(libcb, "robotReset");
    if(robotResetFunc == NULL)
    {
        return false;
    }

    robotGetJointIdsFunc = (robotGetJointIdsType)dlsym(libcb, "robotGetJointIds");
    if(robotGetJointIdsFunc == NULL)
    {
        return false;
    }

    robotGetAnglesFunc = (robotGetAnglesType)dlsym(libcb, "robotGetAngles");
    if(robotGetAnglesFunc == NULL)
    {
        return false;
    }

    robotGetVelocityFunc = (robotGetVelocityType)dlsym(libcb, "robotGetVelocity");
    if(robotGetVelocityFunc == NULL)
    {
        return false;
    }

    robotGetCurrentFunc = (robotGetCurrentType)dlsym(libcb, "robotGetCurrent");
    if(robotGetCurrentFunc == NULL)
    {
        return false;
    }

    robotGetPoseFunc = (robotGetPoseType)dlsym(libcb, "robotGetPose");
    if(robotGetPoseFunc == NULL)
    {
        return false;
    }

    robotForwardKinematicFunc = (robotForwardKinematicType)dlsym(libcb, "robotForwardKinematic");
    if(robotForwardKinematicFunc == NULL)
    {
        return false;
    }

    robotInverseKinematicFunc = (robotInverseKinematicType)dlsym(libcb, "robotInverseKinematic");
    if(robotInverseKinematicFunc == NULL)
    {
        return false;
    }

    robotGetInputFunc = (robotGetInputType)dlsym(libcb, "robotGetInput");
    if(robotGetInputFunc == NULL)
    {
        return false;
    }

    robotSetOutputFunc = (robotSetOutputType)dlsym(libcb, "robotSetOutput");
    if(robotSetOutputFunc == NULL)
    {
        return false;
    }

    robotSetEOATActionFunc = (robotSetEOATActionType)dlsym(libcb, "robotSetEOATAction");
    if(robotSetEOATActionFunc == NULL)
    {
        return false;
    }

    return true;
}

void NeoServiceInterface::unload()
{
    if(libcb != NULL)
    {
       dlclose(libcb);
    }
}


