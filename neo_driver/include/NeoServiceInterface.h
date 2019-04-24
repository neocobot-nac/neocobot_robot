#include <dlfcn.h>
#include "NeoServiceMetaType.h"

typedef NEOCStatus(*ServiceLoginType)(const char*, short int);
typedef NEOCStatus(*ServiceLogoutType)();
typedef NEOCStatus(*RobotSetupType)(const char*, unsigned int);
typedef NEOCStatus(*RobotShutdownType)();
typedef NEOCStatus(*RobotMoveToAnglesType)(size_t, const int*, const double*, const double, const double, unsigned int);
typedef NEOCStatus(*RobotMoveToPoseType)(Pose, const double, const double, unsigned int);
typedef NEOCStatus(*RobotWaitForJointsType)(size_t, const int*);
typedef NEOCStatus(*RobotMoveJType)(size_t, Pose*, const double, const double, const double, unsigned int);
typedef NEOCStatus(*RobotMoveLType)(size_t, Pose*, const double*, const double, const double, const double, unsigned int);
typedef NEOCStatus(*RobotMovePType)(size_t, Pose*, const double, const double, const double, unsigned int);
typedef NEOCStatus(*RobotReleaseType)(size_t, const int*);
typedef NEOCStatus(*RobotHoldType)(size_t, const int*);
typedef NEOCStatus(*RobotStopType)();
typedef NEOCStatus(*RobotRecoverType)();
typedef NEOCStatus(*RobotCalibrateType)();
typedef NEOCStatus(*RobotResetType)();
typedef NEOCStatus(*RobotGetJointIdsType)(size_t* , int*);
typedef NEOCStatus(*RobotGetAnglesType)(size_t, const int*, double*);
typedef NEOCStatus(*RobotGetVelocityType)(size_t, const int*, double*);
typedef NEOCStatus(*RobotGetCurrentType)(size_t, const int*, double*);
typedef NEOCStatus(*RobotGetPoseType)(Pose*);
typedef NEOCStatus(*RobotForwardKinematicType)(const double*, Pose*);
typedef NEOCStatus(*RobotInverseKinematicType)(Pose , double*);
typedef NEOCStatus(*RobotGetInputType)(size_t, const int*, int*);
typedef NEOCStatus(*RobotSetOutputType)(size_t, const int*, const int*);
typedef NEOCStatus(*RobotSetEOATActionType)(const char*, const char*);

class NeoServiceInterface
{
private:
    const char* NeoSI_lib = "libneoserviceinterface.so";
    void* NeoSI = NULL;

public:
    ServiceLoginType            ServiceLoginFunc;
    ServiceLogoutType           ServiceLogoutFunc;
    RobotSetupType              RobotSetupFunc;
    RobotShutdownType           RobotShutdownFunc;
    RobotMoveToAnglesType       RobotMoveToAnglesFunc;
    RobotMoveToPoseType         RobotMoveToPoseFunc;
    RobotWaitForJointsType      RobotWaitForJointsFunc;
    RobotMoveJType              RobotMoveJFunc;
    RobotMoveLType              RobotMoveLFunc;
    RobotMovePType              RobotMovePFunc;
    RobotReleaseType            RobotReleaseFunc;
    RobotHoldType               RobotHoldFunc;
    RobotStopType               RobotStopFunc;
    RobotRecoverType            RobotRecoverFunc;
    RobotCalibrateType          RobotCalibrateFunc;
    RobotResetType              RobotResetFunc;
    RobotGetJointIdsType        RobotGetJointIdsFunc;
    RobotGetAnglesType          RobotGetAnglesFunc;
    RobotGetVelocityType        RobotGetVelocityFunc;
    RobotGetCurrentType         RobotGetCurrentFunc;
    RobotGetPoseType            RobotGetPoseFunc;
    RobotForwardKinematicType   RobotForwardKinematicFunc;
    RobotInverseKinematicType   RobotInverseKinematicFunc;
    RobotGetInputType           RobotGetInputFunc;
    RobotSetOutputType          RobotSetOutputFunc;
    RobotSetEOATActionType      RobotSetEOATActionFunc;

public:
    NeoServiceInterface(){}
    ~NeoServiceInterface(){}
    bool load();
    void unload();
};

bool NeoServiceInterface::load()
{
    if((NeoSI = dlopen(NeoSI_lib, RTLD_NOW)) == NULL)
    {
        return false;
    }

    ServiceLoginFunc = (ServiceLoginType)dlsym(NeoSI, "SI_ServiceLogin");
    if(ServiceLoginFunc == NULL)
    {
        return false;
    }

    ServiceLogoutFunc = (ServiceLogoutType)dlsym(NeoSI, "SI_ServiceLogout");
    if(ServiceLogoutFunc == NULL)
    {
        return false;
    }

    RobotSetupFunc = (RobotSetupType)dlsym(NeoSI, "SI_RobotSetup");
    if(RobotSetupFunc == NULL)
    {
        return false;
    }

    RobotShutdownFunc = (RobotShutdownType)dlsym(NeoSI, "SI_RobotShutdown");
    if(RobotShutdownFunc == NULL)
    {
        return false;
    }

    RobotMoveToAnglesFunc = (RobotMoveToAnglesType)dlsym(NeoSI, "SI_RobotMoveToAngles");
    if(RobotMoveToAnglesFunc == NULL)
    {
        return false;
    }

    RobotMoveToPoseFunc = (RobotMoveToPoseType)dlsym(NeoSI, "SI_RobotMoveToPose");
    if(RobotMoveToPoseFunc == NULL)
    {
        return false;
    }

    RobotWaitForJointsFunc = (RobotWaitForJointsType)dlsym(NeoSI, "SI_RobotWaitForJoints");
    if(RobotWaitForJointsFunc == NULL)
    {
        return false;
    }

    RobotMoveJFunc = (RobotMoveJType)dlsym(NeoSI, "SI_RobotMoveJ");
    if(RobotMoveJFunc == NULL)
    {
        return false;
    }

    RobotMoveLFunc = (RobotMoveLType)dlsym(NeoSI, "SI_RobotMoveL");
    if(RobotMoveLFunc == NULL)
    {
        return false;
    }

    RobotMovePFunc = (RobotMovePType)dlsym(NeoSI, "SI_RobotMoveP");
    if(RobotMovePFunc == NULL)
    {
        return false;
    }

    RobotReleaseFunc = (RobotReleaseType)dlsym(NeoSI, "SI_RobotRelease");
    if(RobotReleaseFunc == NULL)
    {
        return false;
    }

    RobotHoldFunc = (RobotHoldType)dlsym(NeoSI, "SI_RobotHold");
    if(RobotHoldFunc == NULL)
    {
        return false;
    }

    RobotStopFunc = (RobotStopType)dlsym(NeoSI, "SI_RobotStop");
    if(RobotStopFunc == NULL)
    {
        return false;
    }

    RobotRecoverFunc = (RobotRecoverType)dlsym(NeoSI, "SI_RobotRecover");
    if(RobotRecoverFunc == NULL)
    {
        return false;
    }

    RobotCalibrateFunc = (RobotCalibrateType)dlsym(NeoSI, "SI_RobotCalibrate");
    if(RobotCalibrateFunc == NULL)
    {
        return false;
    }

    RobotResetFunc = (RobotResetType)dlsym(NeoSI, "SI_RobotReset");
    if(RobotResetFunc == NULL)
    {
        return false;
    }

    RobotGetJointIdsFunc = (RobotGetJointIdsType)dlsym(NeoSI, "SI_RobotGetJointIds");
    if(RobotGetJointIdsFunc == NULL)
    {
        return false;
    }

    RobotGetAnglesFunc = (RobotGetAnglesType)dlsym(NeoSI, "SI_RobotGetAngles");
    if(RobotGetAnglesFunc == NULL)
    {
        return false;
    }

    RobotGetVelocityFunc = (RobotGetVelocityType)dlsym(NeoSI, "SI_RobotGetVelocity");
    if(RobotGetVelocityFunc == NULL)
    {
        return false;
    }

    RobotGetCurrentFunc = (RobotGetCurrentType)dlsym(NeoSI, "SI_RobotGetCurrent");
    if(RobotGetCurrentFunc == NULL)
    {
        return false;
    }

    RobotGetPoseFunc = (RobotGetPoseType)dlsym(NeoSI, "SI_RobotGetPose");
    if(RobotGetPoseFunc == NULL)
    {
        return false;
    }

    RobotForwardKinematicFunc = (RobotForwardKinematicType)dlsym(NeoSI, "SI_RobotForwardKinematic");
    if(RobotForwardKinematicFunc == NULL)
    {
        return false;
    }

    RobotInverseKinematicFunc = (RobotInverseKinematicType)dlsym(NeoSI, "SI_RobotInverseKinematic");
    if(RobotInverseKinematicFunc == NULL)
    {
        return false;
    }

    RobotGetInputFunc = (RobotGetInputType)dlsym(NeoSI, "SI_RobotGetInput");
    if(RobotGetInputFunc == NULL)
    {
        return false;
    }

    RobotSetOutputFunc = (RobotSetOutputType)dlsym(NeoSI, "SI_RobotSetOutput");
    if(RobotSetOutputFunc == NULL)
    {
        return false;
    }

    RobotSetEOATActionFunc = (RobotSetEOATActionType)dlsym(NeoSI, "SI_RobotSetEOATAction");
    if(RobotSetEOATActionFunc == NULL)
    {
        return false;
    }

    return true;
}

void NeoServiceInterface::unload()
{
    if(NeoSI != NULL)
    {
       dlclose(NeoSI);
    }
}


