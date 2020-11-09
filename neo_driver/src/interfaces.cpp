#include "interfaces.h"


bool Interfaces::load()
{
    if((NeoSI = dlopen(NeoSI_lib, RTLD_NOW)) == NULL)
    {
        return false;
    }
    return true;
}

bool Interfaces::unload()
{
    if(NeoSI != NULL)
    {
        dlclose(NeoSI);
        return true;
    }
    return false;
}

NEOStatus Interfaces::ServiceLogin(const char* ip, short int port, const char* serial, unsigned int timeout, int transport)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    ServiceLoginType ServiceLoginFunc = (ServiceLoginType)dlsym(NeoSI, "ServiceLogin");
    if(ServiceLoginFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return ServiceLoginFunc(ip, port, serial, timeout, transport);
}


NEOStatus Interfaces::ServiceLogout()
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    ServiceLogoutType ServiceLogoutFunc = (ServiceLogoutType)dlsym(NeoSI, "ServiceLogout");
    if(ServiceLogoutFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return ServiceLogoutFunc();
}


NEOStatus Interfaces::RobotSetup(const char* name, unsigned int mode)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    RobotSetupType RobotSetupFunc = (RobotSetupType)dlsym(NeoSI, "RobotSetup");
    if(RobotSetupFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return RobotSetupFunc(name, mode);
}


NEOStatus Interfaces::RobotShutdown()
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    RobotShutdownType RobotShutdownFunc = (RobotShutdownType)dlsym(NeoSI, "RobotShutdown");
    if(RobotShutdownFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return RobotShutdownFunc();
}


NEOStatus Interfaces::Calibrate()
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    CalibrateType CalibrateFunc = (CalibrateType)dlsym(NeoSI, "Calibrate");
    if(CalibrateFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return CalibrateFunc();
}


int Interfaces::IsCalibrating()
{
    if(NeoSI == NULL)
    {
        return 0;
    }

    IsCalibratingType IsCalibratingFunc = (IsCalibratingType)dlsym(NeoSI, "IsCalibrating");
    if(IsCalibratingFunc == NULL)
    {
        return 0;
    }
    return IsCalibratingFunc();
}


NEOStatus Interfaces::Release(size_t size, const int* joint_ids)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    ReleaseType ReleaseFunc = (ReleaseType)dlsym(NeoSI, "Release");
    if(ReleaseFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return ReleaseFunc(size, joint_ids);
}


NEOStatus Interfaces::Hold(size_t size, const int* joint_ids)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    HoldType HoldFunc = (HoldType)dlsym(NeoSI, "Hold");
    if(HoldFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return HoldFunc(size, joint_ids);
}


NEOStatus Interfaces::Stop()
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    StopType StopFunc = (StopType)dlsym(NeoSI, "Stop");
    if(StopFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return StopFunc();
}


NEOStatus Interfaces::Halt()
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    HaltType HaltFunc = (HaltType)dlsym(NeoSI, "Halt");
    if(HaltFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return HaltFunc();
}


NEOStatus Interfaces::Pause()
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    PauseType PauseFunc = (PauseType)dlsym(NeoSI, "Pause");
    if(PauseFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return PauseFunc();
}


NEOStatus Interfaces::Resume()
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    ResumeType ResumeFunc = (ResumeType)dlsym(NeoSI, "Resume");
    if(ResumeFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return ResumeFunc();
}


NEOStatus Interfaces::Recover()
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    RecoverType RecoverFunc = (RecoverType)dlsym(NeoSI, "Recover");
    if(RecoverFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return RecoverFunc();
}

NEOStatus Interfaces::MoveJoints(size_t size, const int* joint_ids, const double* joints, const double velocity, const double acceleration, unsigned int mode)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    MoveJointsType MoveJointsFunc = (MoveJointsType)dlsym(NeoSI, "MoveJoints");
    if(MoveJointsFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return MoveJointsFunc(size, joint_ids, joints, velocity, acceleration, mode);
}


NEOStatus Interfaces::MoveEndpos(Pose pose, const double velocity, const double acceleration, unsigned int mode)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    MoveEndposType MoveEndposFunc = (MoveEndposType)dlsym(NeoSI, "MoveEndpos");
    if(MoveEndposFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return MoveEndposFunc(pose, velocity, acceleration, mode);
}


NEOStatus Interfaces::WaitForJoints(size_t size, const int* joint_ids)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    WaitForJointsType WaitForJointsFunc = (WaitForJointsType)dlsym(NeoSI, "WaitForJoints");
    if(WaitForJointsFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return WaitForJointsFunc(size, joint_ids);
}


NEOStatus Interfaces::MoveJ(size_t size, DoubleArray* points, const double velocity, const double acceleration, const double interval, unsigned int mode, int loop)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    MoveJType MoveJFunc = (MoveJType)dlsym(NeoSI, "MoveJ");
    if(MoveJFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return MoveJFunc(size, points, velocity, acceleration, interval, mode, loop);
}


NEOStatus Interfaces::MoveL(size_t size, Pose* points, const double velocity, const double acceleration, const double interval, unsigned int mode, int loop)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    MoveLType MoveLFunc = (MoveLType)dlsym(NeoSI, "MoveL");
    if(MoveLFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return MoveLFunc(size, points, velocity, acceleration, interval, mode, loop);
}


NEOStatus Interfaces::MoveP(size_t size, Pose* points, const double velocity, const double acceleration, const double interval, unsigned int mode, int loop)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    MovePType MovePFunc = (MovePType)dlsym(NeoSI, "MoveP");
    if(MovePFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return MovePFunc(size, points, velocity, acceleration, interval, mode, loop);
}


NEOStatus Interfaces::GetJointIds(size_t* size, int* joint_ids)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    GetJointIdsType GetJointIdsFunc = (GetJointIdsType)dlsym(NeoSI, "GetJointIds");
    if(GetJointIdsFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return GetJointIdsFunc(size, joint_ids);
}


int Interfaces::GetInnerStatus()
{
    if(NeoSI == NULL)
    {
        return 0;
    }

    GetInnerStatusType GetInnerStatusFunc = (GetInnerStatusType)dlsym(NeoSI, "GetInnerStatus");
    if(GetInnerStatusFunc == NULL)
    {
        return 0;
    }
    return GetInnerStatusFunc();
}


NEOStatus Interfaces::GetJoints(size_t size, const int* joint_ids, double* joints)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    GetJointsType GetJointsFunc = (GetJointsType)dlsym(NeoSI, "GetJoints");
    if(GetJointsFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return GetJointsFunc(size, joint_ids, joints);
}


NEOStatus Interfaces::GetEndpos(Pose* pose)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    GetEndposType GetEndposFunc = (GetEndposType)dlsym(NeoSI, "GetEndpos");
    if(GetEndposFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return GetEndposFunc(pose);
}


NEOStatus Interfaces::GetVelocity(size_t size, const int* joint_ids, double* velocity)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    GetVelocityType GetVelocityFunc = (GetVelocityType)dlsym(NeoSI, "GetVelocity");
    if(GetVelocityFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return GetVelocityFunc(size, joint_ids, velocity);
}


NEOStatus Interfaces::GetCurrent(size_t size, const int* joint_ids, double* current)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    GetCurrentType GetCurrentFunc = (GetCurrentType)dlsym(NeoSI, "GetCurrent");
    if(GetCurrentFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return GetCurrentFunc(size, joint_ids, current);
}


NEOStatus Interfaces::GetInputIO(size_t size, const int* joint_ids, int* signal)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    GetInputIOType GetInputIOFunc = (GetInputIOType)dlsym(NeoSI, "GetInputIO");
    if(GetInputIOFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return GetInputIOFunc(size, joint_ids, signal);
}


NEOStatus Interfaces::GetJointParameters(size_t size, const int* joint_ids, int name, double* parameters)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    GetJointParametersType GetJointParametersFunc = (GetJointParametersType)dlsym(NeoSI, "GetJointParameters");
    if(GetJointParametersFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return GetJointParametersFunc(size, joint_ids, name, parameters);
}


NEOStatus Interfaces::GetDefaultParameters(const char* name, double* parameters)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    GetDefaultParametersType GetDefaultParametersFunc = (GetDefaultParametersType)dlsym(NeoSI, "GetDefaultParameters");
    if(GetDefaultParametersFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return GetDefaultParametersFunc(name, parameters);
}


NEOStatus Interfaces::SetOutputIO(size_t size, const int* joint_ids, const int* signal)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    SetOutputIOType SetOutputIOFunc = (SetOutputIOType)dlsym(NeoSI, "SetOutputIO");
    if(SetOutputIOFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return SetOutputIOFunc(size, joint_ids, signal);
}


NEOStatus Interfaces::AddMonitorParam(int status)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    AddMonitorParamType AddMonitorParamFunc = (AddMonitorParamType)dlsym(NeoSI, "AddMonitorParam");
    if(AddMonitorParamFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return AddMonitorParamFunc(status);
}


NEOStatus Interfaces::RemoveMonitorParam(int status)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    RemoveMonitorParamType RemoveMonitorParamFunc = (RemoveMonitorParamType)dlsym(NeoSI, "RemoveMonitorParam");
    if(RemoveMonitorParamFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return RemoveMonitorParamFunc(status);
}


NEOStatus Interfaces::Forward(const double* joints, Pose* pose)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    ForwardType ForwardFunc = (ForwardType)dlsym(NeoSI, "Forward");
    if(ForwardFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return ForwardFunc(joints, pose);
}


NEOStatus Interfaces::Inverse(Pose pose, const double* previews, DoubleArray* joints)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    InverseType InverseFunc = (InverseType)dlsym(NeoSI, "Inverse");
    if(InverseFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return InverseFunc(pose, previews, joints);
}


NEOStatus Interfaces::AddEndEffector(const char* name, Pose pose, const double mass, const double* CoM)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    AddEndEffectorType AddEndEffectorFunc = (AddEndEffectorType)dlsym(NeoSI, "AddEndEffector");
    if(AddEndEffectorFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return AddEndEffectorFunc(name, pose, mass, CoM);
}


NEOStatus Interfaces::DeleteEndEffector(const char* name)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    DeleteEndEffectorType DeleteEndEffectorFunc = (DeleteEndEffectorType)dlsym(NeoSI, "DeleteEndEffector");
    if(DeleteEndEffectorFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return DeleteEndEffectorFunc(name);
}


NEOStatus Interfaces::LoadEndEffector(const char* name)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    LoadEndEffectorType LoadEndEffectorFunc = (LoadEndEffectorType)dlsym(NeoSI, "LoadEndEffector");
    if(LoadEndEffectorFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return LoadEndEffectorFunc(name);
}


NEOStatus Interfaces::EmitEESignal(unsigned int signal)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    EmitEESignalType EmitEESignalFunc = (EmitEESignalType)dlsym(NeoSI, "EmitEESignal");
    if(EmitEESignalFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return EmitEESignalFunc(signal);
}


NEOStatus Interfaces::UnloadEndEffector()
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    UnloadEndEffectorType UnloadEndEffectorFunc = (UnloadEndEffectorType)dlsym(NeoSI, "UnloadEndEffector");
    if(UnloadEndEffectorFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return UnloadEndEffectorFunc();
}


NEOStatus Interfaces::SaveEEParameters(const char* name, Pose pose, const double mass, const double* CoM)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    SaveEEParametersType SaveEEParametersFunc = (SaveEEParametersType)dlsym(NeoSI, "SaveEEParameters");
    if(SaveEEParametersFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return SaveEEParametersFunc(name, pose, mass, CoM);
}


NEOStatus Interfaces::GetEEParameters(const char* name, Pose* pose, double* mass, double* CoM)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    GetEEParametersType GetEEParametersFunc = (GetEEParametersType)dlsym(NeoSI, "GetEEParameters");
    if(GetEEParametersFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return GetEEParametersFunc(name, pose, mass, CoM);
}


// NEOStatus Interfaces::GetProjectList(size_t* size, string* list)
// {
//     if(NeoSI == NULL)
//     {
//         return NEOCOBOT_ERROR_INIT;
//     }
// }


NEOStatus Interfaces::LoadProject(const char* name)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    LoadProjectType LoadProjectFunc = (LoadProjectType)dlsym(NeoSI, "LoadProject");
    if(LoadProjectFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return LoadProjectFunc(name);
}


NEOStatus Interfaces::CreateProject(const char* name)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    CreateProjectType CreateProjectFunc = (CreateProjectType)dlsym(NeoSI, "CreateProject");
    if(CreateProjectFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return CreateProjectFunc(name);
}


NEOStatus Interfaces::DeleteProject(const char* name)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    DeleteProjectType DeleteProjectFunc = (DeleteProjectType)dlsym(NeoSI, "DeleteProject");
    if(DeleteProjectFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return DeleteProjectFunc(name);
}


NEOStatus Interfaces::CheckProject(const char* name, int* result)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    CheckProjectType CheckProjectFunc = (CheckProjectType)dlsym(NeoSI, "CheckProject");
    if(CheckProjectFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return CheckProjectFunc(name, result);
}


NEOStatus Interfaces::SavePose(const char* name, const double* pose)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    SavePoseType SavePoseFunc = (SavePoseType)dlsym(NeoSI, "SavePose");
    if(SavePoseFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return SavePoseFunc(name, pose);
}


NEOStatus Interfaces::DeletePose(const char* name)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    DeletePoseType DeletePoseFunc = (DeletePoseType)dlsym(NeoSI, "DeletePose");
    if(DeletePoseFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return DeletePoseFunc(name);
}


NEOStatus Interfaces::GetPose(const char* name, DoubleArray* pose)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    GetPoseType GetPoseFunc = (GetPoseType)dlsym(NeoSI, "GetPose");
    if(GetPoseFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return GetPoseFunc(name, pose);
}


// NEOStatus Interfaces::GetPoseList(size_t* size, string* list)
// {
//     if(NeoSI == NULL)
//     {
//         return NEOCOBOT_ERROR_INIT;
//     }
// }


NEOStatus Interfaces::EnableCollisionProtection(int level)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    EnableCollisionProtectionType EnableCollisionProtectionFunc = (EnableCollisionProtectionType)dlsym(NeoSI, "EnableCollisionProtection");
    if(EnableCollisionProtectionFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return EnableCollisionProtectionFunc(level);
}


NEOStatus Interfaces::SetCollisionProtectionLevel(int level)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    SetCollisionProtectionLevelType SetCollisionProtectionLevelFunc = (SetCollisionProtectionLevelType)dlsym(NeoSI, "SetCollisionProtectionLevel");
    if(SetCollisionProtectionLevelFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return SetCollisionProtectionLevelFunc(level);
}


NEOStatus Interfaces::ForbidCollisionProtection()
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    ForbidCollisionProtectionType ForbidCollisionProtectionFunc = (ForbidCollisionProtectionType)dlsym(NeoSI, "ForbidCollisionProtection");
    if(ForbidCollisionProtectionFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return ForbidCollisionProtectionFunc();
}


NEOStatus Interfaces::EnableKinestheticTeaching(double load)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    EnableKinestheticTeachingType EnableKinestheticTeachingFunc = (EnableKinestheticTeachingType)dlsym(NeoSI, "EnableKinestheticTeaching");
    if(EnableKinestheticTeachingFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return EnableKinestheticTeachingFunc(load);
}


NEOStatus Interfaces::DisableKinestheticTeaching()
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    DisableKinestheticTeachingType DisableKinestheticTeachingFunc = (DisableKinestheticTeachingType)dlsym(NeoSI, "DisableKinestheticTeaching");
    if(DisableKinestheticTeachingFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return DisableKinestheticTeachingFunc();
}


NEOStatus Interfaces::EnableVelocityTuning(int mode)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    EnableVelocityTuningType EnableVelocityTuningFunc = (EnableVelocityTuningType)dlsym(NeoSI, "EnableVelocityTuning");
    if(EnableVelocityTuningFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return EnableVelocityTuningFunc(mode);
}


NEOStatus Interfaces::DisableVelocityTuning()
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    DisableVelocityTuningType DisableVelocityTuningFunc = (DisableVelocityTuningType)dlsym(NeoSI, "DisableVelocityTuning");
    if(DisableVelocityTuningFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return DisableVelocityTuningFunc();
}


NEOStatus Interfaces::SetEndposVelocities(double* endpos_velocities)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    SetEndposVelocitiesType SetEndposVelocitiesFunc = (SetEndposVelocitiesType)dlsym(NeoSI, "SetEndposVelocities");
    if(SetEndposVelocitiesFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return SetEndposVelocitiesFunc(endpos_velocities);
}


NEOStatus Interfaces::SetJointVelocities(size_t size, double* joint_velocities)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    SetJointVelocitiesType SetJointVelocitiesFunc = (SetJointVelocitiesType)dlsym(NeoSI, "SetJointVelocities");
    if(SetJointVelocitiesFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return SetJointVelocitiesFunc(size, joint_velocities);
}


NEOStatus Interfaces::EnablePositionTuning()
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    EnablePositionTuningType EnablePositionTuningFunc = (EnablePositionTuningType)dlsym(NeoSI, "EnablePositionTuning");
    if(EnablePositionTuningFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return EnablePositionTuningFunc();
}


NEOStatus Interfaces::DisablePositionTuning()
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    DisablePositionTuningType DisablePositionTuningFunc = (DisablePositionTuningType)dlsym(NeoSI, "DisablePositionTuning");
    if(DisablePositionTuningFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return DisablePositionTuningFunc();
}


NEOStatus Interfaces::SetTarget(size_t size, double* joints, double* velocities)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    SetTargetType SetTargetFunc = (SetTargetType)dlsym(NeoSI, "SetTarget");
    if(SetTargetFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return SetTargetFunc(size, joints, velocities);
}


NEOStatus Interfaces::GetRotationMatrix(RotationMatrix* rotation_matrix)
{
    if(NeoSI == NULL)
    {
        return NEOCOBOT_ERROR_INIT;
    }

    GetRotationMatrixType GetRotationMatrixFunc = (GetRotationMatrixType)dlsym(NeoSI, "GetRotationMatrix");
    if(GetRotationMatrixFunc == NULL)
    {
        return NEOCOBOT_ERROR_ILLFUNC;
    }
    return GetRotationMatrixFunc(rotation_matrix);
}


