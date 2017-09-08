#include "Task.hpp"

using namespace slippage_estimator;
namespace LM = locomotion_switcher;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    integrationWindowSize = _integration_window.value();
    slipRatioBuffer = std::vector<double>(integrationWindowSize,0.0);
    bufferIndex = 0;
    havePreviousPose = false;
    haveMotionCommand= false;
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    if (_pose.read(pose) == RTT::NewData)
    {
        // in the beginning it might happen, that we do not have a motion command or previousPose
        if (havePreviousPose && haveMotionCommand)
        {
            double deltaPose = calcDeltaPose(pose,previousPose);
            double deltaTime = calcDeltaTime(pose,previousPose);
            if (motionCommand.translation != 0.0)
            {
                slipRatioBuffer[bufferIndex] = deltaPose/(motionCommand.translation * deltaTime);
                bufferIndex = (bufferIndex+1)%integrationWindowSize;
            }
        }
        previousPose = pose;
        havePreviousPose = true;
        if (_motion_command.read(motionCommand) == RTT::NewData)
        {
            haveMotionCommand = true;
        }
    }

    double slipRatio = 0.0;
    for (int i=0; i < integrationWindowSize; i++)
    {
        slipRatio += slipRatioBuffer[i];
    }
    _slip_ratio.write(slipRatio/double(integrationWindowSize));
}

double Task::calcDeltaPose(base::samples::RigidBodyState cur, base::samples::RigidBodyState prev)
{
    return sqrt(pow(cur.position[0] - prev.position[0],2) + pow(cur.position[1] - prev.position[1],2) + pow(cur.position[2] - prev.position[2],2));
}

double Task::calcDeltaTime(base::samples::RigidBodyState cur, base::samples::RigidBodyState prev)
{
    return cur.time.toSeconds() - cur.time.toSeconds();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
