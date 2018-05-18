#include "Task.hpp"

using namespace slippage_estimator;
using namespace locomotion_switcher;

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
    if (!TaskBase::configureHook())
        return false;
    integrationWindowSize = _integration_window.value();
    turnSpotWindowSize = _turn_spot_window.value();
    wheelWalkingWindowSize = _ww_window.value();
    ww_velocity = _ww_body_velocity.value(); // [m/s] translational body velocity in WW mode
    pose_samples_period = _pose_samples_period.value(); // 10Hz with vicon. With viso2 it is alternating between 0.266 and 0.533. Why??
    locomotion_mode=DRIVING;
    slipRatioBuffer.resize(integrationWindowSize);
    for (int i=0; i<integrationWindowSize;i ++)
    {
        slipRatioBuffer[i]=0.0;
    }
    bufferIndex = 0;
    havePreviousPose = false;
    haveMotionCommand= false;
    return true;
}
bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    _locomotion_mode.read(locomotion_mode);

    if (_pose.read(pose) == RTT::NewData)
    {
        if (isPoseValid(pose))
        {
            // in the beginning it might happen, that we do not have a motion command or previousPose
            if (havePreviousPose && haveMotionCommand)
            {
                double deltaPose = calcDeltaPose(pose,previousPose);
                double deltaTime = calcDeltaTime(pose,previousPose);
                LOG_DEBUG_S << "deltaPose: " << deltaPose << "  deltaTime: " << deltaTime;
                // pose samples should arrive at a certain frequency.
                // If we get a burst of pose samples with almost zero delta time,
                // it's better not to make slip estimations, it will be noise divided by zero.
                if (0.5*pose_samples_period < deltaTime && deltaTime < 3*pose_samples_period)
                {
                    if (locomotion_mode == DRIVING)
                    {
                        if (motionCommand.translation != 0.0)
                        {
                            // Transitioning from Spot Turn or Wheel Walking to normal driving.
                            // Reduce the counter and unset the flag if enough time is passed.
                            if (turn_spot || wheel_walking)
                            {
                                counter--;
                                if (counter < 1)
                                {
                                    turn_spot = false;
                                    wheel_walking = false;
                                }
                            }
                            else
                            {
                                double slip = 1 - deltaPose/(fabs(motionCommand.translation) * deltaTime);
                                LOG_DEBUG_S << "DR slip: " << slip;
                                slip = (slip < 0 ? 0 : slip);
                                slipRatioBuffer[bufferIndex] = slip;
                                bufferIndex = (bufferIndex+1)%integrationWindowSize;
                            }
                        }
                        else if (motionCommand.rotation != 0.0)
                        {
                            turn_spot = true;
                            counter = turnSpotWindowSize;
                            // We don't calculate slip in Turn Spot manoeuvres.
                        }
                    }
                    else if (locomotion_mode == WHEEL_WALKING)
                    {
                        // reset slip buffer if transitioning to wheel walking
                        if (!wheel_walking)
                        {
                            for (int i=0; i<integrationWindowSize;i ++)
                            {
                                slipRatioBuffer[i]=0.0;
                            }
                            bufferIndex = 0;

                            wheel_walking = true;
                            counter = wheelWalkingWindowSize;
                        }

                        if (motionCommand.translation != 0.0)
                        {
                            double slip = 1 - deltaPose/(ww_velocity * deltaTime);
                            LOG_DEBUG_S << "WW slip: " << slip;
                            slip = (slip < 0 ? 0 : slip);
                            slipRatioBuffer[bufferIndex] = slip;
                            bufferIndex = (bufferIndex+1)%integrationWindowSize;
                        }
                    }
                }
            }
            previousPose = pose;
            havePreviousPose = true;
            if (_motion_command.read(motionCommand) == RTT::NewData)
            {
                haveMotionCommand = true;
            }
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
    return (double(cur.time.toMilliseconds() - prev.time.toMilliseconds())/1000.0);
}

bool Task::isPoseValid(base::samples::RigidBodyState pose)
{
    if ( std::isnan(pose.position(0)) || std::isnan(pose.position(1)) || std::isnan(pose.position(2)) )
        return false;
    return true;
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
