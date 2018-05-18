#ifndef SLIPPAGE_ESTIMATOR_TASK_TASK_HPP
#define SLIPPAGE_ESTIMATOR_TASK_TASK_HPP

#include "slippage_estimator/TaskBase.hpp"
#include <cmath>
#include <base-logging/Logging.hpp>

#define BASE_LOG_DISABLE
#define BASE_LOG_NAMESPACE slippage_estimator

namespace slippage_estimator{
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        std::vector<double> slipRatioBuffer;
        int integrationWindowSize;
        int turnSpotWindowSize;
        int wheelWalkingWindowSize;
        int bufferIndex; // current position in the circular buffer of slip ratios

        base::samples::RigidBodyState pose;
        base::samples::RigidBodyState previousPose;
        base::commands::Motion2D motionCommand;

        double calcDeltaPose(base::samples::RigidBodyState,base::samples::RigidBodyState);
        double calcDeltaTime(base::samples::RigidBodyState,base::samples::RigidBodyState);
	bool isPoseValid(base::samples::RigidBodyState);
	
	double ww_velocity;
	double pose_samples_period;
	locomotion_switcher::LocomotionMode locomotion_mode;

        bool turn_spot;
        bool wheel_walking;
        bool havePreviousPose; // true iff at least two pose samples were read
        bool haveMotionCommand; // true iff we ever read a motion command

        int counter;

    public:
        Task(std::string const& name = "slippage_estimator::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);

	~Task();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif
