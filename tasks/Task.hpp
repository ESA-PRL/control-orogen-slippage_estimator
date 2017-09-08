#ifndef SLIPPAGE_ESTIMATOR_TASK_TASK_HPP
#define SLIPPAGE_ESTIMATOR_TASK_TASK_HPP

#include "slippage_estimator/TaskBase.hpp"

namespace slippage_estimator{
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

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
