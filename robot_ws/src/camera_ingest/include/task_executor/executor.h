#ifndef NSSC_TASK_EXECUTOR_
#define NSSC_TASK_EXECUTOR_

#include "cli.h"
#include "nssc_errors.h"
#include "node.h"

class Executor : public NSSC_ERRORS
{
    public:
        Executor(std::shared_ptr<NSSC> &node);
        void startCLI();
        void stopCLI();
    
    private:
        std::shared_ptr<NSSC> node;
        std::unique_ptr<CLI> cli;
};

#endif  //NSSC_TASK_EXECUTOR_