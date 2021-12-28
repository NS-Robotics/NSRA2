#ifndef NSSC_TASK_EXECUTOR_
#define NSSC_TASK_EXECUTOR_

#include "cli.h"
#include "nssc_errors.h"
#include "node.h"
#include "camera_manager.h"
#include "ndi.h"
#include "ingest.h"

class Executor : public NSSC_ERRORS, public CLI
{
    public:
        Executor(std::shared_ptr<NSSC> &node, std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> &node_executor);
        void exit();
        void init();
        void rawNDI();
        void run_ingest();
        void cancel();
    
    private:
        std::shared_ptr<NSSC> node;
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> node_executor;
        std::shared_ptr<cameraManager> camManager;
        std::shared_ptr<NDI> ndi;

        Ingest* ingest;

        bool initialized = false;
        bool rawNDIstream = false;

        std::string msgCaller = "Executor";
};

#endif  //NSSC_TASK_EXECUTOR_