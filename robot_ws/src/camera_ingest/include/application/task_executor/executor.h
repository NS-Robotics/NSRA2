#ifndef NSSC_TASK_EXECUTOR_
#define NSSC_TASK_EXECUTOR_

#include "cli.h"
#include "nssc_errors.h"
#include "node.h"
#include "camera_manager.h"
#include "ndi.h"
#include "ingest.h"
#include "calibration.h"
#include "triangulation_interface.h"
#include "frame_manager.h"
#include "object_detection.h"

namespace nssc
{
    namespace application
    {
        class Executor : public NSSC_ERRORS, public CLI
        {
        public:
            Executor(std::shared_ptr<ros::NSSC> &node,
                     std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> &node_executor);
            void init();

            void exit() override;
            void toggleNDI(bool mono_stream) override;
            void toggleNDIsource(NSSC_NDI_SEND type) override;
            void runIngest() override;
            void runCalibration(char *setName) override;
            void runTriangulation(char *setName) override;
            void findTriangulationOrigin() override;
            void runDetection() override;
            void setExposure(float exposure_time) override;
            void setGain(float gain) override;
            void cancel() override;

        private:
            std::shared_ptr<ros::NSSC> node;
            std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> node_executor;
            std::shared_ptr<ingest::CameraManager> cam_manager;
            std::shared_ptr<send::NDI> ndi;
            std::unique_ptr<send::FrameManager> frame_manager;

            stereocalibration::Ingest *ingest;
            stereocalibration::Calibration *calibration;
            std::shared_ptr<process::TriangulationInterface> triangulation_interface;
            std::unique_ptr<process::ObjectDetection> object_detection;

            bool ndi_initialized = false;
            bool cam_manager_initialized = false;
            bool ndi_running = false;
            bool triangulation_initialized = false;
            bool detection_running = false;
            bool detection_initalized = false;

            std::string msg_caller = "Executor";

            bool is_closed = false;
        };
    }
}

#endif //NSSC_TASK_EXECUTOR_