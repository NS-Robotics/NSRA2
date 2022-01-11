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

class Executor : public NSSC_ERRORS, public CLI
{
public:
    Executor(std::shared_ptr<NSSC> &node, std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> &node_executor);
    void exit() override;
    void init();
    void toggleNDI(bool mono_stream) override;
    void _toggleNDIsource(NSSC_NDI_SEND type) override;
    void run_ingest() override;
    void run_calibration(char *setName) override;
    void run_triangulation(char *setName) override;
    void find_triangulation_origin() override;
    void run_detection() override;
    void set_exposure(float exposure_time) override;
    void set_gain(float gain) override;
    void cancel() override;

private:
    std::shared_ptr<NSSC> node;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> node_executor;
    std::shared_ptr<cameraManager> camManager;
    std::shared_ptr<NDI> ndi;
    std::unique_ptr<NDIframeManager> frameManager;

    Ingest *ingest;
    Calibration *calibration;
    std::shared_ptr<TriangulationInterface> triangulation_interface;
    std::unique_ptr<ObjectDetection> object_detection;

    bool ndi_initialized = false;
    bool cam_manager_initialized = false;
    bool NDI_running = false;
    bool triangulation_initialized = false;
    bool detection_running = false;
    bool detection_initalized = false;

    std::string msgCaller = "Executor";

    bool is_closed = false;
};

#endif //NSSC_TASK_EXECUTOR_