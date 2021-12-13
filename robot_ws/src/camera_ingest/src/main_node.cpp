#include "camera_manager.h"
#include "node.h"
#include "ndi.h"
#include "nssc_errors.h"
#include "executor.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    auto node = std::make_shared<NSSC>();
    node_executor->add_node(node);

    NSSC_ERRORS eHandler(node);

    Executor executor(node, node_executor);

    auto camManager = std::make_shared<cameraManager>(node);
    NODE_VERIFY_EXIT(camManager->init());

    NDI ndi(node, camManager);
    NODE_VERIFY_EXIT(ndi.init());

    NODE_VERIFY_EXIT(camManager->loadCameras());

    NODE_VERIFY_EXIT(ndi.startStream());

    executor.startCLI();

    node_executor->spin();

    executor.stopCLI();

    NODE_VERIFY_EXIT(ndi.endStream());
    NODE_VERIFY_EXIT(camManager->closeCameras());

    rclcpp::shutdown();
}