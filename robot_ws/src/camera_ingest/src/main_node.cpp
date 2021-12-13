#include "camera_manager.h"
#include "node.h"
#include "ndi.h"
#include "nssc_errors.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NSSC>();

    NSSC_ERRORS eHandler(node);

    auto camManager = std::make_shared<cameraManager>(node);
    NODE_VERIFY_EXIT(camManager->init());

    NDI ndi(node, camManager);
    NODE_VERIFY_EXIT(ndi.init());

    NODE_VERIFY_EXIT(camManager->loadCameras());

    NODE_VERIFY_EXIT(ndi.startStream());

    node->openCLI();

    rclcpp::spin(node);

    NODE_VERIFY_EXIT(ndi.endStream());
    NODE_VERIFY_EXIT(camManager->closeCameras());

    rclcpp::shutdown();
}