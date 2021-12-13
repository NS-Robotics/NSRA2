#include "executor.h"

Executor::Executor(std::shared_ptr<NSSC>& node, std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>& node_executor) : NSSC_ERRORS(node)
{
    this->node = node;
    this->node_executor = node_executor;
}

void Executor::exit()
{
    if(this->rawNDIstream)
    {
        this->ndi->endStream();
    }
    if(this->initialized)
    {
        this->camManager->closeCameras();
        this->ndi->closeNDI();
    }
    if(CLI::cliON.load())
    {
        CLI::closeCLI();
    }
    this->node_executor->cancel();
}

void Executor::init()
{
    this->camManager = std::make_shared<cameraManager>(this->node);
    this->camManager->init();
    this->camManager->loadCameras();

    this->ndi = std::make_shared<NDI>(this->node, this->camManager);
    this->ndi->init();

    CLI::openCLI();

    this->initialized = true;
}

void Executor::rawNDI()
{
    if(this->rawNDIstream)
    {
        this->ndi->endStream();
        this->rawNDIstream = false;
    } else
    {
        this->ndi->startStream();
        this->rawNDIstream = true;
    }
}