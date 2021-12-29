#include "executor.h"

Executor::Executor(std::shared_ptr<NSSC> &node, std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> &node_executor) : NSSC_ERRORS(node)
{
    this->node = node;
    this->node_executor = node_executor;
}

void Executor::exit()
{
    if (this->is_closed) { return; }
    if (this->node->g_config.ingestConfig.is_running)
    {
        this->ingest->cancelIngest();
    }
    if (this->rawNDIstream)
    {
        this->ndi->endStream();
        this->rawNDIstream = false;
    }
    if (this->ndi_initialized)
    {
        this->ndi->closeNDI();
        this->ndi_initialized = false;
    }
    if (this->cam_manager_initialized)
    {
        this->camManager->closeCameras();
        this->cam_manager_initialized = false;
    }
    this->node->printInfo(this->msgCaller, "Shutdown complete");
    this->node_executor->cancel();
    this->is_closed = true;
}

void Executor::init()
{
    this->camManager = std::make_shared<cameraManager>(this->node);
    this->camManager->init();
    this->camManager->loadCameras();
    this->cam_manager_initialized = true;

    this->ndi = std::make_shared<NDI>(this->node, this->camManager);
    this->ndi->init();
    this->ndi_initialized = true;

    CLI::openCLI(this->node);   
}

void Executor::rawNDI(bool stream)
{
    if (this->rawNDIstream)
    {
        this->ndi->endStream();
        this->rawNDIstream = false;
    }
    else if (stream == this->node->g_config.frameConfig.mono_stream)
    {
        this->ndi->startStream();
        this->rawNDIstream = true;
    }
    else
    {
        if (this->ndi_initialized)
        {
            this->ndi->closeNDI();
            this->ndi_initialized = false;
        }
        this->node->g_config.frameConfig.mono_stream = stream;
        this->node->g_config.frameConfig.calculate_params();
        
        this->ndi->init();
        this->ndi_initialized = true;
        this->ndi->startStream();
        this->rawNDIstream = true;
    }
}

void Executor::run_ingest()
{
    this->ingest = new Ingest(this->node, this->camManager);
}

void Executor::run_calibration(char *setName)
{
    
}

void Executor::cancel()
{
    if (this->node->g_config.ingestConfig.is_running)
    {
        this->ingest->cancelIngest();
    }
}
