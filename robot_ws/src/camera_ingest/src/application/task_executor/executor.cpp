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
    if (this->detection_running)
    {
        this->object_detection->stopDetection();
        this->detection_running = false;
    }
    if (this->detection_initalized)
    {
        this->object_detection->closeDetection();
        this->detection_initalized = false;
    }
    if (this->NDI_running)
    {
        this->ndi->endStream();
        this->NDI_running = false;
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

    this->frameManager = NDIframeManager::make_frame(NDI_SEND_RAW);
    this->frameManager->init(this->node, this->camManager);

    this->ndi = std::make_shared<NDI>(this->node, &this->frameManager);
    this->ndi->init();
    this->ndi_initialized = true;

    CLI::openCLI(this->node);
}

void Executor::toggleNDI(bool mono_stream)
{
    if (this->NDI_running)
    {
        this->ndi->endStream();
        this->NDI_running = false;
    }
    else if (mono_stream == this->node->g_config.frameConfig.mono_stream)
    {
        _toggleNDIsource(NDI_SEND_RAW);
    }
    else
    {
        if (this->ndi_initialized)
        {
            this->ndi->closeNDI();
            this->ndi_initialized = false;
        }
        this->node->g_config.frameConfig.mono_stream = mono_stream;
        this->node->g_config.frameConfig.calculate_params();

        this->ndi->init();
        this->ndi_initialized = true;
        this->ndi->startStream();
        this->NDI_running = true;
    }
}

void Executor::_toggleNDIsource(NSSC_NDI_SEND type)
{
    if (this->NDI_running)
    {
        this->ndi->endStream();
        this->NDI_running = false;
    }

    this->frameManager = NDIframeManager::make_frame(type);
    this->frameManager->init(this->node, this->camManager);

    this->ndi->startStream();
    this->NDI_running = true;
}

void Executor::run_ingest()
{
    _toggleNDIsource(NDI_SEND_INGEST);
    this->ingest = new Ingest(this->node, this->camManager);
}

void Executor::run_calibration(char *setName)
{
    _toggleNDIsource(NDI_SEND_CALIBRATION);
    this->calibration = new Calibration(this->node, setName);
}

void Executor::run_triangulation(char *setName)
{
    _toggleNDIsource(NDI_SEND_TRIANGULATION);

    this->triangulation_interface = std::make_shared<TriangulationInterface>(this->node, &this->frameManager, setName);
    if (this->triangulation_interface->init() == NSSC_STATUS_SUCCESS)
        this->triangulation_initialized = true;
    else
        this->triangulation_initialized = false;

    this->ndi->endStream();
    this->NDI_running = false;
}

void Executor::run_detection()
{
    if (this->triangulation_initialized)
    {
        _toggleNDIsource(NDI_SEND_TRIANGULATION);
        this->object_detection = std::make_unique<ObjectDetection>(this->node, this->triangulation_interface);
        this->detection_running = true;
    }
    else
    {
        _toggleNDIsource(NDI_SEND_TRIANGULATION);

        this->triangulation_interface = std::make_shared<TriangulationInterface>(this->node, &this->frameManager, this->node->g_config.triangulationConfig.standard_config_file);
        if (this->triangulation_interface->init() != NSSC_STATUS_SUCCESS)
        {
            this->triangulation_initialized = false;
            return;
        }

        this->triangulation_initialized = true;
        this->object_detection = std::make_unique<ObjectDetection>(this->node, this->triangulation_interface);
        this->detection_running = true;
        this->detection_initalized = true;
    }

}

void Executor::find_triangulation_origin()
{
    _toggleNDIsource(NDI_SEND_TRIANGULATION);

    if (this->triangulation_initialized)
    {
        this->triangulation_interface->findOrigin();
    }
    else
    {
        this->node->printError(this->msgCaller, "Triangulation Interface not initialized!");
    }

    this->ndi->endStream();
    this->NDI_running = false;
}

void Executor::cancel()
{
    if (this->node->g_config.ingestConfig.is_running)
    {
        this->ingest->cancelIngest();
    }
    if (this->detection_running)
    {
        this->object_detection->stopDetection();
        this->detection_running = false;

        this->ndi->endStream();
        this->NDI_running = false;
    }
}