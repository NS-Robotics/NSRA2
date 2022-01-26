#include "camera_manager.h"

cameraManager::cameraManager(std::shared_ptr<NSSC>& node) : NSSC_ERRORS(node)
{
    this->node = node;
    this->cb = std::make_unique<CyclicBarrier>(2);
}

cameraManager::~cameraManager()
{
    closeCameras();
}

NSSC_STATUS cameraManager::init()
{
    this->cam1 = std::make_unique<Camera>(this->node, this->cb);
    this->cam2 = std::make_unique<Camera>(this->node, this->cb);

    _CM_VERIFY_EXIT(this->cam1->init());
    _CM_VERIFY_EXIT(this->cam2->init());

    for (int i = 0; i < 5; i++)
    {
        stereoFrame* frame = stereoFrame::make_frame(this->node->g_config.frameConfig.g_type);
        frame->alloc(this->node);
        frame->concatenateBuf.id = i;

        this->emptyFrameBuf.enqueue(frame);
        this->numOfEmpty++;
    }

    this->node->printInfo(this->msgCaller, "initalized!");

    return NSSC_STATUS_SUCCESS;
}

NSSC_STATUS cameraManager::setExposure(float exposure_time)
{
    this->cam1->setExposure(exposure_time);
    this->cam2->setExposure(exposure_time);
    return NSSC_STATUS_SUCCESS;
}

NSSC_STATUS cameraManager::setGain(float gain)
{
    this->cam1->setGain(gain);
    this->cam2->setGain(gain);
    return NSSC_STATUS_SUCCESS;
}

NSSC_STATUS cameraManager::loadCameras()
{
    this->node->printInfo(this->msgCaller, "Loading Cameras");

    _CM_VERIFY_EXIT(this->cam1->loadCamera("TV0200110012"));
    _CM_VERIFY_EXIT(this->cam2->loadCamera("TV0200110013"));

    _CM_VERIFY_EXIT(this->cam1->startAcquisition());
    _CM_VERIFY_EXIT(this->cam2->startAcquisition());

    this->node->printInfo(this->msgCaller, "Cameras loaded");

    return NSSC_STATUS_SUCCESS;
}

NSSC_STATUS cameraManager::closeCameras()
{
    if(this->is_closed) { return NSSC_STATUS_SUCCESS; }

    this->is_closed = true;

    this->cam1->stop_age_check = false;
    this->cam2->stop_age_check = false;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    this->cb->Break();
    NSSC_STATUS status = this->cam1->closeCamera();
    if(status != NSSC_STATUS_SUCCESS)
    {
        this->cam2->closeCamera();
        this->node->printWarning(this->msgCaller, "Cameras closed");
        return status;
    } else
    {
        status = this->cam2->closeCamera();
        this->node->printInfo(this->msgCaller, "Cameras closed");
        return status;
    }
}

stereoFrame *cameraManager::getFrame()
{
    stereoFrame* stereoFrame;
    this->emptyFrameBuf.wait_dequeue(stereoFrame);
    this->numOfEmpty--;

    stereoFrame->convert(this->cam1->getFrame(), this->cam2->getFrame(), this->node->g_config.frameConfig.resize_frame);

    return stereoFrame;
}

NSSC_STATUS cameraManager::returnBuf(stereoFrame* stereoFrame)
{
    NSSC_STATUS status;

    status = this->cam1->returnBuf(stereoFrame->leftCamera);
    status = this->cam2->returnBuf(stereoFrame->rightCamera);

    this->emptyFrameBuf.enqueue(stereoFrame);
    this->numOfEmpty++;

    return status;
}