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

    for (int i = 0; i < 3; i++)
    {
        stereoFrame* frame = stereoFrame::make_frame(this->node->g_config.g_type);
        frame->alloc();

        this->emptyFrameBuf.enqueue(frame);
        this->numOfEmpty++;
    }

    this->node->printInfo(this->msgCaller, "initalized!");

    return NSSC_STATUS_SUCCESS;
}

NSSC_STATUS cameraManager::loadCameras()
{
    this->node->printInfo(this->msgCaller, "Loading Cameras");

    _CM_VERIFY_EXIT(this->cam1->LoadCamera("TV0200110012"));
    _CM_VERIFY_EXIT(this->cam2->LoadCamera("TV0200110013"));

    _CM_VERIFY_EXIT(this->cam1->startAcquisition());
    _CM_VERIFY_EXIT(this->cam2->startAcquisition());

    this->node->printInfo(this->msgCaller, "Cameras loaded!");

    return NSSC_STATUS_SUCCESS;
}

NSSC_STATUS cameraManager::closeCameras()
{
    this->cb->Break();
    NSSC_STATUS status = this->cam1->CloseCamera();
    if(status != NSSC_STATUS_SUCCESS)
    {
        this->cam2->CloseCamera();
        this->node->printInfo(this->msgCaller, "Cameras closed!");
        return status;
    } else
    {
        status = this->cam2->CloseCamera();
        this->node->printInfo(this->msgCaller, "Cameras closed!");
        return status;
    }
}

stereoFrame* cameraManager::getFrame()
{
    stereoFrame* stereoFrame;
    this->emptyFrameBuf.wait_dequeue(stereoFrame);
    this->numOfEmpty--;

    stereoFrame->convert(this->cam1->getFrame(), this->cam2->getFrame());

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