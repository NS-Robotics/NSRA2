#include "camera_manager.h"
#include "node.h"
#include "camera.h"
#include "cam_sync.h"
#include "stereo_frame.h"
#include "nssc_errors.h"

nssc::ingest::CameraManager::CameraManager(std::shared_ptr<ros::NSSC>& node) : NSSC_ERRORS(node)
{
    this->node = node;
    this->cb = std::make_unique<CyclicBarrier>(2);
}

nssc::ingest::CameraManager::~CameraManager()
{
    closeCameras();
}

nssc::NSSC_STATUS nssc::ingest::CameraManager::init()
{
    this->cam1 = std::make_unique<Camera>(this->node, this->cb);
    this->cam2 = std::make_unique<Camera>(this->node, this->cb);

    _CM_VERIFY_EXIT(this->cam1->init());
    _CM_VERIFY_EXIT(this->cam2->init());

    for (int i = 0; i < 7; i++)
    {
        framestruct::StereoFrame* frame = framestruct::StereoFrame::makeFrame(this->node->g_config.frameConfig.g_type);
        frame->alloc(this->node);
        frame->concatenate_buf.id = i;

        this->buf_empty.enqueue(frame);
        this->num_empty++;
    }

    this->node->printInfo(this->msg_caller, "initalized!");

    return NSSC_STATUS_SUCCESS;
}

nssc::NSSC_STATUS nssc::ingest::CameraManager::setExposure(float exposure_time)
{
    this->cam1->setExposure(exposure_time);
    this->cam2->setExposure(exposure_time);
    return NSSC_STATUS_SUCCESS;
}

nssc::NSSC_STATUS nssc::ingest::CameraManager::setGain(float gain)
{
    this->cam1->setGain(gain);
    this->cam2->setGain(gain);
    return NSSC_STATUS_SUCCESS;
}

nssc::NSSC_STATUS nssc::ingest::CameraManager::loadCameras()
{
    this->node->printInfo(this->msg_caller, "Loading Cameras");

    _CM_VERIFY_EXIT(this->cam1->loadCamera("TV0200110012"));
    _CM_VERIFY_EXIT(this->cam2->loadCamera("TV0200110013"));

    _CM_VERIFY_EXIT(this->cam1->startAcquisition());
    _CM_VERIFY_EXIT(this->cam2->startAcquisition());

    this->node->printInfo(this->msg_caller, "Cameras loaded");

    return NSSC_STATUS_SUCCESS;
}

nssc::NSSC_STATUS nssc::ingest::CameraManager::closeCameras()
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
        this->node->printWarning(this->msg_caller, "Cameras closed");
        return status;
    } else
    {
        status = this->cam2->closeCamera();
        this->node->printInfo(this->msg_caller, "Cameras closed");
        return status;
    }
}

nssc::framestruct::StereoFrame *nssc::ingest::CameraManager::getFrame()
{
    framestruct::StereoFrame* stereoFrame;
    this->buf_empty.wait_dequeue(stereoFrame);
    this->num_empty--;

    stereoFrame->convert(this->cam1->getFrame(), this->cam2->getFrame(), this->node->g_config.frameConfig.resize_frame);

    return stereoFrame;
}

nssc::NSSC_STATUS nssc::ingest::CameraManager::returnBuf(framestruct::StereoFrame* stereoFrame)
{
    NSSC_STATUS status;

    status = this->cam1->returnBuf(stereoFrame->left_camera);
    status = this->cam2->returnBuf(stereoFrame->right_camera);

    this->buf_empty.enqueue(stereoFrame);
    this->num_empty++;

    return status;
}