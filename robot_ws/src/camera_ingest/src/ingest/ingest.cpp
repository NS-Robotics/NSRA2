#include "ingest.h"

Ingest::Ingest(std::shared_ptr<NSSC>& node, std::shared_ptr<cameraManager>& camManager, int ingestAmount, char* setName) : NSSC_ERRORS(node)
{
    this->node = node;
    this->camManager = camManager;
    this->ingestAmount = ingestAmount;
    this->setName = setName;

    this->setPath = this->node->g_config.share_dir + "/NSSC/" + setName + "/";
    this->node->printInfo(this->msgCaller, this->setPath);

    this->runIngest = true;
    this->iThread = std::thread(&Ingest::ingestThread, this);
    
    this->node->g_config.ingestConfig.is_running = true;
    this->node->printInfo(this->msgCaller, "Ingest!");
}

void Ingest::ingestThread()
{
    stereoFrame* stereoFrame;
    this->node->g_config.ingestConfig.current_frame_idx = 0;

    for (int i = 0; i < this->ingestAmount; i++)
    {
        if (!this->runIngest.load()) { break; }

        while(this->runIngest.load())
        {
            stereoFrame = this->camManager->getFrame();
            if(stereoFrame->timedif < this->node->g_config.ingestConfig.max_frame_time_diff)
            {
                break;
            }
        }

        cv::Mat leftFrame(cv::Size(this->node->g_config.frameConfig.cam_x_res, this->node->g_config.frameConfig.cam_y_res), CV_8UC4, stereoFrame->leftCamera->frameBuf.hImageBuf);
        cv::Mat rightFrame(cv::Size(this->node->g_config.frameConfig.cam_x_res, this->node->g_config.frameConfig.cam_y_res), CV_8UC4, stereoFrame->rightCamera->frameBuf.hImageBuf);
        
        cv::Mat left_conv;
        cv::Mat right_conv;

        cv::cvtColor(leftFrame, left_conv, cv::COLOR_RGBA2BGRA);
        cv::cvtColor(rightFrame, right_conv, cv::COLOR_RGBA2BGRA);

        cv::imwrite((this->setPath + "img_right_%d.png", this->node->g_config.ingestConfig.current_frame_idx), right_conv);
        cv::imwrite((this->setPath + "img_left_%d.png", this->node->g_config.ingestConfig.current_frame_idx), left_conv);

        this->node->g_config.ingestConfig.current_frame_idx++;
        this->node->printInfo(this->msgCaller, "Ingest frame!");
        std::this_thread::sleep_for(std::chrono::milliseconds(this->node->g_config.ingestConfig.wait_duration));
    }
}

void Ingest::cancelIngest()
{
    this->runIngest = false;
    this->node->g_config.ingestConfig.is_running = false;
    this->iThread.join();
    this->node->printInfo(this->msgCaller, "Ingest canceled");
}