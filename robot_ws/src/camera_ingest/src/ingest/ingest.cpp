#include "ingest.h"

Ingest::Ingest(std::shared_ptr<NSSC>& node, std::shared_ptr<cameraManager>& camManager, int ingestAmount) : NSSC_ERRORS(node)
{
    this->node = node;
    this->camManager = camManager;
    this->ingestAmount = ingestAmount;

    this->runIngest = true;
    this->iThread = std::thread(&Ingest::runIngest, this);
    
    this->node->g_config.ingestConfig.is_running = true;
    this->node->printInfo(this->msgCaller, "Ingest!");
}

void Ingest::ingestThread()
{
    stereoFrame* stereoFrame;

    for (int i = 0; i < this->ingestAmount; i++)
    {
        while(this->runIngest.load())
        {
            stereoFrame = this->camManager->getFrame();
            if(stereoFrame->timedif < this->node->g_config.ingestConfig.max_frame_time_diff)
            {
                break;
            }
            this->node->printInfo(this->msgCaller, "Ingest frame!");
            std::this_thread::sleep_for(std::chrono::milliseconds(this->node->g_config.ingestConfig.wait_duration));
        }
        
    }
}