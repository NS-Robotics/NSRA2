#include "ingest.h"

Ingest::Ingest(std::shared_ptr<NSSC>& node, std::shared_ptr<cameraManager>& camManager, int ingestAmount) : NSSC_ERRORS(node)
{
    this->node = node;
    this->camManager = camManager;
    this->ingestAmount = ingestAmount;

    this->runIngest = true;
    this->iThread = std::thread(&Ingest::ingestThread, this);
    
    this->node->g_config.ingestConfig.is_running = true;
    this->node->printInfo(this->msgCaller, "Ingest!");
}

void Ingest::ingestThread()
{
    stereoFrame* stereoFrame;
    this->node->printInfo(this->msgCaller, "test1");
    for (int i = 0; i < this->ingestAmount; i++)
    {
        this->node->printInfo(this->msgCaller, "test2");
        while(this->runIngest.load())
        {
            stereoFrame = this->camManager->getFrame();
            this->node->printInfo(this->msgCaller, "test3");
            if(stereoFrame->timedif < this->node->g_config.ingestConfig.max_frame_time_diff)
            {
                this->node->printInfo(this->msgCaller, "test4");
                break;
            }
        }
        this->node->printInfo(this->msgCaller, "Ingest frame!");
        std::this_thread::sleep_for(std::chrono::milliseconds(this->node->g_config.ingestConfig.wait_duration));
    }
}