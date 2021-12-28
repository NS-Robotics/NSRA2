#include "ingest.h"

Ingest::Ingest(std::shared_ptr<NSSC>& node, std::shared_ptr<cameraManager>& camManager) : NSSC_ERRORS(node)
{
    this->node = node;
    this->camManager = camManager;

    this->setPath = this->node->g_config.share_dir + "/" + this->node->g_config.ingestConfig.set_name + "/";
    std::system(("mkdir -p " + this->setPath).c_str());

    tinyxml2::XMLDocument xmlDoc;

    int ret = xmlDoc.Parse((this->setPath + "config.xml").c_str());
    this->node->printInfo(this->msgCaller, "doc " + std::to_string(ret));
    
    if(ret == 0)
    {
        this->node->printWarning(this->msgCaller, "This ingest configuration already exists");
    } else
    {
        //tinyxml2::XMLDeclaration* decl = new tinyxml2::XMLDeclaration("1.0", "UTF-8", "");
        xmlDoc.NewDeclaration();

        tinyxml2::XMLElement *pRoot = xmlDoc.NewElement("NSSC");
        xmlDoc.InsertFirstChild(pRoot);

        tinyxml2::XMLElement *setName = xmlDoc.NewElement("setName");
        setName->SetText("test");
        pRoot->InsertEndChild(setName);

        tinyxml2::XMLElement *ingestAmount = xmlDoc.NewElement("ingestAmount");
        ingestAmount->SetText(this->node->g_config.ingestConfig.ingest_amount);
        pRoot->InsertEndChild(ingestAmount);

        xmlDoc.SaveFile((this->setPath + "config.xml").c_str());

        this->runIngest = true;
        this->iThread = std::thread(&Ingest::ingestThread, this);

        this->node->g_config.ingestConfig.is_running = true;
        this->node->printInfo(this->msgCaller, "Ingest!");
    }
}

void Ingest::ingestThread()
{
    stereoFrame* stereoFrame;
    this->node->g_config.ingestConfig.current_frame_idx = 0;

    for (int i = 0; i < this->node->g_config.ingestConfig.ingest_amount; i++)
    {
        if (!this->runIngest.load()) { break; }
        
        while(this->runIngest.load())
        {
            stereoFrame = this->camManager->getFrame();
            if(stereoFrame->timedif < this->node->g_config.ingestConfig.max_frame_time_diff)
            {
                break;
            } else
            {
                this->camManager->returnBuf(stereoFrame);
            }
        }

        this->node->g_config.ingestConfig.image_taken = true;
        
        cv::Mat leftFrame(cv::Size(this->node->g_config.frameConfig.cam_x_res, this->node->g_config.frameConfig.cam_y_res), CV_8UC4, stereoFrame->leftCamera->frameBuf.hImageBuf);
        cv::Mat rightFrame(cv::Size(this->node->g_config.frameConfig.cam_x_res, this->node->g_config.frameConfig.cam_y_res), CV_8UC4, stereoFrame->rightCamera->frameBuf.hImageBuf);

        cv::Mat left_conv;
        cv::Mat right_conv;

        cv::cvtColor(leftFrame, left_conv, cv::COLOR_RGBA2BGRA);
        cv::cvtColor(rightFrame, right_conv, cv::COLOR_RGBA2BGRA);

        cv::imwrite(this->setPath + "img_right_" + std::to_string(this->node->g_config.ingestConfig.current_frame_idx) + ".png", right_conv);
        cv::imwrite(this->setPath + "img_left_" + std::to_string(this->node->g_config.ingestConfig.current_frame_idx) + ".png", left_conv);

        this->camManager->returnBuf(stereoFrame);

        this->node->g_config.ingestConfig.current_frame_idx++;
        this->node->printInfo(this->msgCaller, "Ingest frame nr: " + std::to_string(this->node->g_config.ingestConfig.current_frame_idx));
        this->node->g_config.ingestConfig.sleep_timestamp = std::chrono::high_resolution_clock::now();
        this->node->g_config.ingestConfig.image_taken = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(this->node->g_config.ingestConfig.wait_duration));
    }
    this->node->g_config.ingestConfig.is_running = false;
    this->runIngest = false;
}

void Ingest::cancelIngest()
{
    this->runIngest = false;
    this->node->g_config.ingestConfig.is_running = false;
    this->iThread.join();
    this->node->printInfo(this->msgCaller, "Ingest canceled");
}