#include "ingest.h"

Ingest::Ingest(std::shared_ptr<NSSC> &node, std::shared_ptr<cameraManager> &camManager) : NSSC_ERRORS(node)
{
    this->node = node;
    this->camManager = camManager;

    this->setPath = this->node->g_config.share_dir + "/" + this->node->g_config.ingestConfig.set_name + "/";
    std::system(("mkdir -p " + this->setPath).c_str());

    std::ifstream xmlFile(this->setPath + "config.xml");
    if (!xmlFile.fail())
    {
        this->node->printWarning(this->msgCaller, "This ingest configuration already exists");
    }
    else
    {
        saveConfig();

        this->runIngest = true;
        this->iThread = std::thread(&Ingest::ingestThread, this);

        this->node->g_config.ingestConfig.is_running = true;
        this->node->printInfo(this->msgCaller, "Ingest!");
    }
}

void Ingest::saveConfig()
{
    rapidxml::xml_document<> new_doc;

    rapidxml::xml_node<> *decl = new_doc.allocate_node(rapidxml::node_declaration);
    decl->append_attribute(new_doc.allocate_attribute("version", "1.0"));
    decl->append_attribute(new_doc.allocate_attribute("encoding", "utf-8"));
    new_doc.append_node(decl);

    rapidxml::xml_node<> *root = new_doc.allocate_node(rapidxml::node_element, "NSSC");
    root->append_attribute(new_doc.allocate_attribute("type", "ingest config"));
    new_doc.append_node(root);

    rapidxml::xml_node<> *config = new_doc.allocate_node(rapidxml::node_element, "Config");
    config->append_attribute(new_doc.allocate_attribute("setName", this->node->g_config.ingestConfig.set_name));
    std::string s = std::to_string(this->node->g_config.ingestConfig.ingest_amount);
    config->append_attribute(new_doc.allocate_attribute("ingestAmount", s.c_str()));
    root->append_node(config);

    std::string xml_as_string;
    rapidxml::print(std::back_inserter(xml_as_string), new_doc);

    std::ofstream file_stored(this->setPath + "config.xml");
    file_stored << new_doc;
    file_stored.close();
    new_doc.clear();
}

void Ingest::editConfig()
{
    rapidxml::xml_document<> doc;
    rapidxml::xml_node<> *root_node = NULL;

    std::ifstream xmlFile(this->setPath + "config.xml");
    std::vector<char> buffer((std::istreambuf_iterator<char>(xmlFile)), std::istreambuf_iterator<char>());
    buffer.push_back('\0');

    doc.parse<rapidxml::parse_full | rapidxml::parse_no_data_nodes>(&buffer[0]);

    root_node = doc.first_node("NSSC");

    std::string s = std::to_string(this->node->g_config.ingestConfig.current_frame_idx);
    root_node->first_node("Config")->first_attribute("ingestAmount")->value(s.c_str());

    std::ofstream file_stored(this->setPath + "config.xml");
    file_stored << doc;
    file_stored.close();
    doc.clear();
}

void Ingest::ingestThread()
{
    stereoFrame *stereoFrame;
    this->node->g_config.ingestConfig.current_frame_idx = 0;

    for (int i = 0; i < this->node->g_config.ingestConfig.ingest_amount; i++)
    {
        while (this->runIngest.load())
        {
            stereoFrame = this->camManager->getFrame(true);
            if (stereoFrame->timedif < this->node->g_config.ingestConfig.max_frame_time_diff)
            {
                break;
            }
            else
            {
                this->camManager->returnBuf(stereoFrame);
            }
        }

        if (!this->runIngest.load()) { break; }

        this->node->g_config.ingestConfig.image_taken = true;

        cv::Mat leftFrame(cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res), CV_8UC4, stereoFrame->leftCamera->frameBuf.hImageBuf);
        cv::Mat rightFrame(cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res), CV_8UC4, stereoFrame->rightCamera->frameBuf.hImageBuf);

        cv::Mat left_conv;
        cv::Mat right_conv;

        cv::cvtColor(leftFrame, left_conv, cv::COLOR_RGBA2BGRA);
        cv::cvtColor(rightFrame, right_conv, cv::COLOR_RGBA2BGRA);

        std::string right_name(this->node->g_config.ingestConfig.right_img_name);
        cv::imwrite(this->setPath + right_name + std::to_string(this->node->g_config.ingestConfig.current_frame_idx) + ".png", right_conv);
        std::string left_name(this->node->g_config.ingestConfig.left_img_name);
        cv::imwrite(this->setPath + left_name + std::to_string(this->node->g_config.ingestConfig.current_frame_idx) + ".png", left_conv);

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
    editConfig();
    this->runIngest = false;
    this->node->g_config.ingestConfig.is_running = false;
    this->iThread.join();
    this->node->printInfo(this->msgCaller, "Ingest canceled");
}