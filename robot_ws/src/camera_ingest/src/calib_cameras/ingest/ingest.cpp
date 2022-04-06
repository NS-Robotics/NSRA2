/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Noa Sendlhofer
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Noa Sendlhofer

#include "ingest.h"
#include "camera_manager.h"
#include "node.h"
#include "stereo_frame.h"
#include "nssc_errors.h"

nssc::stereocalibration::Ingest::Ingest(std::shared_ptr<nssc::ros::NSSC> &node,
                                        std::unique_ptr<nssc::send::FrameManager>* frame_manager) : NSSC_ERRORS(node)
{
    this->node = node;
    this->frame_manager = frame_manager;

    this->set_path = this->node->g_config.share_dir + "/" + this->node->g_config.ingestConfig.set_name + "/";
    std::system(("mkdir -p " + this->set_path).c_str());

    std::ifstream xmlFile(this->set_path + "config.xml");
    if (!xmlFile.fail())
    {
        this->node->printWarning(this->msg_caller, "This ingest configuration already exists");
    }
    else
    {
        cv::FileStorage config_file(this->set_path + "config.xml", cv::FileStorage::WRITE);
        config_file << "setName" << this->node->g_config.ingestConfig.set_name;
        config_file << "ingestAmount" << this->node->g_config.ingestConfig.ingest_amount;

        this->run_ingest = true;
        this->i_thread = std::thread(&nssc::stereocalibration::Ingest::ingestThread, this);

        this->node->g_config.ingestConfig.is_running = true;
        this->node->printInfo(this->msg_caller, "Ingest!");
    }
}
/*
__attribute__((unused)) void Ingest::saveConfig()
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

    std::ofstream file_stored(this->set_path + "config.xml");
    file_stored << new_doc;
    file_stored.close();
    new_doc.clear();
}

__attribute__((unused)) void Ingest::editConfig()
{
    rapidxml::xml_document<> doc;
    rapidxml::xml_node<> *root_node = nullptr;

    std::ifstream xmlFile(this->set_path + "config.xml");
    std::vector<char> buffer((std::istreambuf_iterator<char>(xmlFile)), std::istreambuf_iterator<char>());
    buffer.push_back('\0');

    doc.parse<rapidxml::parse_full | rapidxml::parse_no_data_nodes>(&buffer[0]);

    root_node = doc.first_node("NSSC");

    std::string s = std::to_string(this->node->g_config.ingestConfig.current_frame_idx);
    root_node->first_node("Config")->first_attribute("ingestAmount")->value(s.c_str());

    std::ofstream file_stored(this->set_path + "config.xml");
    file_stored << doc;
    file_stored.close();
    doc.clear();
}
*/
void nssc::stereocalibration::Ingest::ingestThread()
{
    this->node->g_config.ingestConfig.current_frame_idx = 0;

    while (this->run_ingest.load() &&
            this->node->g_config.ingestConfig.current_frame_idx < this->node->g_config.ingestConfig.ingest_amount)
    {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::milliseconds difference = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->node->g_config.ingestConfig.sleep_timestamp);
        std::chrono::milliseconds duration = std::chrono::milliseconds(this->node->g_config.ingestConfig.wait_duration);
        if (difference >= duration)
        {
            this->node->printInfo(this->msg_caller, "Take Image!");
            Ingest::_takeImage();
        }

        if (this->run_ingest.load())
        {
            this->node->printInfo(this->msg_caller, "Send Image!");
            Ingest::_sendImage();
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(this->node->g_config.ingestConfig.wait_duration));

    this->node->g_config.ingestConfig.is_running = false;
    this->run_ingest = false;
}

void nssc::stereocalibration::Ingest::_sendImage()
{
    nssc::framestruct::StereoFrame *stereo_frame;
    stereo_frame = (*this->frame_manager)->getCameraFrame();

    cv::Mat sendFrame(cv::Size(this->node->g_config.frameConfig.stream_x_res, this->node->g_config.frameConfig.stream_y_res), CV_8UC4, stereo_frame->stereo_buf->hImageBuf);

    cv::putText(sendFrame, "Image idx: " + std::to_string(this->node->g_config.ingestConfig.current_frame_idx) + " out of: " + std::to_string(this->node->g_config.ingestConfig.ingest_amount),
                cv::Point(25, 60), //top-left position
                cv::FONT_HERSHEY_DUPLEX,
                2.0,
                cv::Scalar(254, 0, 0),
                2);

    auto now = std::chrono::high_resolution_clock::now();
    auto time_left = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->node->g_config.ingestConfig.sleep_timestamp);
    int countdown_i = this->node->g_config.ingestConfig.wait_duration - time_left.count();
    if(countdown_i < 0)
    {
        countdown_i = 0;
    }
    std::string countdown = "Next image in " + std::to_string(countdown_i) + " ms";

    if(countdown_i == 0 && this->node->g_config.ingestConfig.image_taken)
    {
        countdown = "Image taken!";
    }

    cv::putText(sendFrame, countdown,
                cv::Point(1000, 60),
                cv::FONT_HERSHEY_DUPLEX,
                2.0,
                cv::Scalar(254, 0, 0),
                2);

    (*this->frame_manager)->sendFrame(stereo_frame);
}

void nssc::stereocalibration::Ingest::_takeImage()
{
    nssc::framestruct::StereoFrame *stereo_frame;

    while (this->node->g_config.frameConfig.stream_on && this->run_ingest.load())
    {
        stereo_frame = (*this->frame_manager)->getCameraFrame();
        if (stereo_frame->timedif < this->node->g_config.ingestConfig.max_frame_time_diff)
            break;
        else
            (*this->frame_manager)->returnBuf(stereo_frame);
    }

    if (!this->run_ingest.load()) { return; }

    this->node->printInfo(this->msg_caller, "Got Frame");

    this->node->g_config.ingestConfig.image_taken = true;

    cv::Mat leftFrame(cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res),
                      CV_8UC4, stereo_frame->left_camera->frame_buf.hImageBuf);
    cv::Mat rightFrame(cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res),
                       CV_8UC4, stereo_frame->right_camera->frame_buf.hImageBuf);

    cv::Mat left_conv;
    cv::Mat right_conv;

    cv::cvtColor(leftFrame, left_conv, cv::COLOR_RGBA2BGRA);
    cv::cvtColor(rightFrame, right_conv, cv::COLOR_RGBA2BGRA);

    std::string right_name(this->node->g_config.ingestConfig.right_img_name);
    cv::imwrite(this->set_path + right_name + std::to_string(this->node->g_config.ingestConfig.current_frame_idx) + ".png", right_conv);
    std::string left_name(this->node->g_config.ingestConfig.left_img_name);
    cv::imwrite(this->set_path + left_name + std::to_string(this->node->g_config.ingestConfig.current_frame_idx) + ".png", left_conv);

    (*this->frame_manager)->sendFrame(stereo_frame);

    this->node->g_config.ingestConfig.current_frame_idx++;
    this->node->printInfo(this->msg_caller, "Ingest frame nr: " + std::to_string(this->node->g_config.ingestConfig.current_frame_idx));

    this->node->g_config.ingestConfig.sleep_timestamp = std::chrono::high_resolution_clock::now();
    this->node->g_config.ingestConfig.image_taken = false;
}

void nssc::stereocalibration::Ingest::cancelIngest()
{
    cv::FileStorage config_file(this->set_path + "config.xml", cv::FileStorage::WRITE);
    config_file << "ingestAmount" << this->node->g_config.ingestConfig.current_frame_idx;

    this->run_ingest = false;
    this->node->g_config.ingestConfig.is_running = false;
    this->i_thread.join();
    this->node->printInfo(this->msg_caller, "Ingest canceled");
}