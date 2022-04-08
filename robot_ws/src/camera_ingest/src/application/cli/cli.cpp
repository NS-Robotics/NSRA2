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

#include "cli.h"
#include "node.h"

nssc::application::CLI::CLI(std::shared_ptr<ros::NSSC> &node, std::shared_ptr<Executor> &executor)
{
    this->node = node;
    this->executor = executor;
    this->cli_running = true;
    this->cli_thread = std::thread(&CLI::CLIFunc, this);
}

nssc::application::CLI::~CLI()
{
    CLI::closeCLI();
}

void nssc::application::CLI::closeCLI()
{
    if (this->cli_running.load())
    {
        this->cli_running = false;
        this->cli_thread.join();
    }
}

void nssc::application::CLI::CLIFunc()
{
    char *buf;

    while ((buf = readline("\033[1;32m[NSSC client]\033[0m >> ")) != nullptr && this->cli_running.load())
    {
        if (strlen(buf) > 0)
        {
            add_history(buf);
        }
        else
        {
            continue;
        }

        std::vector<char *> cmd = {};

        if (__procArg(buf, cmd) != NSSC_STATUS_SUCCESS)
        {
            this->printError("Bad argument!");
            continue;
        }

        if (strcmp(cmd[0], "NDI") == 0)
        {
            if(cmd.size() == 1)
            {
                this->executor->toggleNDI(this->node->g_config.frameConfig.mono_stream);
                continue;
            }
            if(cmd.size() == 2)
            {
                bool raw_stream;
                if (getBoolArg(cmd, 'r', raw_stream) != NSSC_STATUS_SUCCESS)
                {
                    this->printError("Bad argument!");
                    continue;
                }
                this->executor->toggleNDIsource(NDI_SEND_RAW);
            } else
            {
                bool mono_stream;
                if (getBoolArg(cmd, 'm', mono_stream) != NSSC_STATUS_SUCCESS)
                {
                    this->printError("Bad argument!");
                    continue;
                }
                this->executor->toggleNDI(false);
                bool raw_stream;
                if (getBoolArg(cmd, 'r', raw_stream) != NSSC_STATUS_SUCCESS)
                {
                    this->printError("Bad argument!");
                    continue;
                }
                this->executor->toggleNDIsource(NDI_SEND_RAW);
            }
        }
        else if (strcmp(cmd[0], "ingest") == 0)
        {
            int ingestAmount;
            if (getIntArg(cmd, 'n', ingestAmount) != NSSC_STATUS_SUCCESS || ingestAmount < 1)
            {
                this->printError("Bad argument!");
                continue;
            }
            char *setName;
            if (getStrArg(cmd, 'd', &setName) != NSSC_STATUS_SUCCESS)
            {
                this->printError("Bad argument!");
                continue;
            }
            this->node->g_config.ingestConfig.set_name = setName;
            this->node->g_config.ingestConfig.ingest_amount = ingestAmount;
            this->executor->runIngest();
            delete[] setName;
        }
        else if (strcmp(cmd[0], "calibrate") == 0)
        {
            char *setName;
            if (getStrArg(cmd, 'd', &setName) != NSSC_STATUS_SUCCESS)
            {
                this->printError("Bad argument!");
            }
            else
            {
                this->executor->runCalibration(setName);
                delete[] setName;
            }
        }
        else if (strcmp(cmd[0], "triangulate") == 0)
        {
            if (cmd.size() == 1)
            {
                this->executor->runTriangulation(const_cast<char *>(this->node->g_config.triangulationConfig.standard_config_file));
                continue;
            }
            char *setName;
            if (getStrArg(cmd, 'd', &setName) != NSSC_STATUS_SUCCESS)
            {
                this->printError("Bad argument!");
            }
            else
            {
                this->executor->runTriangulation(setName);
                delete[] setName;
            }
        }
        else if (strcmp(cmd[0], "detect") == 0)
        {
            this->executor->runDetection();
        }
        else if (strcmp(cmd[0], "calib_origin") == 0)
        {
            this->executor->findTriangulationOrigin();
        }
        else if (strcmp(cmd[0], "setExposure") == 0)
        {
            int exposure_time;
            if (getIntArg(cmd, 'e', exposure_time) != NSSC_STATUS_SUCCESS || exposure_time < 1)
            {
                this->printError("Bad argument!");
            }
            else
            {
                this->executor->setExposure((float) exposure_time);
            }
        }
        else if (strcmp(cmd[0], "setGain") == 0)
        {
            int gain;
            if (getIntArg(cmd, 'g', gain) != NSSC_STATUS_SUCCESS || gain < 1)
            {
                this->printError("Bad argument!");
            }
            else
            {
                this->executor->setGain((float) gain);
            }
        }
        else if (strcmp(cmd[0], "cancel") == 0)
        {
            this->executor->cancel();
        }
        else if (strcmp(cmd[0], "exit") == 0)
        {
            this->executor->exit();
            break;
        }
        else if (strcmp(cmd[0], "help") == 0)
        {
            printf("\033[1;34m[Executor] \033[0mUsage:\n"
                   "  NDI                                       - Toggle raw NDI stream\n"
                   "  ingest [-n number of images -d set name]  - Start the calibration capture\n"
                   "  calibrate [-d image set name]             - Run the calibration\n"
                   "  triangulate [-d calibration config]       - Prepare NSSC for object detection\n"
                   "  calib_origin                              - Recalibrate the robot origin\n"
                   "  detect                                    - Run the NSSC object detection\n"
                   "  setExposure [-e time in microseconds]    - Change the camera exposure time\n"
                   "  setGain [-g gain]                        - Change the camera gain\n"
                   "  cancel                                    - Cancel the currently running process\n"
                   "  exit                                      - Close the application\n");
        }
        else
        {
            this->printError("Unknown command!");
        }

        free(buf);
    }
    this->cli_running = false;
}

void nssc::application::CLI::printError(const char *message)
{
    printf("\033[1;34m[Executor] \033[1;31mError: %s\033[0m\n", message);
}