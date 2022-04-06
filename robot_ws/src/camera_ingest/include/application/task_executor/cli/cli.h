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

#ifndef NSSC_TASK_EXECUTOR_CLI_
#define NSSC_TASK_EXECUTOR_CLI_

#include <readline/readline.h>
#include <readline/history.h>

#include <thread>
#include <atomic>
#include <string.h>
#include <boost/lexical_cast.hpp>

#include "node.h"
#include "nssc_errors.h"

namespace nssc
{
    namespace application
    {
        class CLI
        {
        public:
            void CLIFunc();
            void openCLI(std::shared_ptr<ros::NSSC> &node);
            void closeCLI();

            virtual void exit() = 0;
            virtual void toggleNDI(bool mono_stream) = 0;
            virtual void toggleNDIsource(NSSC_NDI_SEND type) = 0;
            virtual void runIngest() = 0;
            virtual void runCalibration(char *setName) = 0;
            virtual void runTriangulation(char *setName) = 0;
            virtual void findTriangulationOrigin() = 0;
            virtual void runDetection() = 0;
            virtual void setExposure(float exposure_time) = 0;
            virtual void setGain(float gain) = 0;
            virtual void cancel() = 0;

            std::atomic<bool> cli_running{false};

        private:
            std::thread cli_thread;

            std::shared_ptr<ros::NSSC> node;

            void printError(const char *message);

            NSSC_STATUS __procArg(char *buf, std::vector<char *> &cmd);
            NSSC_STATUS getIntArg(std::vector<char *> cmd, char par, int &ret);
            NSSC_STATUS getBoolArg(std::vector<char *> cmd, char par, bool &ret);
            NSSC_STATUS getStrArg(std::vector<char *> cmd, char par, char **ret);
            const char del[2] = "-";
            const char arg_del[2] = " ";
        };
    }
}

#endif // NSSC_TASK_EXECUTOR_CLI_