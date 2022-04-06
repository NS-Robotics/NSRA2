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