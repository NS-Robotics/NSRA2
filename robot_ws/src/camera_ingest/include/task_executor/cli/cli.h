#ifndef NSSC_TASK_EXECUTOR_CLI_
#define NSSC_TASK_EXECUTOR_CLI_

#include <readline/readline.h>
#include <readline/history.h>

#include <thread>
#include <atomic>
#include <string.h>

#include "node.h"
#include "nssc_errors.h"

class CLI
{
    public:
        void CLIFunc();
        void openCLI();
        void closeCLI();

        virtual void exit() = 0;
        virtual void rawNDI() = 0;
        virtual void ingest() = 0;

        std::atomic<bool> cliON{false};

    private:
        std::thread CLIThread;

        NSSC_STATUS getIntArg(std::vector<char*> cmd, char par, int& arg);
        const char del[2] = "-";
        const char arg_del[2] = " ";
};

#endif  // NSSC_TASK_EXECUTOR_CLI_