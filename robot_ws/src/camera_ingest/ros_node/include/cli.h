#ifndef NSSC_ROS_NODE_CLI_
#define NSSC_ROS_NODE_CLI_

#include <readline/readline.h>
#include <readline/history.h>

#include <thread>
#include <atomic>
#include <string.h>

class CLI
{
    public:
        CLI();
        void CLIFunc();
        void stopCLI();

    private:
        std::thread CLIThread;
        std::atomic<bool> streamON{false};
};

#endif  // NSSC_ROS_NODE_CLI_