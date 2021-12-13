#include "cli.h"

CLI::CLI(std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>& node_executor)
{
    this->node_executor = node_executor;
    this->streamON = true;
    this->CLIThread = std::thread(&CLI::CLIFunc, this);
}

void CLI::CLIFunc()
{
    char *buf;

    while ((buf = readline("[NSSC client] ")) != nullptr && this->streamON.load())
    {
        if (strlen(buf) > 0)
        {
            add_history(buf);
        }

        //printf("[%s]\n", buf);

        if(strcmp(buf, "hello") == 0)
        {
            printf("hello!\n");
        } else if(strcmp(buf, "exit") == 0)
        {
            printf("closing CLI!\n");
            this->streamON = false;
            break;
        }

        
        free(buf);
    }
    this->node_executor->cancel();
}

void CLI::stopCLI()
{
    this->streamON = false;
    this->CLIThread.join();
}