#include "cli.h"

CLI::CLI()
{
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

        if(buf == "hello")
        {
            printf("hello!");
        } else if(buf == "exit")
        {
            printf("closing CLI!");
            this->streamON = false;
        }

        
        free(buf);
    }
}

void CLI::stopCLI()
{
    this->streamON = false;
}