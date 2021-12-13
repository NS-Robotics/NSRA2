#include "cli.h"

CLI::CLI()
{
    this->streamON = true;
    this->CLIThread = std::thread(&CLI::CLIFunc, this);
}

void CLI::CLIFunc()
{
    char *buf;
    char hello[] = "hello";
    char exit[] = "exit";

    while ((buf = readline("[NSSC client] ")) != nullptr && this->streamON.load())
    {
        if (strlen(buf) > 0)
        {
            add_history(buf);
        }

        //printf("[%s]\n", buf);

        if(strcmp(buf, hello) == 0)
        {
            printf("hello!");
        } else if(strcmp(buf, exit) == 0)
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