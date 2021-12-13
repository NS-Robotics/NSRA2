#include "cli.h"

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
            this->streamON = false;
            break;
        }

        
        free(buf);
    }
    exit();
}

void CLI::openCLI()
{
    this->streamON = true;
    this->CLIThread = std::thread(&CLI::CLIFunc, this);
}

void CLI::closeCLI()
{
    this->streamON = false;
    this->CLIThread.join();
}