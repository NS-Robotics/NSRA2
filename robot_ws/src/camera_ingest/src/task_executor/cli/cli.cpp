#include "cli.h"

void CLI::CLIFunc()
{
    char *buf;

    while ((buf = readline("[NSSC client] ")) != nullptr && this->cliON.load())
    {
        if (strlen(buf) > 0)
        {
            add_history(buf);
        }

        //printf("[%s]\n", buf);

        if(strcmp(buf, "NDI") == 0)
        {
            rawNDI();
        } 
        else if(strcmp(buf, "exit") == 0)
        {
            this->cliON = false;
            exit();
            break;
        }

        
        free(buf);
    }
}

void CLI::openCLI()
{
    this->cliON = true;
    this->CLIThread = std::thread(&CLI::CLIFunc, this);
}

void CLI::closeCLI()
{
    this->cliON = false;
    this->CLIThread.join();
}