#include "cli.h"

void CLI::CLIFunc()
{
    char *buf;

    while ((buf = readline("[NSSC client] >> ")) != nullptr && this->cliON.load())
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
        else if(strcmp(buf, "help") == 0)
        {
            printf("[Executor] Usage:\n \
                    NDI - Open raw NDI stream\n \
                    exit - Close Application\n");
        }
        else
        {
            printf("\033[1;31m[Executor] Error: Unknown command!\033[0m\n");
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