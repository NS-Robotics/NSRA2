#include "cli.h"

void CLI::CLIFunc()
{
    char *buf;

    while ((buf = readline("\033[1;32m[NSSC client] >> \033[0m")) != nullptr && this->cliON.load())
    {
        if (strlen(buf) > 0)
        {
            add_history(buf);
        } else
        {
            continue;
        }

        if(strcmp(buf, "NDI") == 0)
        {
            rawNDI();
        }
        else if(strcmp(buf, "ingest") == 0)
        {
            ingest();
        }
        else if(strcmp(buf, "exit") == 0)
        {
            this->cliON = false;
            exit();
            break;
        }
        else if(strcmp(buf, "help") == 0)
        {
            printf("\033[1;34m[Executor] \033[0mUsage:\n"
                   "  NDI - Toggle raw NDI stream\n"
                   "  exit - Close Application\n"
                   "  ingest - Start calibration capture\n");
        }
        else
        {
            printf("\033[1;34m[Executor] \033[1;31mError: Unknown command!\033[0m\n");
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