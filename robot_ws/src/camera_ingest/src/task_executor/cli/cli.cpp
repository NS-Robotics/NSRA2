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

        std::vector<char*> cmd = {};

        if(procArg(buf, cmd) != NSSC_STATUS_SUCCESS)
        {
            this->printError("Bad argument!");
            continue;
        }

        if(strcmp(cmd[0], "NDI") == 0)
        {
            bool ret;
            if(getBoolArg(cmd, 'r', ret) != NSSC_STATUS_SUCCESS)
            {
                this->printError("Bad argument!");
            } else
            {
                std::cout << ret << std::endl;
                rawNDI();
            }
        }
        else if(strcmp(cmd[0], "test") == 0)
        {
            std::cout << "test" << std::endl;
            char* ret;
            if(getStrArg(cmd, 't', &ret) != NSSC_STATUS_SUCCESS)
            {
                this->printError("Bad argument!");
            } else
            {
                std::cout << ret << std::endl;
                delete [] ret;
            }
        }
        else if(strcmp(cmd[0], "ingest") == 0)
        {
            int ret;
            if(getIntArg(cmd, 'n', ret) != NSSC_STATUS_SUCCESS || ret < 1)
            {
                this->printError("Bad argument!");
            } else
            {
                std::cout << ret << std::endl;
                ingest();
            }
        }
        else if(strcmp(cmd[0], "exit") == 0)
        {
            this->cliON = false;
            exit();
            break;
        }
        else if(strcmp(cmd[0], "help") == 0)
        {
            printf("\033[1;34m[Executor] \033[0mUsage:\n"
                   "  NDI [-r resize frame]             - Toggle raw NDI stream\n"
                   "  exit                              - Close Application\n"
                   "  ingest [-n number of images]      - Start calibration capture\n");
        }
        else
        {
            this->printError("Unknown command!");
        }
        
        free(buf);
    }
}

void CLI::printError(const char* message)
{
    printf("\033[1;34m[Executor] \033[1;31mError: %s\033[0m\n", message);
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