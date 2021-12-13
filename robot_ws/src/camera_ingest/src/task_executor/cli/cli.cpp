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

        char *token;
        token = strtok(buf, del);

        std::vector<char*> cmd = {};

        NSSC_STATUS arg_status = NSSC_STATUS_SUCCESS;

        while (token != NULL)
        {
            while(token[strlen(token) - 1] == ' ')
                token[strlen(token) - 1] = '\0';
            
            if(strlen(token) <= 1)
            {
                char *numtoken;
                numtoken = strtok(NULL, del);
                if(numtoken != NULL && isdigit(numtoken[0]))
                {
                    while(numtoken[strlen(numtoken) - 1] == ' ')
                        numtoken[strlen(numtoken) - 1] = '\0';

                    char *out = new char[strlen(numtoken) + strlen(token) + 3];
                    std::strcpy(out, token);
                    std::strcat(out, " -");
                    std::strcat(out, numtoken);

                    cmd.push_back(out);

                    token = strtok(NULL, del);
                    if(token == NULL) break;
                    
                } else
                {
                    arg_status = NSSC_CLI_ARGUMENT_TYPE_ERROR;
                    break;
                }
            } else
            {
                cmd.push_back(token);
                token = strtok(NULL, del);
            }
        }

        if(arg_status != NSSC_STATUS_SUCCESS)
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
        else if(strcmp(cmd[0], "ingest") == 0)
        {
            ingest();
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

NSSC_STATUS CLI::getIntArg(std::vector<char*> cmd, char par, int& ret)
{
    for (int i = 1; i < cmd.size(); i++)
    {
        if (cmd[i][0] == par)
        {
            char *arg;
            arg = strtok(cmd[i], arg_del);
            arg = strtok(NULL, del);
            while (arg[strlen(arg) - 1] == ' ')
                arg[strlen(arg) - 1] = '\0';
            while (arg[0] == ' ')
                arg++;
            try
            {
                ret = boost::lexical_cast<int>(arg);
                return NSSC_STATUS_SUCCESS;
            }
            catch(boost::bad_lexical_cast &)
            {
                return NSSC_CLI_ARGUMENT_TYPE_ERROR;
            }
        }
    }
}

NSSC_STATUS CLI::getBoolArg(std::vector<char*> cmd, char par, bool& ret)
{
    for (int i = 1; i < cmd.size(); i++)
    {
        if (cmd[i][0] == par)
        {
            char *arg;
            arg = strtok(cmd[i], arg_del);
            arg = strtok(NULL, arg_del);
            while (arg[strlen(arg) - 1] == ' ')
                arg[strlen(arg) - 1] = '\0';
            while (arg[0] == ' ')
                arg++;
            try
            {
                ret = boost::lexical_cast<bool>(arg);
                return NSSC_STATUS_SUCCESS;
            }
            catch(boost::bad_lexical_cast &)
            {
                return NSSC_CLI_ARGUMENT_TYPE_ERROR;
            }
        }
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