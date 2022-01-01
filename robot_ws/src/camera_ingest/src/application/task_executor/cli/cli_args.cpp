#include "cli.h"

/*  TEST CASES:
    --r
    -r-r
    -rrr
    -r-
*/

NSSC_STATUS CLI::__procArg(char *buf, std::vector<char *> &cmd)
{
    char *token;
    token = strtok(buf, del);

    NSSC_STATUS arg_status = NSSC_STATUS_SUCCESS;

    while (token != NULL)
    {
        while (token[strlen(token) - 1] == ' ')
            token[strlen(token) - 1] = '\0';

        if (strlen(token) <= 1)
        {
            char *numtoken;
            numtoken = strtok(NULL, del);
            if (numtoken != NULL && isdigit(numtoken[0]))
            {
                while (numtoken[strlen(numtoken) - 1] == ' ')
                    numtoken[strlen(numtoken) - 1] = '\0';

                char *out = new char[strlen(numtoken) + strlen(token) + 3];
                std::strcpy(out, token);
                std::strcat(out, " -");
                std::strcat(out, numtoken);

                cmd.push_back(out);

                delete[] out;

                token = strtok(NULL, del);
                if (token == NULL)
                    break;
            }
            else
            {
                arg_status = NSSC_CLI_ARGUMENT_TYPE_ERROR;
                break;
            }
        }
        else
        {
            cmd.push_back(token);
            token = strtok(NULL, del);
        }
    }

    return arg_status;
}

NSSC_STATUS CLI::getStrArg(std::vector<char *> cmd, char par, char **ret)
{
    for (int i = 1; i < cmd.size(); i++)
    {
        if (cmd[i][0] == par)
        {
            char *out;
            out = new char[strlen(cmd[i])];

            strcpy(out, cmd[i]);

            while (out[strlen(out) - 1] != '"' && strlen(out) > 0)
                out[strlen(out) - 1] = '\0';
            out[strlen(out) - 1] = '\0';
            while (out[0] != '"' && strlen(out) > 0)
                out++;
            out++;

            *ret = (char *)malloc(strlen(out));
            strcpy(*ret, out);

            if (strlen(out) <= 0)
            {
                return NSSC_CLI_ARGUMENT_TYPE_ERROR;
            }
            else
            {
                return NSSC_STATUS_SUCCESS;
            }
        }
    }
    return NSSC_CLI_ARGUMENT_TYPE_ERROR;
}

NSSC_STATUS CLI::getIntArg(std::vector<char *> cmd, char par, int &ret)
{
    for (int i = 1; i < cmd.size(); i++)
    {
        if (cmd[i][0] == par)
        {
            char *arg;
            arg = strtok(cmd[i], arg_del);
            arg = strtok(NULL, arg_del);
            if (arg == NULL)
                return NSSC_CLI_ARGUMENT_TYPE_ERROR;
            while (arg[strlen(arg) - 1] == ' ')
                arg[strlen(arg) - 1] = '\0';
            while (arg[0] == ' ')
                arg++;
            try
            {
                ret = boost::lexical_cast<int>(arg);
                return NSSC_STATUS_SUCCESS;
            }
            catch (boost::bad_lexical_cast &)
            {
                return NSSC_CLI_ARGUMENT_TYPE_ERROR;
            }
        }
    }
    return NSSC_CLI_ARGUMENT_TYPE_ERROR;
}

NSSC_STATUS CLI::getBoolArg(std::vector<char *> cmd, char par, bool &ret)
{
    for (int i = 1; i < cmd.size(); i++)
    {
        if (cmd[i][0] == par)
        {
            char *arg;
            arg = strtok(cmd[i], arg_del);
            arg = strtok(NULL, arg_del);
            if (arg == NULL)
                return NSSC_CLI_ARGUMENT_TYPE_ERROR;
            while (arg[strlen(arg) - 1] == ' ')
                arg[strlen(arg) - 1] = '\0';
            while (arg[0] == ' ')
                arg++;
            try
            {
                ret = boost::lexical_cast<bool>(arg);
                return NSSC_STATUS_SUCCESS;
            }
            catch (boost::bad_lexical_cast &)
            {
                if (strcmp(arg, "True") == 0 || strcmp(arg, "true") == 0)
                {
                    ret = 1;
                    return NSSC_STATUS_SUCCESS;
                }
                else if (strcmp(arg, "False") == 0 || strcmp(arg, "false") == 0)
                {
                    ret = 0;
                    return NSSC_STATUS_SUCCESS;
                }
                else
                {
                    return NSSC_CLI_ARGUMENT_TYPE_ERROR;
                }
            }
        }
    }
    return NSSC_CLI_ARGUMENT_TYPE_ERROR;
}