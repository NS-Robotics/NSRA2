/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Noa Sendlhofer
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Noa Sendlhofer

#include "cli.h"
#include "nssc_errors.h"

/*  TEST CASES:
    --r
    -r-r
    -rrr
    -r-
*/

nssc::NSSC_STATUS nssc::application::CLI::__procArg(char *buf, std::vector<char *> &cmd)
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

nssc::NSSC_STATUS nssc::application::CLI::getStrArg(std::vector<char *> cmd, char par, char **ret)
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

nssc::NSSC_STATUS nssc::application::CLI::getIntArg(std::vector<char *> cmd, char par, int &ret)
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

nssc::NSSC_STATUS nssc::application::CLI::getBoolArg(std::vector<char *> cmd, char par, bool &ret)
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