/* Open Sensor Platform Project
* https://github.com/sensorplatforms/open-sensor-platform
*
* Copyright (C) 2013 Sensor Platforms Inc.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
//Define module ID
#include <stdio.h>
#include <algorithm>
#include <string.h>

#include "osp_debuglogging.h"
#include "osp_configuration.h"
#include "osp_ConfigParser.h"

#ifdef ANDROID
extern "C" {
size_t getline (char * *lineptr, size_t *n, FILE *stream);
}
#endif

OSP::ConfigParser::ConfigParser()
{
}


bool OSP::ConfigParser::tryParseInt( const std::string& first, const std::string & second)
{
    int values[18];

    if (second.find('.') != std::string::npos) {
        return false;
    }
    int count = sscanf( second.c_str(), " %d, %d, %d, %d, %d, %d,%d, %d, %d,"
                                        "%d, %d, %d,%d, %d, %d, %d, %d, %d",
        &values[0], &values[1], &values[2], &values[3],
        &values[4], &values[5],
        &values[6], &values[7], &values[8],
        &values[9], &values[10], &values[11],
        &values[12], &values[13], &values[14],
        &values[15], &values[16], &values[17]);
    if (count <= 0) return false;
    OspConfiguration::setConfigItemInt( first.c_str(), values, (unsigned int)count );
    return true;
}


bool OSP::ConfigParser::tryParseFloat( const std::string& first,
    const std::string & second)
{
    float values[18];
    int count = sscanf( second.c_str(), " %f, %f, %f, %f, %f, %f,%f, %f,"
                                        "%f, %f, %f, %f,%f, %f, %f, %f, %f, %f",
        &values[0], &values[1], &values[2], &values[3],
        &values[4], &values[5],
        &values[6], &values[7], &values[8], &values[9],
        &values[10], &values[11],
        &values[12], &values[13], &values[14], &values[15],
        &values[16], &values[17]);

    if (count <= 0) return false;
    OspConfiguration::setConfigItemFloat( first.c_str(), values, (unsigned int)count );
    return true;
}


int OSP::ConfigParser::process_line(const char *const line)
{
    int status = 0;
    std::string cppline = line;
    int pos = -1;

    if (cppline.size() &&
        ( cppline[0] != '#') &&
        ( (pos = cppline.find('=')) != std::string::npos) ) {
        std::string first = line;
        first.erase( pos );
        first.erase( std::remove_if( first.begin(), first.end(), ::isspace ), first.end() );
        std::string second = line;
        ++pos;
        while (pos < (int)second.size() && second[pos] == ' ') {
            ++pos;
        }
        bool have_quote = false;
        if ((pos < (int)second.size()) && (second[pos] == '"')) {
            ++pos;
            have_quote = true;
        }
        second.erase( 0, pos);
        if (!have_quote) {
            second.erase( std::remove_if( second.begin(), second.end(), ::isspace ), second.end() );
        } else {
            pos = 0;
            while (pos < (int)second.size() && second[pos] != '"') {
                ++pos;
            }
            if ((pos == (int)second.size()) || (second[pos] != '"')) {
                LOG_Err("Unterminated quote in value. Aborting");
                status = -1;
                goto onerror;
            }
            ++pos;
            while (++pos < (int)second.size()) {
                if ((second[pos] != ' ') && (second[pos] != '\t')) {
                    LOG_Err("Trailing text after closing quote. Aborting");
                    status = -2;
                    goto onerror;
                }
            }
            --pos; //remove trailing quote
            second.erase( pos - 1, second.size());
        }
        if (!tryParseInt( first, second )) {
            if (!tryParseFloat( first, second) ) {
                if ((first == "sensor") || (first == "module") ||
                    ( first.find(".product") != std::string::npos) ) {
                    OspConfiguration::setConfigItem( first.c_str(), second.c_str(), true);
                } else {
                    OspConfiguration::setConfigItem( first.c_str(), second.c_str());
                }
            }
        }
    }
onerror:
    return status;
}


int OSP::ConfigParser::parse(const char *const configFilename,
    const char *const defaultProtocol)
{
    int status = 0;
    FILE *const f = fopen( configFilename, "r");
    size_t size = 0;
    char *line = NULL;

    OspConfiguration::setConfigItem( OspConfiguration::keyFrom("SystemEventProducer",
        OspConfiguration::SENSOR_TYPE).c_str(),
        "SystemEventProducer");

    if (!f) {
        LOG_Err("Unable to read file '%s'", configFilename);
        status = -1;
        goto onerror;
    }
    line = NULL;
    if (getline( &line, &size, f)< 0) {
        LOG_Err("Error reading first line of '%s'", configFilename);
        status = -1;
        goto onerror;
    }
    if (std::string(line).find("#?format_version=2.0") == std::string::npos) {
        free(line);
        line = NULL;
        if (getline( &line, &size, f)< 0) {
            LOG_Err("Error reading second line of '%s'", configFilename);
            status = -1;
            goto onerror;
        }
    }

    if (std::string(line).find("#?format_version=2.0") == std::string::npos) {
        free(line);
        line = NULL;
        LOG_Err("No version info found in '%s'", configFilename);
        status = -1;
        goto onerror;
    } else {
        LOG_Info("Reading config file...");
        free(line);
        line = NULL;
        do {
            status = getline( &line, &size, f);
            if ((status >= 0) && line) {
                ConfigParser::process_line(line);
                free(line);
            }
            line = NULL;
        } while (status != EOF && status >= 0);
        if ((status != EOF) || (status < 0)) {
            if (!feof(f)) {
                LOG_Err("I/O error reading '%s'", configFilename);
                status = -1;
            } else {
                status = 0;
            }
        } else {
            LOG_Info("Done procesing config file.");
            status = 0;
        }
    }
onerror:
    return status;
}
