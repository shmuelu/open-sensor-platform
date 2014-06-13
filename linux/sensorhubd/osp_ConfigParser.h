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

#ifndef __OSP__CONFIG_PARSER_H_
#define __OSP__CONFIG_PARSER_H_

#include <stdint.h>
#include <string>
#include <vector>
#include "osp-types.h"


//! parses command line parameters + config file with elements specific to OSP
namespace OSP {
    class ConfigParser {
public:

        static int parse(const char *const configFileName,
            const char *const defaultProtocol = NULL);

private:

        static int process_line( const char *const line);

        static bool tryParseInt( const std::string& first, const std::string & second);
        static bool tryParseFloat( const std::string& first, const std::string & second);

        ConfigParser();
        ~ConfigParser(){}
    };
}
#endif // _OSP_CONFIGPARSER_H_
