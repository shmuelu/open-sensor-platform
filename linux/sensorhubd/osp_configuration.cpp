/* Open Sensor Platform Project
 * https://github.com/sensorplatforms/open-sensor-platform
 *
 * Copyright (C) 2013 Sensor Platforms Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/
#include <string.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <algorithm>

#include "osp-types.h"
#include "osp_names.h"
#include "osp_debuglogging.h"
#include "osp_configuration.h"
#include "ospd.h"

/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
//TBD - Move to platform header
#define DEFAULT_MAGNETOMETER_NOISE  1.0f
#define DEFAULT_GYROSCOPE_NOISE     1.0f
#define DEFAULT_ACCELEROMETER_NOISE 1.0f

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E/C L A S S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
/* used for stl sort algorithm:
*/
namespace{
class CompareKeys{
public:
    bool operator()( const std::string & left,
                     const std::string & right){
        if (left.find("protocol") == 0 && right.find("protocol") == 0){
            return left < right;
        } else if (left.find("protocol")== 0){
            return true;
        } else if (right.find("protocol") == 0){
            return false;
        }
        if (left.find("sensor=") == 0 && right.find("sensor=") == 0){
            return left < right;
        } else if (left.find("sensor=")== 0){
            auto leftkey = left;

            return leftkey.erase( 0,7 ) < right;
        } else if (right.find("sensor") == 0){
            auto rightkey = right;
            return left < rightkey.erase(0,7);
        }
        return left < right;
    }
};
}



/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/



/****************************************************************************************************
 * @fn      Init
 *          Init routine
 *
 ***************************************************************************************************/
OSP::OspConfiguration::Init::Init(){
}


/****************************************************************************************************
 * @fn      getConfigItem
 *          Helper routine for getting configuration parameter
 *
 ***************************************************************************************************/
const char* const
OSP::OspConfiguration::getConfigItem( const char* const name ){
    if (name == NULL){
        LOG_Err("Attempt to get config item for null name");
    } else if( configItemsString.find(std::string(name)) != configItemsString.end()){
        if (configItemsString.find(name)->second.size() >= 1){
            return  (configItemsString.find(name)->second)[0];
        }
    }
    return NULL;
}


/****************************************************************************************************
 * @fn      getConfigItemsMultiple
 *          Helper routine for getting configuration parameter
 *
 ***************************************************************************************************/
std::vector<const char* >
OSP::OspConfiguration::getConfigItemsMultiple( const char* const name ){
    std::vector< const char*  > retval;
    if (name == NULL){
        LOG_Err("Attempt to get config item for null name");
    }

    if (name != NULL && configItemsString.find(name) != configItemsString.end()){
        for (auto it = configItemsString.find(name)->second.begin();
             it != configItemsString.find(name)->second.end();
             ++it){
            if(*it)
                retval.push_back( *it );
        }
    }
    return retval;
}


/****************************************************************************************************
 * @fn      getConfigItemFloat
 *          Helper routine for getting configuration parameter
 *
 ***************************************************************************************************/
const float *
OSP::OspConfiguration::getConfigItemFloat( const char* const name ,  unsigned int* size ){
    if (name == NULL){
        if(size) *size = 0;
        return NULL;
    }
    if ( configItemsFloat.end() == configItemsFloat.find(name)){
        if(size)  *size = 0;
        return NULL;
    }

    std::pair< const float*, unsigned int> pair = configItemsFloat[name];
    if(size) *size = pair.second;
    return pair.first;
}


/****************************************************************************************************
 * @fn      getConfigItemIntV
 *          Helper routine for getting configuration parameter
 *
 ***************************************************************************************************/
int
OSP::OspConfiguration::getConfigItemIntV(
        const char* const name,
        const int defaultValue,
        int* status){
    unsigned  int size;
    if(status)
        *status = OSP_STATUS_OK;
    const int* item = NULL;
    if( name == NULL){
        LOG_Err("Attempt to get int item from null name");
        if(status)*status = -1;
        item = NULL;
    } else {
        item = getConfigItemInt( name, &size);
        if (item && size != 1){
            LOG_Err("Request for integer config item %s that is not a single integer value: %d", name, size);
            if(status)*status = -1;
            item = NULL;
        }

    }
    return item?*item:defaultValue;
}


/****************************************************************************************************
 * @fn      getConfigItemInt
 *          Helper routine for getting configuration parameter
 *
 ***************************************************************************************************/
const int *
OSP::OspConfiguration::getConfigItemInt(
        const char* const name,
        unsigned int* size ){
    if (name == NULL){
        if (size) *size = 0;
        return NULL;
    }
    if ( configItemsInt.end() == configItemsInt.find(name)){
        if(size) *size = 0;
        return NULL;
    }
    std::pair< const int*, unsigned int> pair = configItemsInt[name];
    if(size) *size = pair.second;
    return pair.first;
}


/****************************************************************************************************
 * @fn      setConfigItem
 *          Helper routine for setting configuration parameter
 *
 ***************************************************************************************************/
int
OSP::OspConfiguration::setConfigItem(
        const char* const name,
        const char* const value,
        const bool allowMultiple,
        const bool override){
    LOG_Info("Setting config item %s", name);
    int status = -1;
    if ( name == NULL || value == NULL){
        status = -1;
    } else if (!allowMultiple && !override && configItemsString.find( name ) != configItemsString.end() ){
        status = -1;
    } else {
        if (configItemsString.find(name) == configItemsString.end()){
            configItemsString.insert(std::pair<std::string,
                                     std::vector<const char*> >(name, std::vector<const char*>()));
        }
        if (allowMultiple){
            bool found = false;
            for(  auto item =  configItemsString.find(name)->second.begin();
                  item  !=  configItemsString.find(name)->second.end();
                  ++item){
                found = found || (std::string(value) == std::string(*item));
            }
            if(!found){
                char *clone = new char[strlen(value)+1];
                clone[strlen(value)] = 0;
                memcpy( clone, value, strlen(value));
                configItemsString.find(name)->second.push_back(clone);
            }
        } else {
            char *clone = new char[strlen(value)+1];
            clone[strlen(value)] = 0;
            memcpy( clone, value, strlen(value));
            configItemsString.find(name)->second.clear();
            configItemsString.find(name)->second.push_back(clone);
        }
        status = 0;
    }
    return status;
}


/****************************************************************************************************
 * @fn      setConfigItemFloat
 *          Helper routine for setting configuration parameter
 *
 ***************************************************************************************************/
int
OSP::OspConfiguration::setConfigItemFloat(
        const char* const name,
        const float* const value,
        const unsigned int size,
        const bool override){
    LOG_Info("Setting config item %s", name);
    int status = -1;
    if (!override && configItemsFloat.find( name ) != configItemsFloat.end() ){
        status = -1;
    } else {
        if (configItemsFloat.find(name) != configItemsFloat.end() ){
            std::pair< const float * , unsigned int> pair = configItemsFloat[name];
            delete [] pair.first;
            //configItemsFloat.clear();
        }
        std::pair< const float*, unsigned int> pair;
        pair.first = new float[ size ];
        pair.second = size;
        for (unsigned int i =0; i < size; ++i ){
            const_cast< float* >( pair.first )[i] = value[i];
        }
        configItemsFloat.insert(std::pair<std::string, std::pair<const float*,unsigned int> >(name, pair) );
        status = 0;
    }
    return status;
}


/****************************************************************************************************
 * @fn      clear
 *          Helper routine for clearing all configuration parameters
 *
 ***************************************************************************************************/
void
OSP::OspConfiguration::clear(const bool final){
    typedef std::map<std::string,  std::pair< const float *,  unsigned int> >::iterator it_type;
    for(it_type iterator = configItemsFloat.begin(); iterator != configItemsFloat.end(); iterator++) {
        delete[] (*iterator).second.first;
        (*iterator).second.first = NULL;
        (*iterator).second.second = 0;
    }
    typedef std::map<std::string,  std::pair< const int *,  unsigned int> >::iterator intit_type;
    for(intit_type iterator = configItemsInt.begin(); iterator != configItemsInt.end(); iterator++) {
        delete[] (*iterator).second.first;
        (*iterator).second.first = NULL;
        (*iterator).second.second = 0;
    }
    for (auto it = configItemsString.begin();
         it != configItemsString.end();
         ++it){
        for (auto strit = configItemsString.find(it->first)->second.begin();
             strit != configItemsString.find(it->first)->second.end();
             ++strit){
            delete [] *strit;
        }
    }
    configItemsInt.clear();
    configItemsFloat.clear();
    configItemsString.clear();
}


/****************************************************************************************************
 * @fn      setConfigItemInt
 *          Helper routine for setting configuration parameter
 *
 ***************************************************************************************************/
int
OSP::OspConfiguration::setConfigItemInt(
        const char* const name,
        const int* const value,
        const unsigned int size,
        const bool override){
    LOG_Info("Setting config item %s", name);
    int status = -1;
    if (!override && configItemsInt.find( name ) != configItemsInt.end() ){
        status = -1;
    } else {
        if (configItemsInt.find(name) != configItemsInt.end()){
            std::pair< const int * , unsigned int> pair = configItemsInt[name];
            delete [] pair.first;
        }
        std::pair< const int*, unsigned int> pair;
        pair.first = new int[ size ];
        pair.second = size;
        for (unsigned int i =0; i < size; ++i ){
            const_cast< int* >( pair.first )[i] = value[i];
        }
        configItemsInt.insert(std::pair<std::string, std::pair<const int*, unsigned int> >(name, pair) );
        status = 0;
    }
    return status;
}




/****************************************************************************************************
 * @fn      dump
 *          Helper routine for dumping the current configuration to a file for debugging
 *
 ***************************************************************************************************/
int
OSP::OspConfiguration::dump( const char* const filename ){
    FILE* f = ::fopen( filename, "w");
    if (!f){
        LOG_Err("Unable to open file '%s'",filename);
        return -1;
    }
    fprintf(f, "#for hash-code\n");
    fprintf(f, "?format_version = 2.0\n");

    /* collect all keys over string, float, int items for sorting*/
    std::vector< std::string > keys;
    for (auto it = configItemsString.begin();
         it != configItemsString.end();
         ++it){
        if (it->first == "sensor"){
            for( auto item = it->second.begin();
                 item != it->second.end();
                 ++item){
                keys.push_back( it->first+"="+*item );/* for sorting properly*/
            }
        } else{
            if (it->first != "debugOutputFileName" &&
                    it->first != "requested-result")
                keys.push_back( it->first );
        }
    }
    for (auto it = configItemsFloat.begin();
         it != configItemsFloat.end();
         ++it){
        keys.push_back( it->first );
    }
    for (auto it = configItemsInt.begin();
         it != configItemsInt.end();
         ++it){
        keys.push_back( it->first );
    }
    std::sort<  std::vector<std::string>::iterator, CompareKeys>
            ( keys.begin(), keys.end(), CompareKeys());
    /* loope over sorted keys and spit out to file */
    for (auto it = keys.begin();
         it != keys.end();
         ++it){
        const std::string key = *it;
        if (key.find("sensor") == 0 ){
            //handle sensor separately to get better ordering in file
            //and make it "flow" better
            fprintf(f,"\n\n%s\n", key.c_str());
        } else if (configItemsString.find(key) != configItemsString.end()){
            const std::vector<const char*>& items = configItemsString.find(key)->second;
            if( items.size()>1)
                fprintf(f,"\n");

            for(auto it2 = items.begin();
                it2 != items.end();
                ++it2){

                fprintf( f, "%s = \"%s\"\n", key.c_str(),
                         *it2);

            }
        } else if (configItemsFloat.find(key) != configItemsFloat.end()){
            std::pair< const float*, unsigned int> floatarray =
                    configItemsFloat[key];
            fprintf( f, "%s = ", key.c_str());
            for(unsigned int i = 0; i < floatarray.second; ++i){
                if (i==0)
                    fprintf( f, "%f", floatarray.first[i]);
                else
                    fprintf( f, ",%f", floatarray.first[i]);
            }
            fprintf(f, "\n");
        } else if (configItemsInt.find(key) != configItemsInt.end()){
            std::pair< const int*, unsigned int> intarray =
                    configItemsInt[key];
            fprintf( f, "%s = ", key.c_str());
            for(unsigned int i = 0; i < intarray.second; ++i){
                if (i==0)
                    fprintf( f, "%d", intarray.first[i]);
                else
                    fprintf( f, ",%d", intarray.first[i]);
            }
            fprintf(f, "\n");
        }

    }
    fclose(f);

    return 0;

}


extern "C++"{
std::map<std::string, std::vector<const char*> > OSP::OspConfiguration::configItemsString;
std::map<std::string, std::pair< const float*, unsigned int>  > OSP::OspConfiguration::configItemsFloat;
std::map<std::string, std::pair< const int *,  unsigned int>  > OSP::OspConfiguration::configItemsInt;
OSP::OspConfiguration::Init OSP::OspConfiguration::initializer;
}

/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
