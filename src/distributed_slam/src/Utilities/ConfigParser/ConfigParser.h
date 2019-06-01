//
// Created by Mason U'Ren on 2019-02-14.
//

#ifndef C_CONFIGPARSER_H
#define C_CONFIGPARSER_H

#include <shared_structs/SLAMConfigIn.h>
#include <shared_structs/SharedMemoryStructs.h>
#include <nholmann_json/json.hpp>

#include <iostream>
#include <sstream>
#include <fstream>

using json = nlohmann::json;

class ConfigParser {
public:
    ConfigParser() = default;
    ~ConfigParser() = default;

    bool loadJSONFromFile(const std::string &filePath, json *dataPtr);
    void parseConfig(SYS_CONFIG_IN *in, json *dataPtr);
};


#endif //C_CONFIGPARSER_H
