#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <yaml-cpp/yaml.h>
#include <string>
#include <stdexcept>
#include <iostream>
#include <fstream>

class ConfigManager {
    public:
        static ConfigManager& getInstance() {
            static ConfigManager instance;
            return instance;
        }
    
        void loadConfig(const std::string& filePath);
        std::string getValue(const std::string& key);
        void updateValue(const std::string& key, const std::string& value);
    
    private:
        ConfigManager() {}
        YAML::Node configData;
    };

#endif CONFIGMANAGER_H