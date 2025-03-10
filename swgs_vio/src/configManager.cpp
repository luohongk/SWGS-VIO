#include "configManager.h"
#include <fstream>

void ConfigManager::loadConfig(const std::string& filePath) {
    try {
        configData = YAML::LoadFile(filePath);
    } catch (const std::exception& e) {
        std::cerr << "Error loading YAML file: " << e.what() << std::endl;
        throw; // 重新抛出异常
    }
}

std::string ConfigManager::getValue(const std::string& key) {
    if (configData[key]) {
        return configData[key].as<std::string>();
    } else {
        throw std::runtime_error("Key not found: " + key);
    }
}

void ConfigManager::updateValue(const std::string& key, const std::string& value) {
    configData[key] = value;
}

void ConfigManager::saveConfig(const std::string& filePath) {
    try {
        std::ofstream fout(filePath);
        fout << configData;
        fout.close();
    } catch (const std::exception& e) {
        std::cerr << "Error saving YAML file: " << e.what() << std::endl;
    }
}
