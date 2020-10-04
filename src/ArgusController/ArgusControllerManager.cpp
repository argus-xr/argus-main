#include "ArgusControllerManager.h"
std::unordered_map<uint64_t, std::shared_ptr<ArgusController>> ArgusControllerManager::controllers;