#include "data_manager.h"
#include "diff_data_manager.h"

bool DataManager::LoadData() {
    if(nullptr == base_data_manager_)
        return false;

    if(!base_data_manager_->LoadData(extent_))
        return false;

    if(nullptr == diff_data_manager_)
        return false;

    if (!diff_data_manager_->LoadData(extent_))
        return false;

    return true;
}