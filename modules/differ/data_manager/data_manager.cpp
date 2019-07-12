//
// Created by liujian on 19-7-9.
//

#include "data_manager.h"
#include "diff_data_manager.h"

bool DataManager::LoadData() {
    if(nullptr == base_data_manager_)
        return false;

    //if(!base_data_manager_->LoadData(extent_))
    //    return false;
    DiffDataManager *diff_data_manager_ = DiffDataManager::GetInstance();
    if (!diff_data_manager_->LoadData(extent_))
        return false;

    return true;
}