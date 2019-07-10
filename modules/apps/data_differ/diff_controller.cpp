//
// Created by liujian on 19-7-10.
//

#include "diff_controller.h"

#include "data_manager/data_types.h"
#include "data_manager/data_manager.h"
#include "data_manager/mesh_manage.h"

bool DiffController::Differing() {
    DataManager data_manager;
    data_manager.extent_ =  KDExtent(115.384, 117.5, 39.322, 41.2);
    data_manager.base_data_manager_ = MeshManage::GetInstance();

    if(!data_manager.LoadData()) {
        return false;
    }

    return true;
}