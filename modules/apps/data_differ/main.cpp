#include <iostream>

#include "diff_controller.h"

#include "global_cache.h"
#include "glog/logging.h"
#include "glog/log_severity.h"

int main(int argc, const char* argv[]) {
    if(argc < 4){
        cout << "usage: " << argv[0] << " <mesh_path> <kxs_path> <log_path> " << endl;
        return -1;
    }

    int res = 0;
    string mesh_path = argv[1];
    string diff_path = argv[2];
    string out_path = argv[3];

    google::InitGoogleLogging(argv[0]);
    google::LogToStderr();
    google::SetLogDestination(0, (out_path + "/output.log.").c_str());

    GlobalCache * globalCache = GlobalCache::GetInstance();
    globalCache->set_mesh_data_path(mesh_path);
    globalCache->set_diff_data_path(diff_path);
    globalCache->set_out_path(out_path);

    DiffController controler;
    if(!controler.Differing())
        res = -1;

    cout<<"finished^_*"<<endl;

    google::ShutdownGoogleLogging();

    return res;
}