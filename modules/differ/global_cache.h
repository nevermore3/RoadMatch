//
// Created by gaoyanhong on 2018/11/13.
//

#ifndef MATCH_CORE_GLOBALCACHE_H
#define MATCH_CORE_GLOBALCACHE_H

#include <string>

using namespace std;


class GlobalCache {

public:
    static GlobalCache *GetInstance() {
        static GlobalCache instance_;
        return &instance_;
    }

    void set_mesh_data_path(string data_path) { mesh_data_path_ = data_path; }

    void set_diff_data_path(string data_path) { diff_data_path_ = data_path; }

    void set_out_path(string data_path) { out_path_ = data_path; }


    string mesh_data_path() { return mesh_data_path_; }

    string diff_data_path() { return diff_data_path_; }

    string out_path() { return out_path_; }

protected:
    GlobalCache() {
    };

private:
    string mesh_data_path_;

    string diff_data_path_;

    string out_path_;

    double match_buffer_;

};


#endif //MATCH_CORE_GLOBALCACHE_H
