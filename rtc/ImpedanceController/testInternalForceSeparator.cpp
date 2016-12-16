/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "InternalForceSeparator.h"
/* samples */
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <vector>
#include <map>

class testInternalForceSeparator
{
protected:
    InternalForceSeparator ifs;
public:
    std::vector<std::string> arg_strs;
    testInternalForceSeparator () : ifs() {};
    void test0 ()
    {
        std::cerr << "test0 : Set" << std::endl;
        /* set ee_info */
        std::vector<std::string> arm_names(2); arm_names[0] = "rarm"; arm_names[1] = "larm";
        std::map<std::string, EndEffectorInfo> ee_info;
        ee_info["larm"].abs_force = hrp::Vector3(0,-1,2);
        ee_info["rarm"].abs_force = hrp::Vector3(0, 1,2);
        ee_info["larm"].pos = hrp::Vector3(0,1,0);
        ee_info["rarm"].pos = hrp::Vector3(0, -1,0);
        /* test function */
        ifs.printp = true;
        ifs.debug_level = 2;
        ifs.calcInternalForce(ee_info);
    };
};

void print_usage ()
{
    std::cerr << "Usage : testInternalForceSeparator [option]" << std::endl;
    std::cerr << " [option] should be:" << std::endl;
    std::cerr << "  --test0 : calc internal force using PseudoInverse" << std::endl;
};

int main(int argc, char* argv[])
{
    int ret = 0;
    if (argc >= 2) {
        testInternalForceSeparator tifs;
        for (int i = 1; i < argc; ++ i) {
            tifs.arg_strs.push_back(std::string(argv[i]));
        }
        if (std::string(argv[1]) == "--test0") {
            tifs.test0();
        } else {
            print_usage();
            ret = 1;
        }
    } else {
        print_usage();
        ret = 1;
    }
    return ret;
}

