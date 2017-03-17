/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "InternalForceSeparator.h"
/* samples */
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <vector>
#include <map>
#include <sys/time.h>
#include <iomanip>

class testInternalForceSeparator
{
protected:
    InternalForceSeparator ifs;
public:
    std::vector<std::string> arg_strs;
    testInternalForceSeparator () : ifs() {};
    void test(bool use_moment, bool use_qp=false, bool print=true) {
        /* set ee_info */
        std::vector<std::string> arm_names(2); arm_names[0] = "rarm"; arm_names[1] = "larm";
        std::map<std::string, EndEffectorInfo> ee_info;
        ee_info["larm"].abs_force = hrp::Vector3(0, 1,-2);
        ee_info["rarm"].abs_force = hrp::Vector3(0,-1,-2);
        ee_info["larm"].ref_force = hrp::Vector3(0, 0.1,0);
        ee_info["rarm"].ref_force = hrp::Vector3(0,-1.0,0);
        ee_info["larm"].pos = hrp::Vector3(0, 1,0);
        ee_info["rarm"].pos = hrp::Vector3(0,-1,0);
        ee_info["larm"].contact_force_dir = hrp::Vector3(0, 1,0);
        ee_info["rarm"].contact_force_dir = hrp::Vector3(0,-1,0);
        /* test function */
        ifs.printp = print;
        ifs.debug_level = 2;
        ifs.useMoment(use_moment);
        ifs.useQP(use_qp);
        ifs.calcInternalForce(ee_info);
    }
    void test0 (bool print=true)
    {
        if (print) std::cerr << "test0 : Set" << std::endl;
        test(true, false, print);
    };
    void test1 (bool print=true)
    {
        if (print) std::cerr << "test1 : Set" << std::endl;
        test(false, false, print);
    };
    void test2 (bool print=true)
    {
        if (print) std::cerr << "test2 : Set" << std::endl;
        test(false, true, print);
    };
};

void print_usage ()
{
    std::cerr << "Usage : testInternalForceSeparator COMMAND [OPTION]" << std::endl;
    std::cerr << " COMMAND should be:" << std::endl;
    std::cerr << "  --test0 : calc internal force using PseudoInverse" << std::endl;
    std::cerr << "  --test1 : calc internal force (mode: using only force)" << std::endl;
    std::cerr << "  --test2 : calc internal force (mode: using only force & QP)" << std::endl;
    std::cerr << " OPTION should be:" << std::endl;
    std::cerr << "  --silent : do not print results of calculation" << std::endl;
};

int main(int argc, char* argv[])
{
    struct timeval t_calc_start, t_calc_end;
    int ret = 0;
    bool print_results = true;
    if (argc >= 2) {
        testInternalForceSeparator tifs;
        for (int i = 1; i < argc; ++ i) {
            tifs.arg_strs.push_back(std::string(argv[i]));
        }
        if(std::find(tifs.arg_strs.begin(), tifs.arg_strs.end(), "--silent") != tifs.arg_strs.end()) {
            print_results = false;
        }

        gettimeofday(&t_calc_start, NULL);
        if (std::string(argv[1]) == "--test0") {
            tifs.test0(print_results);
        } else if (std::string(argv[1]) == "--test1") {
            tifs.test1(print_results);
        } else if (std::string(argv[1]) == "--test2") {
            tifs.test2(print_results);
        } else {
            print_usage();
            ret = 1;
        }
        gettimeofday(&t_calc_end, NULL);
        std::cout<<(double)(t_calc_end.tv_sec - t_calc_start.tv_sec) + (t_calc_end.tv_usec - t_calc_start.tv_usec)/1000000.0<<std::endl;
    } else {
        print_usage();
        ret = 1;
    }
    return ret;
}

