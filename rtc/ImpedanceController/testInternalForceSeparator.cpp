/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "InternalForceSeparator.h"
/* samples */
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <vector>
#include <map>

#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif
#ifndef deg2rad
#define deg2rad(deg) (deg * M_PI / 180)
#endif

class testInternalForceSeparator
{
protected:
    double dt; /* [s] */
    InternalForceSeparator ifs;
    bool use_gnuplot;
    void gen_pattern_and_plot (const std::vector<double>& force_vec,
                               const std::vector<double>& time_vec)
    {
        parse_params();
        std::string fname("/tmp/plot-ifs.dat");
        FILE* fp = fopen(fname.c_str(), "w");
        double detect_time;
        bool detected = false;
        // for (size_t i = 0; i < time_vec.size();i++) {
        //     bool tmp_detected = ifs.checkDetection(force_vec[i]);
        //     if (tmp_detected && !detected) {
        //         detect_time = time_vec[i];
        //         detected = true;
        //     }
        //     fprintf(fp, "%f %f %f %f\n", time_vec[i], force_vec[i], ifs.getFilteredWrench(), ifs.getFilteredDwrench(), detected);
        // }
        fclose(fp);
        if (use_gnuplot) {
            // plot
            std::string titles[2] = {"Wrench", "Dwrench"};
            //   plot pos
            FILE* gp = popen("gnuplot", "w");
            fprintf(gp, "set multiplot layout 2, 1 title 'Results'\n");
            fprintf(gp, "set xlabel 'Time [s]'\n");
            fprintf(gp, "set ylabel 'Wrench'\n");
            fprintf(gp, "set arrow from %f,%f to %f,%f\n", detect_time, force_vec.front(), detect_time, force_vec.back());
            fprintf(gp, "plot '/tmp/plot-ifs.dat' using 1:2 with lines title 'Wrench' lw 4, '/tmp/plot-ifs.dat' using 1:3 with lines title 'FilteredWrench' lw 4\n");
            fprintf(gp, "set xlabel 'Time [s]'\n");
            fprintf(gp, "set ylabel 'Dwrench'\n");
            fprintf(gp, "plot '/tmp/plot-ifs.dat' using 1:4 with lines title 'Dwrench' lw 4\n");
            fflush(gp);
            double tmp;
            std::cin >> tmp;
            pclose(gp);
        }
    };
public:
    std::vector<std::string> arg_strs;
    testInternalForceSeparator (const double _dt = 0.004) : dt(_dt), ifs(), use_gnuplot(true) {};
    void test0 ()
    {
        std::cerr << "test0 : Set" << std::endl;
        double tm = 0.0, total_tm = 2.0, df = 10;
        std::vector<double> time_vec, force_vec;
        for (size_t i = 0; i < static_cast<size_t>(total_tm/dt);i++) {
            time_vec.push_back(tm);
            if (i*dt < total_tm*0.2) {
                force_vec.push_back(0);
            } else if (i*dt > total_tm*0.8) {
                force_vec.push_back(df*total_tm*(0.8-0.2));
            } else {
                force_vec.push_back(df*(i*dt-total_tm*0.2));
            }
            tm += dt;
        }
        /* set ee_info */
        std::vector<std::string> arm_names(2); arm_names[0] = "rarm"; arm_names[1] = "larm";
        std::map<std::string, EndEffectorInfo> ee_info;
        ee_info["larm"].abs_force = hrp::Vector3(0,-1,2);
        ee_info["rarm"].abs_force = hrp::Vector3(0, 1,2);
        ee_info["larm"].pos = hrp::Vector3(0,1,0);
        ee_info["rarm"].pos = hrp::Vector3(0, -1,0);
        /* test function */
        ifs.printp = true;
        ifs.calcInternalForce(ee_info);
        //gen_pattern_and_plot (force_vec, time_vec);
    };
    void parse_params ()
    {
      for (unsigned int i = 0; i < arg_strs.size(); ++ i) {
          if ( arg_strs[i]== "--use-gnuplot" ) {
              if (++i < arg_strs.size()) use_gnuplot = (arg_strs[i]=="true");
          }
      }
    };
};

void print_usage ()
{
    std::cerr << "Usage : testInternalForceSeparator [option]" << std::endl;
    std::cerr << " [option] should be:" << std::endl;
    std::cerr << "  --test0 : Set ref force" << std::endl;
    // std::cerr << "  --test1 : Move pos and rot" << std::endl;
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
        // } else if (std::string(argv[1]) == "--test1") {
        //     tiog.test1();
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

