#ifndef CONFIG_H
#define CONFIG_H
#include <iostream>
#include <string>
#include <chrono>
#include <random>
using namespace std;

class Config
{
public:

    string input;
    string param_input = "/home/toto/Code/cumulative-drone/instances/params.txt";
    string output;
    double time_limit = 30.0;
    int max_iter = 10000;
    double cooling_rate = 0.9995;
    double threshold = 0.15;
    double max_removal_rate = 0.3;
    int seed;

    Config(int argc, char* argv[]) {
        for (int i = 1; i < argc; ++i) {
            string key = argv[i];

            if (key == "-i") {
                string value = argv[++i];
                input = value;
                found_input = true;
            }

            else if (key == "-pi") {
                string value = argv[++i];
                param_input = value;
                found_param_input = true;
            }

            else if (key == "-o") {
                string value = argv[++i];
                output = value;
                found_output = true;
            }

            else if (key == "-t") {
                string value = argv[++i];
                time_limit = stof(value);
                found_time_limit = true;
            }

            else if (key == "-it") {
                string value = argv[++i];
                max_iter = stoi(value);
                found_max_iter = true;
            }

            else if (key == "-c"){
                string value = argv[++i];
                cooling_rate = stod(value);
                found_cooling_rate = true;
            }

            else if (key == "-th"){
                string value = argv[++i];
                threshold = stod(value);
                found_threshold = true;
            }

            else if (key == "-rmr"){
                string value = argv[++i];
                max_removal_rate = stod(value);
                found_max_removal_rate = true;
            }

            else if (key == "-s"){
                string value = argv[++i];
                seed = stoi(value);
                found_seed = true;
            }

            else {
                cerr << "Invalid argument !!!\n";
                exit(0);
            }
        }

        if (!found_input) {
            cerr << "ERROR: Input is required !!!\n";
            exit(0);
        }
        if (!found_output) {
    //	    cout << "Warning: Output is missing !!!!\n";
        }
        if (!found_time_limit) {
    //	    cout << "Warning: time_limit default = 30.0s\n";
        }
        if (!found_max_iter) {
//            cout << "Warning: max_iter default = 50,000 iterations\n";
        }
        if (!found_cooling_rate) {
//            cout << "Warning: cooling_rate default = 0.9995\n";
        }
        if (found_seed) {
//            cout << "Fixed seed = " << seed << "\n";
        } else {
            seed = chrono::high_resolution_clock::now().time_since_epoch().count();
//            cout << "Random seed = " << seed << "\n";
        }
//        cout << flush;
    }
private:
    bool found_input = false;
    bool found_param_input = false;
    bool found_output = false;
    bool found_time_limit = false;
    bool found_max_iter = false;
    bool found_cooling_rate = false;
    bool found_threshold = false;
    bool found_max_removal_rate = false;
    bool found_seed = false;
};

#endif // CONFIG_H
