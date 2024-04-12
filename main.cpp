#include "instance.h"
#include "config.h"
#include "utils.h"
#include <fstream>
#include <iomanip>
using namespace std;

int main(int argc, char* argv[])
{
    try{
        Config config(argc, argv);
        Utils::initRand(config.seed);
        Instance instance(config.input, config.param_input);

        istringstream iss(instance.instanceName);
        string id;
        getline(iss, id, '-');
        string depoSetting;
        getline(iss, depoSetting, '-');

        istringstream isss(instance.paramfileName);
        string batteryID;
        getline(isss, batteryID, '-');
        getline(isss, batteryID, '-');
        batteryID = batteryID[0];

        string tauFile = id + "-" + depoSetting + "-" + batteryID + "-tau.txt";
        ofstream outFile(tauFile);
        outFile << setw(10) << "request_id";
        outFile << setw(9) << "tau_in";
        outFile << setw(9) << "tau_out";
        outFile << endl;

        for (int customer = 1; customer < instance.num_nodes; ++customer){
            double flight_time_in = instance.tdrone_in(customer) ;
            double flight_time_out = instance.tdrone_out(customer) ;
            stringstream ss;
            ss << setw(9);
            ss << std::fixed << setprecision(0) << flight_time_in;
            ss << setw(9);
            ss << std::fixed << setprecision(0) << flight_time_out;
            string v_text = ss.str();
            if (instance.tdrone(customer) < 0)
                v_text = "        -        -";

            outFile << setw(10) << customer;
            outFile << v_text;
            outFile << endl;
        }
        cout << "DONE " << instance.instanceName << endl;
    }
    catch (const string& e) { std::cerr << "EXCEPTION | " << e << "\n"; }

    return 0;
}
