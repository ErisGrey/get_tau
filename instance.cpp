#include "instance.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <map>
#include <string>
#include <iostream>
#include "utils.h"
#include <boost/filesystem.hpp>

Instance::Instance(string instanceFile, string paramsFile)
{
    boost::filesystem::path p(instanceFile);
    instanceName = p.filename().string();

    boost::filesystem::path pp(paramsFile);
    paramfileName = pp.filename().string();

    read_input(instanceFile);
    read_otherParams(paramsFile);
    initialize();

    energyModel = new EnergyModel ( h_flight,  v_takeoff,  v_landing,  v_cruise_default,  v_cruise_max,  air_density_rho,  peukert_constant,  battery_reserve,  payload_weight_max);
    dconfig = new DroneConfig(1,
                          frame_weight,
                          battery_weight,
                          num_propellers,
                          rotor_blade_radius,
                          battery_capacity,
                          discharge_rate,
                          voltage,
                          battery_reserve    );
    dconfig->flightTime.resize(num_nodes, pair(-1,-1));

    cout.precision(10);
    for (int customer = 1; customer < getNum_nodes(); ++customer){
//        cout << "CUSTOMER " << customer << ": " << getWeight(customer) << endl;
        double dist = dist_drone[0][customer] * 100000; // km to m, normally *1000, analysis *100000
//        cout << customer << " " << dist << endl;

        vector<double> optimal_setting = energyModel->get_optimal_power(*dconfig, getWeight(customer), dist);
        if (optimal_setting[0] != -1){
//            dconfig->flightTime[customer] =
//                    dist/optimal_setting[0] + // inbound trip in second
//                    dist/optimal_setting[1] + // outbound trip in second
//                    2 * energyModel->h_flight / energyModel->v_takeoff + // 2 times takeoff in second
//                    2 * energyModel->h_flight / energyModel->v_landing  // 2 time landing in second
//                    ;
            dconfig->flightTime[customer].first =
                    dist/optimal_setting[0] + // inbound trip in second
                    energyModel->h_flight / energyModel->v_takeoff + //  times takeoff in second
                    energyModel->h_flight / energyModel->v_landing  //  time landing in second
                    ;
            dconfig->flightTime[customer].second =
                    dist/optimal_setting[1] + // outbound trip in second
                    energyModel->h_flight / energyModel->v_takeoff + //  times takeoff in second
                    energyModel->h_flight / energyModel->v_landing  //  time landing in second
                    ;
            dconfig->flightTime[customer].first = Utils::round( dconfig->flightTime[customer].first/60 , 0 ); // /60 to minute
            dconfig->flightTime[customer].second = Utils::round( dconfig->flightTime[customer].second/60 , 0 ); // /60 to minute
//                cout << optimal_setting[0] << "-" << optimal_setting[1] << " ";
        } else {
//                cout  << "X ";
        }
//        cout << endl;
    }
}

Instance::~Instance()
{
    delete energyModel;
    delete dconfig;
}

void Instance::read_input(const string &inputFile)
{
    ifstream myFile(inputFile);
    if (!myFile.is_open())
    {
        cout << "Unable to open instance file \n";
        exit(0);
    }

    num_nodes = 0;
    string line;
    getline(myFile, line); // skip first line
    while (getline(myFile, line))
    {
        istringstream iss(line);
        string token;
        iss >> token;
        iss >> token;
        iss >> token; // x
        x.push_back( stod(token) );
        iss >> token; // y
        y.push_back( stod(token) );
        iss >> token; // w
        weight.push_back( stod(token) );
        num_nodes++;
    }
    weight[0] = 0;

    for (int i = 0; i < num_nodes; ++i){
        weight[i] = std::floor(weight[i] * 1000) / 1000; /*Utils::round(weight[i], 3);*/
        weight[i] = Utils::round(weight[i], 2);
    }
    myFile.close();
}

void Instance::read_otherParams(const string &paramsFile)
{
    ifstream myFile(paramsFile);
    if (!myFile.is_open()){
        cout << "Unable to open params file \n";
        exit(0);
    }
    string token;
    myFile >> token >> drone_prep_time;
    myFile >> token >> frame_weight;
    myFile >> token >> num_propellers;
    myFile >> token >> rotor_blade_radius;
    myFile >> token >> peukert_constant;
    myFile >> token >> h_flight;
    myFile >> token >> v_cruise_default;
    myFile >> token >> v_cruise_max;
    myFile >> token >> v_takeoff;
    myFile >> token >> v_landing;
    myFile >> token >> air_density_rho;
    myFile >> token >> truck_speed;
    myFile >> token >> battery_weight;
    myFile >> token >> battery_capacity;
    myFile >> token >> discharge_rate;
    myFile >> token >> voltage;
    myFile >> token >> battery_reserve;
    myFile >> token >> payload_weight_max;
}

void Instance::initialize()
{
    // Compute distance and time matrix
    dist_drone.resize(num_nodes);
    for (int i = 0; i < num_nodes; i++)
        dist_drone[i].resize(num_nodes);

    for (int i = 0; i < num_nodes; i++) {
        for (int j = 0; j < num_nodes; j++) {
            float euc_d = pow(pow(x[i] - x[j],2) + pow(y[i] - y[j],2), 0.5);
            dist_drone[i][j] = euc_d;
        }
    }
}

double Instance::tdrone(const int &customer) const
{
    return dconfig->flightTime[customer].first + // bay di
            dconfig->flightTime[customer].second; // bay ve
}

double Instance::tdrone_in(const int &customer) const
{
    return dconfig->flightTime[customer].first;
}

double Instance::tdrone_out(const int &customer) const
{
    return dconfig->flightTime[customer].second;
}

double Instance::serviceTime_drone(const int &customer) const
{
    return tdrone(customer) + drone_prep_time;
}

double Instance::ttruck(int i, int j)
{
    return time_truck[i][j];
}

double Instance::dtruck(int i, int j)
{
    return dist_truck[i][j];
}

int Instance::getNum_nodes() const
{
    return num_nodes;
}

double Instance::getWeight(int i) const
{
    return weight[i];
}

bool Instance::isTruckonly(int customer)
{
    if (dconfig->flightTime[customer].first == -1)
        return true;
    else
        return false;
}

void Instance::export_stat()
{
    // count inreach, overweight, drone-el
    int inreach = 0;
    int overw = 0;
    int overw_inreach = 0;
    int el = 0;
//    for (int i = 1; i < num_nodes; ++i){
//        if (tdrone(i)/v_drone <= endurance_drone)
//            ++inreach;

//        if (weight[i] > cap_drone)
//            ++overw;

//        if (tdrone(i)/v_drone <= endurance_drone && weight[i] > cap_drone && ceil(weight[i] / cap_drone) <= num_drones)
//            ++overw_inreach;

//        if (tdrone(i)/v_drone <= endurance_drone && ceil(weight[i] / cap_drone) <= num_drones)
//            ++el;
//    }

    string s = "";
    s += instanceName + ",";
    s += to_string(num_nodes-1) + ",";
    s += to_string(inreach) + ",";
    s += to_string(overw) + ",";
    s += to_string(overw_inreach) + ",";
    s += to_string(el);

    std::ofstream outCSV;
    outCSV.open("instance_stat_summary.csv", std::ios_base::app);
    outCSV << s << "\n" << std::flush;
    outCSV.close();
}

template<class Container>
void Instance::split(const std::string &str, Container &cont, char delim)
{
    cont.clear();
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delim)) {
        if (token != "")
            cont.push_back(token);
    }
}
