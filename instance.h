#ifndef INSTANCE_H
#define INSTANCE_H

#include <vector>
#include <string>
#include <set>
#include <energymodel.h>
#include <unordered_map>

using namespace std;


class Instance{
public:
    string instanceName;
    string paramfileName;
    int num_nodes;

    vector<double> x;
    vector<double> y;

    set<int> freeCustomer;
    set<int> truckonlyCustomer;

    DroneConfig* dconfig;
    EnergyModel* energyModel;

    vector<double> weight;
    vector<vector<double>> dist_drone,dist_truck;
    vector<vector<double>> time_truck;


    Instance(string inputFile, string paramFile);
    ~Instance();
    void export_stat();

    int getNum_nodes() const;
    double getWeight(int i) const;
    double tdrone(const int &customer) const;
    double tdrone_in(const int &customer) const;
    double tdrone_out(const int &customer) const;
    double ttruck(int i, int j);
    double dtruck(int i, int j);
    double serviceTime_drone(const int &customer) const;
    bool isTruckonly(int customer);

private:
//    double coupletime;
    double drone_prep_time;
    double frame_weight;
    int num_propellers;

    double rotor_blade_radius;
    double peukert_constant;
    double h_flight; // = 100; // (m) < 400 feet
    double v_cruise_default;
    double v_cruise_max;
    double v_takeoff;
    double v_landing;
    double air_density_rho;
    double truck_speed;
    double battery_weight;
    double battery_capacity;
    double discharge_rate;
    double voltage;
    double battery_reserve;
    double payload_weight_max;

    void read_input(const string &inputFile);
    void read_otherParams(const string &paramsFile);
    void initialize();
    template <class Container>
    void split(const std::string& str, Container& cont, char delim);
};

#endif
