#ifndef UTILS_H
#define UTILS_H
#include <algorithm>
#include <random>
#include <ctime>
#include <chrono>
#include <vector>
#include <iostream>
#include <math.h>
#include <assert.h>
#include <map>

class Utils
{
//    static std::mt19937 mt;
    static inline std::mt19937 mt = std::mt19937(0);


public:
    static void initRand(int seed){
        Utils::mt = std::mt19937(seed);
    }

    static int integer_random_generator(const int &a , const int &b ){
        assert(b>a);
        return std::uniform_int_distribution<int>{a, b-1}(Utils::mt);
    }

    static double real_random_generator(const double& a, const double& b){
        return std::uniform_real_distribution<double>{a, b}(Utils::mt);
    }

    static double random_choice(const std::vector<double> &vec)
    {
        int rnd = integer_random_generator(0, vec.size());
        return vec[rnd];
    }

    static int biased_selection(const std::vector<double> &vec){
        std::discrete_distribution<> d(vec.begin(), vec.end());

        return d(mt);
    }

        static void shuffle(std::vector<int> &vec){
            std::shuffle(vec.begin(), vec.end(), Utils::mt);
        }

        template <typename T>
        static void remove(std::vector<T> &c, T &element){
            for (auto it = c.begin(); it != c.end(); /* "it" updated inside loop body */ ){
                if (*it == element){
                    it = c.erase(it);
                    break;
                }
                else {
                    ++it;
                }
            }
        }

        template <typename T>
        static void print_vec(std::vector<T> &c){
            for (auto i : c)
                std::cout << i << " ";

            std::cout << "\n";
        }

        static void reverse(std::vector<int> &route, int i, int j){

        }

        static double round(double val){
            return roundf(val * 100000) / 100000;
        }

        static double round(double val, int n){
            const double multiplier = std::pow(10.0, n);
            return std::ceil(val * multiplier) / multiplier;
        }

        static int sumVec_int(std::vector<int>& vec){
            int sum = 0;
            for (int& i : vec)
                sum += i;
            return sum;
        }

        static int position(std::vector<int>& vec, int element){
            auto it = std::find(vec.begin(), vec.end(), element);
            if (it == vec.end())
            {
                // element not in vector
                assert(false);
                return -1;
            } else
            {
              return std::distance(vec.begin(), it);
            }
        }

        static std::vector<int> gen_vector(int a, int b){
            std::vector<int> vec;
            for (int i = a; i < b; ++i){
                vec.push_back(i);
            }
            return vec;
        }

        static double polarAngle(double xDepot, double yDepot, double xRef, double yRef, double x, double y)
        {
            double Dx_R = xRef-xDepot;
            double Dy_R = yRef-yDepot;
            double Dx = x-xDepot;
            double Dy = y-yDepot;
            double a = acos(
                        ( (Dx_R * Dx + Dy_R * Dy) /
                          ( sqrt(Dx_R*Dx_R + Dy_R*Dy_R) * sqrt(Dx*Dx + Dy*Dy) ) )
                        ) * 180.0 / 3.14159265;

            //    if (a <= 180.0){
            //        return a;
            //    } else {
            //        return 360.0 - a;
            //    }
            return a;
        }

        static double angle(double new_ray, double value)
        {
            double v;
            v = abs(value - new_ray);
            if (v > 180)
                return 360 - v;
            return v;

        }

        static bool contain_pair(std::multimap<int,int>& container, int &i, int &j){
            auto it = container.find(i);
            while (it != container.end()) {
                if (it->first != i)
                    return false;
                if (it->second == j)
                    return true;
                ++it;
            }
            return false;
        }

    private:

};

// dynamic seed


// fixed seed
//std::mt19937 Utils::mt = std::mt19937(0);

#endif // UTILS_H
