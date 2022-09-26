#include <string.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <math.h>
#include <fstream>
#include <cstdlib>
#include <unistd.h>
#include <iomanip>

#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <sw/redis++/redis++.h>

struct Geographic_point
{
    long double longitude, latitude;

    Geographic_point(long double a, long double b)
        : longitude(a)
        , latitude(b)
        {}
};

struct Data_node
{
    Geographic_point point;
    int node_ID;
    double col_idx, row_idx;

    Data_node(int a, double b, double c)
        : point(b,c)
        , node_ID(a)
        , col_idx(0)
        , row_idx(0)
        {}
};

struct Data_road
{
    int road_ID;
    Data_node* A;
    Data_node* B;
    double deg_to_A, deg_to_B;
    double length;
    bool available;
    double max_speed;

    Data_road(int a, Data_node* b, Data_node* c)
        : road_ID(a)
        , A(b)
        , B(c)
        , deg_to_A(0.1)
        , deg_to_B(0.1)
        {init_data_road();}

    long double toRadians(const long double degree)
    {
        long double one_deg = (M_PI) / 180;
        return (one_deg * degree);
    }

    void init_data_road()
    {
        max_speed = 7.001;
        available = true;
        
        length = fget_angular_distance(&A->point, &B->point);
        deg_to_A = fget_bearing(&B->point, &A->point);
        deg_to_B = fget_bearing(&A->point, &B->point);
    }

    double fget_angular_distance(Geographic_point* pointA, Geographic_point* pointB)
    {
        double lat1  = pointA->latitude;
        double long1 = pointA->longitude;
        double lat2  = pointB->latitude;
        double long2 = pointB->longitude;
        
        double R = 6371000;
        double r1 = lat1 * M_PI / 180;
        double r2 = lat2 * M_PI / 180;
        double dl = (lat2 - lat1) * M_PI/180;
        double dd = (long2 - long1) * M_PI/180;

        double a = sin(dl/2) * sin(dl/2) + cos(r1) * cos(r2) * sin(dd/2) * sin(dd/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        // return c;
        return R * c;
    }

    double fget_bearing(Geographic_point* pointA, Geographic_point* pointB)
    {
        double lat1  = deg_to_rad(pointA->latitude);
        double long1 = deg_to_rad(pointA->longitude);
        double lat2  = deg_to_rad(pointB->latitude);
        double long2 = deg_to_rad(pointB->longitude);

        double y = sin(long2 - long1) * cos(lat2);
        double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(long2 - long1);
        double o = atan2(y, x);

        if(o*180/M_PI < 0.0)
        {
            return 360 + o*180/M_PI;
        }

        return o*180/M_PI;
    }

    long double deg_to_rad(const long double degree)
    {
        long double one_deg = (M_PI) / 180;
        return (one_deg * degree);
    }
};

int get_redis_multi_str(sw::redis::Redis* redis, std::string channel, std::vector<std::string>& stockage);
std::string get_redis_str(sw::redis::Redis* redis, std::string channel);

struct sim_robot
{
    Geographic_point* point;
    double hdg;

    sim_robot(double a, double b, double c)
        : hdg(c)
        {point = new Geographic_point(a,b);}

    void update(sw::redis::Redis* redis)
    {
        std::vector<std::string> vect_redis_str;
        get_redis_multi_str(redis, "NAV_GLOBAL_POSITION", vect_redis_str);

        // std::cout << vect_redis_str[1] << " " << std::stod(vect_redis_str[1]) << std::endl;

        point->longitude = std::stod(vect_redis_str[1]);
        point->latitude  = std::stod(vect_redis_str[2]);
        hdg = std::stod(vect_redis_str[3]);
    }
};

void f_sim();
void f_keyboard();
void f_rendering();
void Read_YAML_file(sw::redis::Redis* redis, std::string path, std::vector<Geographic_point>* ref_border);
void set_redis_var(sw::redis::Redis* redis, std::string channel, std::string value);
void pub_redis_var(sw::redis::Redis* redis, std::string channel, std::string value);
int get_multi_str(std::string str, std::vector<std::string>& vec_str);
int64_t get_curr_timestamp();
std::string get_event_str(int ID_event, std::string event_description, std::string event_info);
void read_yaml(sw::redis::Redis* redis, cv::FileStorage* file_mng, std::string channel);
double frequency_to_ms(int frequency);
bool time_is_over(int64_t curr_timestamp, int64_t ref_timestamp, int64_t max_duration_ms);
void print_redis(sw::redis::Redis* redis, std::string channel_str);
std::string get_standard_robot_id_str(sw::redis::Redis* redis);
bool compare_redis_var(sw::redis::Redis* redis, std::string channel, std::string compare);


// DATA MANAGEMENT.
void Read_TXT_file(std::string path, std::vector<Data_node>& vector_node, std::vector<Data_road>& road_vector);

// RENDERING
void Read_JPG_file(std::string path, cv::Mat& img);
void Init_data_map(cv::Mat& map_current, cv::Mat& map_data);
void Project_all_element(std::vector<Geographic_point>& ref_border, std::vector<Data_node>& node_vector, cv::Mat& map_current, cv::Mat& map_data, std::vector<Data_road>& road_vector, bool speed_view);
void project_geo_element(std::vector<Geographic_point>& ref_border, cv::Mat& map_current, int element_type, Geographic_point* position, double hdg);
Geographic_point get_new_position(Geographic_point* start_position, double bearing, double distance);
long double deg_to_rad(const long double degree);
double get_angular_distance(Geographic_point* pointA, Geographic_point* pointB);
double rad_to_deg(double rad);