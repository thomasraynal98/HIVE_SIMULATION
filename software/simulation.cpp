#include "useful.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

std::thread thread_rendering, thread_keyboard, thread_sim;

// VARIABLE FOR RENDERING.
cv::Mat map_current, map_current_copy, map_data; 
std::vector<Geographic_point> ref_border;

// VARIABLE FOR DATA.
std::vector<Data_node> node_vector;
std::vector<Data_road> road_vector;

void mouseHandler(int event, int x, int y, int flags, void* param)
{
    // [?] Si besoin.
}

int main()
{
    Read_YAML_file(&redis, "../data/HMD.yaml", &ref_border);
    Read_TXT_file(get_redis_str(&redis, "SIM_HMD_TXT_PATH"), node_vector, road_vector);
    Read_JPG_file(get_redis_str(&redis, "SIM_MAP_JPG_PATH"), map_current);
    map_current_copy = map_current.clone();
    Init_data_map(map_current, map_data);
    Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, false);

    // Thread run.
    thread_rendering    = std::thread(&f_rendering);
    thread_keyboard     = std::thread(&f_keyboard);
    thread_sim          = std::thread(&f_sim);

    thread_rendering.join();
    thread_keyboard.join();
    thread_sim.join();
}

void f_rendering()
{
    while(true)
    {
        double ms_for_loop = frequency_to_ms(10);
        auto next = std::chrono::high_resolution_clock::now();

        int mouseParam = cv::EVENT_FLAG_LBUTTON;
        cv::namedWindow( "HIVE MAP EDITOR", 4);
        cv::setMouseCallback("HIVE MAP EDITOR",mouseHandler, NULL);

        while(true)
        {
            next += std::chrono::milliseconds((int)ms_for_loop);
            std::this_thread::sleep_until(next);

            map_current_copy = map_current.clone();
            Init_data_map(map_current, map_data);
            Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, false); 

            cv::imshow("HIVE MAP EDITOR", map_current_copy);
            char d =(char)cv::waitKey(25);
        }
    }
}

void f_keyboard()
{
    while(true)
    {
        
    }
}

void f_sim()
{
    while(true)
    {
        
    }
}