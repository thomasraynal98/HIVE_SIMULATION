#include "useful.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

std::thread thread_rendering, thread_keyboard, thread_sim;

// VARIABLE FOR RENDERING.
cv::Mat map_current, map_current_copy, map_data; 
std::vector<Geographic_point> ref_border;

// SIM
sim_robot masimulation = sim_robot(std::stod(get_redis_str(&redis, "SIM_START_LONGITUDE")), std::stod(get_redis_str(&redis, "SIM_START_LATITUDE")), std::stod(get_redis_str(&redis, "SIM_START_HDG")));

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
    thread_sim          = std::thread(&f_sim);
    thread_rendering    = std::thread(&f_rendering);
    thread_keyboard     = std::thread(&f_keyboard);

    thread_sim.join();
    thread_rendering.join();
    thread_keyboard.join();
}

void f_rendering()
{
    while(true)
    {
        double ms_for_loop = frequency_to_ms(10);
        auto next = std::chrono::high_resolution_clock::now();

        int mouseParam = cv::EVENT_FLAG_LBUTTON;
        cv::namedWindow( "HIVE SIM", 4);
        cv::setMouseCallback("HIVE SIM",mouseHandler, NULL);

        while(true)
        {
            next += std::chrono::milliseconds((int)ms_for_loop);
            std::this_thread::sleep_until(next);

            map_current_copy = map_current.clone();
            Init_data_map(map_current, map_data);
            Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, false); 
            Geographic_point robot = Geographic_point(masimulation.point->longitude, masimulation.point->latitude);
            project_geo_element(ref_border, map_current_copy, 1, &robot, masimulation.hdg);

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
    // [!] STARTING SIMULATION PUSH NEW POSITION ON REDIS VAR.
    std::string debug_str = "";
    debug_str = std::to_string(get_curr_timestamp()) + "|";
    debug_str += std::to_string(std::stod(get_redis_str(&redis, "SIM_START_LONGITUDE"))) + "|";
    debug_str += std::to_string(std::stod(get_redis_str(&redis, "SIM_START_LATITUDE"))) + "|";
    debug_str += std::to_string(std::stod(get_redis_str(&redis, "SIM_START_HDG"))) + "|";
    set_redis_var(&redis, "NAV_GLOBAL_POSITION", debug_str);

    double ms_for_loop = frequency_to_ms(20);
    auto next = std::chrono::high_resolution_clock::now();

    while(true)
    {
        std::cout << "FRAME SIM" << std::endl;
        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);

        // 1. CHECK LES HARD_MOTOR_COMMAND
        std::vector<std::string> vect_redis_str;
        get_redis_multi_str(&redis, "HARD_MOTOR_COMMAND", vect_redis_str);
        if(std::stoul(vect_redis_str[0]) != 0)
        {
            double speed_motor_l = std::stod(vect_redis_str[2]);
            double speed_motor_r = std::stod(vect_redis_str[5]);

            // 2. Calculer le changement d'orientation et de distance.
            double bearing  = (speed_motor_r - speed_motor_l) * (ms_for_loop / 1000) / std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"));
            double distance = (speed_motor_r + speed_motor_l) * (ms_for_loop / 1000) / 2;

            // 3. Compute new position GPS.
            masimulation.update(&redis);

            double new_hdg = rad_to_deg(deg_to_rad(masimulation.hdg) + bearing);

            Geographic_point new_position = get_new_position(masimulation.point, new_hdg, distance);

            // 4. UPDATE ALL.
            debug_str = std::to_string(get_curr_timestamp()) + "|";
            debug_str += std::to_string(new_position.longitude) + "|";
            debug_str += std::to_string(new_position.latitude) + "|";
            debug_str += std::to_string(new_hdg) + "|";
            set_redis_var(&redis, "NAV_GLOBAL_POSITION", debug_str);

            masimulation.point->longitude = new_position.longitude;
            masimulation.point->latitude  = new_position.latitude;
            masimulation.hdg              = new_hdg;
        }
    }
}