#include "useful.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

std::thread thread_rendering, thread_keyboard, thread_sim;

// VARIABLE FOR RENDERING.
cv::Mat map_current, map_current_copy, map_data; 
std::vector<Geographic_point> ref_border;

// SIM
sim_robot masimulation = sim_robot(2.12740,48.89721,90.0);

// VARIABLE FOR DATA.
std::vector<Data_node> node_vector;
std::vector<Data_road> road_vector;

void mouseHandler(int event, int x, int y, int flags, void* param)
{
    // [?] Si besoin.
}

int main()
{
    std::cout << std::setprecision(10);

    Read_YAML_file(&redis, "../data/HMD.yaml", &ref_border);
    Read_TXT_file(get_redis_str(&redis, "SIM_HMD_TXT_PATH"), node_vector, road_vector);
    Read_JPG_file(get_redis_str(&redis, "SIM_MAP_JPG_PATH"), map_current);
    map_current_copy = map_current.clone();
    Init_data_map(map_current, map_data);
    Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, false);

    // Thread run.
    thread_sim          = std::thread(&f_sim);
    thread_rendering    = std::thread(&f_rendering);
    // thread_keyboard     = std::thread(&f_keyboard);

    thread_sim.join();
    thread_rendering.join();
    // thread_keyboard.join();
}

void f_rendering()
{
    while(true)
    {
        double ms_for_loop = frequency_to_ms(5);
        auto next = std::chrono::high_resolution_clock::now();

        int mouseParam = cv::EVENT_FLAG_LBUTTON;
        cv::namedWindow( "HIVE MAP EDITOR", 4);
        cv::setMouseCallback("HIVE MAP EDITOR",mouseHandler, NULL);

        while(true)
        {
            next += std::chrono::milliseconds((int)ms_for_loop);
            std::this_thread::sleep_until(next);

            // DRAW MAP.
            map_current_copy = map_current.clone();
            Init_data_map(map_current, map_data);
            Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, false); 

            // DRAW ALL GLOBAL PATH.
            std::vector<std::string> vect_str_redis;
            get_redis_multi_str(&redis, "SIM_GLOBAL_PATH", vect_str_redis);

            Geographic_point pointA = Geographic_point(0.0,0.0);
            Geographic_point pointB = Geographic_point(0.0,0.0);

            for(std::string road_curr : vect_str_redis)
            {
                for(auto road : road_vector)
                {
                    if(std::stoi(road_curr) == road.road_ID)
                    {
                        pointA = road.A->point;
                        pointB = road.B->point;
                        project_multi_geo_element(ref_border, map_current_copy, 2, &pointA, &pointB);
                        break;
                    }
                }
            }

            // DRAW CURRENT MAP.
            get_redis_multi_str(&redis, "NAV_ROAD_CURRENT_ID", vect_str_redis);

            for(auto road : road_vector)
            {
                if(road.road_ID == std::stoi(vect_str_redis[1]))
                {
                    pointA = road.A->point;
                    pointB = road.B->point;
                }
            }
            project_multi_geo_element(ref_border, map_current_copy, 1, &pointA, &pointB);


            // DRAW POINT FUTUR.
            // get_redis_multi_str(&redis, "SIM_AUTO_PT_FUTUR", vect_str_redis);
            // Geographic_point pt_futur = Geographic_point(std::stod(vect_str_redis[0]), std::stod(vect_str_redis[1]));
            // project_geo_element(ref_border, map_current_copy, 2, &pt_futur, 1.0);

            // DRAW POINT PROJECTED FUTUR.
            get_redis_multi_str(&redis, "SIM_AUTO_PROJECT_PT_FUTUR", vect_str_redis);
            Geographic_point pt_pfutur = Geographic_point(std::stod(vect_str_redis[0]), std::stod(vect_str_redis[1]));
            project_geo_element(ref_border, map_current_copy, 2, &pt_pfutur, 2.0);

            // DRAW POINT TARGET.
            get_redis_multi_str(&redis, "SIM_AUTO_PT_TARGET", vect_str_redis);
            Geographic_point pt_target = Geographic_point(std::stod(vect_str_redis[0]), std::stod(vect_str_redis[1]));
            project_geo_element(ref_border, map_current_copy, 2, &pt_target, 3.0);


            // DRAW DESTINATION PROJECTED POINT.
            get_redis_multi_str(&redis, "NAV_AUTO_PROJECT_DESTINATION", vect_str_redis);
            if(std::stoul(vect_str_redis[0]) != 0)
            {
                Geographic_point pt_destination_project = Geographic_point(std::stod(vect_str_redis[1]), std::stod(vect_str_redis[2]));
                project_geo_element(ref_border, map_current_copy, 2, &pt_destination_project, 4.0);
                
                // DRAW DESTINATION POINT.
                get_redis_multi_str(&redis, "NAV_AUTO_DESTINATION", vect_str_redis);
                Geographic_point pt_destination = Geographic_point(std::stod(vect_str_redis[1]), std::stod(vect_str_redis[2]));
                project_geo_element(ref_border, map_current_copy, 2, &pt_destination, 1.0);
            }

            // UNCOMMANT FOR VISUALISATION.
            // get_redis_multi_str(&redis, "NAV_GLOBAL_POSITION", vect_str_redis);
            // Geographic_point real_robot = Geographic_point(std::stod(vect_str_redis[1]), std::stod(vect_str_redis[2]));
            // project_geo_element(ref_border, map_current_copy, 1, &real_robot, std::stod(vect_str_redis[3]));

            // UNCOMMANT FOR SIMULATION
            // DRAW ROBOT 
            Geographic_point robot = Geographic_point(masimulation.point->longitude, masimulation.point->latitude);
            project_geo_element(ref_border, map_current_copy, 1, &robot, masimulation.hdg);

            // DRAW ROBOT WITH ERROR.
            get_redis_multi_str(&redis, "NAV_GLOBAL_POSITION", vect_str_redis);
            Geographic_point robot_err = Geographic_point(std::stod(vect_str_redis[1]), std::stod(vect_str_redis[2]));
            project_geo_element(ref_border, map_current_copy, 5, &robot_err, std::stod(vect_str_redis[3]));

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
    std::cout << std::setprecision(10);

    // [!] STARTING SIMULATION PUSH NEW POSITION ON REDIS VAR.
    std::string debug_str = "";
    debug_str = std::to_string(get_curr_timestamp()) + "|";
    debug_str += get_redis_str(&redis, "SIM_START_LONGITUDE") + "|";
    debug_str += get_redis_str(&redis, "SIM_START_LATITUDE") + "|";
    debug_str += get_redis_str(&redis, "SIM_START_HDG") + "|";
    set_redis_var(&redis, "NAV_GLOBAL_POSITION", debug_str);

    double ms_for_loop = frequency_to_ms(20);
    auto next = std::chrono::high_resolution_clock::now();

    std::default_random_engine generator;
    std::normal_distribution<double> distribution_angle(0.0,720.0);

    std::default_random_engine generator2;
    std::normal_distribution<double> distribution_dist(0.0,std::stod(get_redis_str(&redis, "SIM_GPS_POS_ERR_M")));

    std::default_random_engine generator3;
    std::normal_distribution<double> distribution_new_hdg(0.0,std::stod(get_redis_str(&redis, "SIM_GPS_HDG_ERR_M")));

    while(true)
    {
        std::normal_distribution<double> distribution_dist(0.0,std::stod(get_redis_str(&redis, "SIM_GPS_POS_ERR_M")));
        std::normal_distribution<double> distribution_new_hdg(0.0,std::stod(get_redis_str(&redis, "SIM_GPS_HDG_ERR_M")));

        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);


        // 1. CHECK LES HARD_MOTOR_COMMAND
        std::vector<std::string> vect_redis_str;
        get_redis_multi_str(&redis, "HARD_MOTOR_COMMAND", vect_redis_str);

        // std::cout << "FRAME SIMULATION." << vect_redis_str[0] << std::endl;
        
        if(std::stoul(vect_redis_str[0]) != 0)
        {
            double speed_motor_l = std::stod(vect_redis_str[2]);
            double speed_motor_r = std::stod(vect_redis_str[5]);

            // 2. Calculer le changement d'orientation et de distance.
            long double bearing  = (speed_motor_r - speed_motor_l) * (ms_for_loop / 1000) / std::stod(get_redis_str(&redis, "HARD_WHEEL_DISTANCE"));
            long double distance = (speed_motor_r + speed_motor_l) * (ms_for_loop / 1000) / 2;

            // 3. COMPUTE NEW GPS.
            // masimulation.update(&redis);

            long double new_hdg = rad_to_deg(deg_to_rad(masimulation.hdg) - bearing);

            Geographic_point new_position = get_new_position(masimulation.point, new_hdg, distance);

            // 4. UPDATE ALL.
            // debug_str = std::to_string(get_curr_timestamp()) + "|";
            // debug_str += std::to_string(new_position.longitude) + "|";
            // debug_str += std::to_string(new_position.latitude) + "|";
            // debug_str += std::to_string(new_hdg) + "|";


            if(new_position.latitude > 0 && new_position.longitude > 0)
            {
                // NORMAL NO ERROR.
                // set_redis_var(&redis, "NAV_GLOBAL_POSITION", debug_str);
                masimulation.point->longitude = new_position.longitude;
                masimulation.point->latitude  = new_position.latitude;
                masimulation.hdg              = new_hdg;

                // WITH ERROR.
                double angle_new_pos, dist_new_pose, new_hdg2;
                angle_new_pos = distribution_angle(generator);
                dist_new_pose = distribution_dist(generator2);
                new_hdg2 = new_hdg + distribution_new_hdg(generator3);
                if(new_hdg2 > 360) new_hdg2 -= 360;
                if(new_hdg2 < 0) new_hdg2 += 360;

                Geographic_point pos_with_error = get_new_position(masimulation.point, angle_new_pos, dist_new_pose);
                debug_str = std::to_string(get_curr_timestamp()) + "|";
                debug_str += std::to_string(pos_with_error.longitude) + "|";
                debug_str += std::to_string(pos_with_error.latitude) + "|";
                debug_str += std::to_string(new_hdg2) + "|";
                set_redis_var(&redis, "NAV_GLOBAL_POSITION", debug_str);
            }

        }
    }
}