#include "useful.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

std::thread thread_rendering, thread_keyboard, thread_sim;

// VARIABLE FOR RENDERING.
cv::Mat map_current, map_current_copy, map_data; 
std::vector<Geographic_point> ref_border;

// SIM
sim_robot masimulation = sim_robot(2.0024775, 49.0464248, 90.0);
bool zoom_flag = true;
std::vector<fake_obj> obj_vector;
std::vector<Sensor_prm> vect_sensor_prm;

// VARIABLE FOR DATA.
std::vector<Data_node> node_vector;
std::vector<Data_road> road_vector;

////////////////////////////////////////////////////////////////////////////////
double pix_per_m = 5.0;

// SIZE OF ZOOM MAP
double size_zoom = 15.0;
////////////////////////////////////////////////////////////////////////////////


void mouseHandler(int event, int x, int y, int flags, void* param)
{
    // [?] Si besoin.
    switch(event){
    case cv::EVENT_LBUTTONDOWN:
        fake_obj n_obj = fake_obj(x, y);
        obj_vector.push_back(n_obj);
    }
}

/**
 * NOTE: Fonctionnement du soft.
 * 
 * 1 - SIMULATION : Lancer tout les threads + mettre à true dans function zoom.
 * 2 - VISUALISATION : Lancer tout les threads sauf le thread de simulation + mettre à true dans function zoom.
 */

int main()
{
    std::cout << std::setprecision(10);

    Read_YAML_file(&redis, "../data/COURDIMANCHE.yaml", &ref_border);
    Read_TXT_file(get_redis_str(&redis, "SIM_HMD_TXT_PATH"), node_vector, road_vector);
    Read_JPG_file(get_redis_str(&redis, "SIM_MAP_JPG_PATH"), map_current);
    update_sensor_prm(&redis, vect_sensor_prm);
    map_current_copy = map_current.clone();
    Init_data_map(map_current, map_data);
    Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, false);

    set_redis_var(&redis, "ROBOT_MODE", "MANUAL");
    set_redis_var(&redis, "MISSION_MOTOR_BRAKE", "TRUE");

    // Thread run.
    // thread_sim          = std::thread(&f_sim);
    thread_rendering    = std::thread(&f_rendering);
    thread_keyboard     = std::thread(&f_keyboard);

    // thread_sim.join();
    thread_rendering.join();
    thread_keyboard.join();
}

void f_rendering()
{
    /*
        TODO: get how many pixel for 10m.
    */

    ////////////////////////////////////////////////////////////////////////////////
    Geographic_point tempo1_curr = Geographic_point(masimulation.point->longitude, masimulation.point->latitude);
    position_pxl tempo1_curr_pxl = get_pixel_pos(ref_border, map_current_copy, &tempo1_curr);
    // std::cout << tempo1_curr_pxl.idx_i << " " << tempo1_curr_pxl.idx_j << std::endl;
    Geographic_point tempo2_curr = get_new_position(&tempo1_curr, 0, 100);
    position_pxl tempo2_curr_pxl = get_pixel_pos(ref_border, map_current_copy, &tempo2_curr);
    // std::cout << tempo2_curr_pxl.idx_i << " " << tempo2_curr_pxl.idx_j << std::endl;
    pix_per_m = sqrt(pow(tempo1_curr_pxl.idx_row - tempo2_curr_pxl.idx_row, 2)) / 100;
    int pixel_box = (int)(pix_per_m * size_zoom);


    cv::Mat zoom;


    while(true)
    {
        double ms_for_loop = frequency_to_ms(10);
        auto next = std::chrono::high_resolution_clock::now();

        int mouseParam = cv::EVENT_FLAG_LBUTTON;
        cv::namedWindow( "HIVE_SIMULATION", 4);
        cv::setMouseCallback("HIVE_SIMULATION",mouseHandler, NULL);

        cv::namedWindow( "ZOOM_HIVE_SIMULATION", 4);

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

            // DRAW CIRCLE POINT.
            get_redis_multi_str(&redis, "SIM_AUTO_PT_ICC", vect_str_redis);
            Geographic_point pt_circle = Geographic_point(std::stod(vect_str_redis[0]), std::stod(vect_str_redis[1]));
            int radius = std::stoi(get_redis_str(&redis, "SIM_AUTO_RADIUS_ICC"));
            project_geo_element(ref_border, map_current_copy, 3, &pt_circle, (double)(radius*pix_per_m));

            // DRAW NEW CIRCLE POINT.
            get_redis_multi_str(&redis, "SIM_AUTO_PT_ICC_NEW", vect_str_redis);
            Geographic_point pt_circle2 = Geographic_point(std::stod(vect_str_redis[0]), std::stod(vect_str_redis[1]));
            int radius2 = std::stoi(get_redis_str(&redis, "SIM_AUTO_RADIUS_ICC_NEW"));
            project_geo_element(ref_border, map_current_copy, 4, &pt_circle2, (double)(radius2*pix_per_m));

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
            get_redis_multi_str(&redis, "NAV_GLOBAL_POSITION", vect_str_redis);
            Geographic_point real_robot = Geographic_point(std::stod(vect_str_redis[1]), std::stod(vect_str_redis[2]));
            project_geo_element(ref_border, map_current_copy, 1, &real_robot, std::stod(vect_str_redis[3]));

            // UNCOMMANT FOR SIMULATION
            // DRAW ROBOT 
            // Geographic_point robot = Geographic_point(masimulation.point->longitude, masimulation.point->latitude);
            // project_geo_element(ref_border, map_current_copy, 1, &robot, masimulation.hdg);

            // DRAW ROBOT WITH ERROR.
            get_redis_multi_str(&redis, "NAV_GLOBAL_POSITION", vect_str_redis);
            Geographic_point robot_err = Geographic_point(std::stod(vect_str_redis[1]), std::stod(vect_str_redis[2]));
            project_geo_element(ref_border, map_current_copy, 5, &robot_err, std::stod(vect_str_redis[3]));

            // DRAW OBSTACLE
            for(auto obj : obj_vector)
            {
                cv::circle(map_current_copy, cv::Point((int)(obj.pxl->idx_col),(int)(obj.pxl->idx_row)), (int)(0), cv::Scalar(0, 102, 204), 1, cv::LineTypes::LINE_8);
            }

            int direct_map_pt_number = 0;
            if(zoom_flag)
            {
                position_pxl tempo_curr_pxl = position_pxl(0.0, 0.0);

                // POUR LA SIMULATION
                if(false)
                {
                    Geographic_point tempo_curr = Geographic_point(masimulation.point->longitude, masimulation.point->latitude);
                    tempo_curr_pxl = get_pixel_pos(ref_border, map_current_copy, &tempo_curr);
                }
                // POUR LA VISUALISATION
                if(true)
                {
                    Geographic_point tempo_curr = Geographic_point(robot_err.longitude, robot_err.latitude);
                    tempo_curr_pxl = get_pixel_pos(ref_border, map_current_copy, &tempo_curr);
                }


                map_current_copy(cv::Rect(tempo_curr_pxl.idx_col-pixel_box, tempo_curr_pxl.idx_row-pixel_box, pixel_box*2, pixel_box*2)).copyTo(zoom);
                // map_current_copy(cv::Rect(0, 0, 200, 200)).copyTo(zoom);
                
                // IF ERROR COMMENT DRAW POINT.
                // DRAW DIRECT MAP
                // std::vector<std::string> direct_map_vect;
                // get_redis_multi_str(&redis, "SIM_DIRECT_MAP", direct_map_vect);
                // for(int i = 0; i < direct_map_vect.size(); i+=2)
                // {
                //     double idx_col, idx_row;

                //     double dist  = sqrt(pow(masimulation.lpoint->longitude-std::stod(direct_map_vect[i]),2)+pow(masimulation.lpoint->latitude-std::stod(direct_map_vect[i+1]),2)) * pix_per_m;
                //     double angle = 2 * atan((std::stod(direct_map_vect[i+1]) - masimulation.lpoint->latitude) / ((std::stod(direct_map_vect[i]) - masimulation.lpoint->longitude)+sqrt(pow(masimulation.lpoint->longitude-std::stod(direct_map_vect[i]),2)+pow(masimulation.lpoint->latitude-std::stod(direct_map_vect[i+1]),2))));
                //     // std::cout << "SIM ANGLE : " << angle << std::endl;
                //     idx_col = pixel_box + dist * cos(angle);
                //     idx_row = pixel_box + dist * sin(angle);
                //     cv::circle(zoom, cv::Point((int)(idx_col),(int)(idx_row)),1, cv::Scalar(0,0,255), 1, cv::LineTypes::LINE_8);

                //     direct_map_pt_number++;
                // }


                cv::imshow("ZOOM_HIVE_SIMULATION", zoom);
            }

            // DRAW TEXT
            cv::Scalar curr_color;
            if(direct_map_pt_number < 200) curr_color = cv::Scalar(0,255,0);
            if(direct_map_pt_number >= 200 && direct_map_pt_number <= 1000) curr_color = cv::Scalar(102,255,255);
            if(direct_map_pt_number > 1000) curr_color = cv::Scalar(0,0,255);
            
            cv::putText(map_current_copy, //target image
            "Nb obstacle : ", //text
            cv::Point(0, 50), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            1,
            cv::Scalar(255,255,255), //font color
            2);

            cv::putText(map_current_copy, //target image
            std::to_string(direct_map_pt_number), //text
            cv::Point(250, 50), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            1,
            curr_color, //font color
            2);
        

            // DRAW ALL
            cv::imshow("HIVE_SIMULATION", map_current_copy);
            char d =(char)cv::waitKey(25);
        }
    }
}

void f_keyboard()
{
    std::string str_input;

    while(true)
    {
        std::cin >> str_input;
        if(str_input.compare("ZOOM") == 0)
        {
            zoom_flag = !zoom_flag;
            // std::cout << "ZOOM_FLAG:" << zoom_flag << std::endl;
        }
        if(str_input.compare("CLEAR") == 0)
        {
            obj_vector.clear();
        }
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

    std::default_random_engine generator_lx;
    std::normal_distribution<double> distribution_lx(0.0,std::stod(get_redis_str(&redis, "SIM_GPS_LX_ERR_M")));

    std::default_random_engine generator_ly;
    std::normal_distribution<double> distribution_ly(0.0,std::stod(get_redis_str(&redis, "SIM_GPS_LY_ERR_M")));

    std::default_random_engine generator_lhdg;
    std::normal_distribution<double> distribution_lhdg(0.0,std::stod(get_redis_str(&redis, "SIM_GPS_LHDG_ERR_M")));

    std::default_random_engine generator_sensor;
    std::normal_distribution<double> distribution_sensor(0.0,std::stod(get_redis_str(&redis, "SIM_SENSOR_ERR_M")));

    std::vector<std::string> previous_vect_state_mcu_cargo;
    previous_vect_state_mcu_cargo.push_back("000000000000");
    previous_vect_state_mcu_cargo.push_back("CLOSE");
    previous_vect_state_mcu_cargo.push_back("CLOSE");
    previous_vect_state_mcu_cargo.push_back("CLOSE");
    set_redis_var(&redis, "MISSION_HARD_CARGO", "0000000000|CLOSE|CLOSE|CLOSE|");
    uint64_t timestamp_open = get_curr_timestamp();
    bool open_timestamp_on = false;

    while(true)
    {
        // PUT ERROR IN YOUR LIFE
        std::normal_distribution<double> distribution_dist(0.0,std::stod(get_redis_str(&redis, "SIM_GPS_POS_ERR_M")));
        std::normal_distribution<double> distribution_new_hdg(0.0,std::stod(get_redis_str(&redis, "SIM_GPS_HDG_ERR_M")));
        std::normal_distribution<double> distribution_lx(0.0,std::stod(get_redis_str(&redis, "SIM_GPS_LX_ERR_M")));
        std::normal_distribution<double> distribution_ly(0.0,std::stod(get_redis_str(&redis, "SIM_GPS_LY_ERR_M")));
        std::normal_distribution<double> distribution_lhdg(0.0,std::stod(get_redis_str(&redis, "SIM_GPS_LHDG_ERR_M")));
        std::normal_distribution<double> distribution_sensor(0.0,std::stod(get_redis_str(&redis, "SIM_SENSOR_ERR_M")));
        // END OF ERROR.

        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);


        // 1. CHECK LES HARD_MOTOR_COMMAND
        std::vector<std::string> vect_redis_str;
        get_redis_multi_str(&redis, "HARD_MOTOR_COMMAND", vect_redis_str);

        // std::cout << "FRAME SIMULATION." << vect_redis_str[0] << std::endl;
        
        // if(std::stoul(vect_redis_str[0]) != 0)
        // {
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

                // WITHOUT ERROR.
                // Geographic_point pos_with_error = get_new_position(masimulation.point, angle_new_pos, dist_new_pose);
                // debug_str = std::to_string(get_curr_timestamp()) + "|";
                // debug_str += std::to_string(masimulation.point->longitude) + "|";
                // debug_str += std::to_string(masimulation.point->latitude) + "|";
                // debug_str += std::to_string(new_hdg) + "|";
                // set_redis_var(&redis, "NAV_GLOBAL_POSITION", debug_str);

                Geographic_point pos_with_error = get_new_position(masimulation.point, angle_new_pos, dist_new_pose);
                debug_str = std::to_string(get_curr_timestamp()) + "|";
                debug_str += std::to_string(pos_with_error.longitude) + "|";
                debug_str += std::to_string(pos_with_error.latitude) + "|";
                debug_str += std::to_string(new_hdg2) + "|";
                set_redis_var(&redis, "NAV_GLOBAL_POSITION", debug_str);    

                // UPDATE POSITION OF ALL SENSOR AND COMPUTE FAKE OBJ.
                // CAM1
                Geographic_point pos_cam1 = get_new_position(masimulation.point, masimulation.hdg + vect_sensor_prm[0].pos_pol->y, vect_sensor_prm[0].pos_pol->x);
                position_pxl pos_cam1_pxl = get_pixel_pos(ref_border, map_current_copy, &pos_cam1);

                std::string cam1_str = std::to_string(get_curr_timestamp()) + "|";
                for(auto obj : obj_vector)
                {
                    double dist = sqrt(pow(obj.pxl->idx_col-pos_cam1_pxl.idx_col,2)+pow(obj.pxl->idx_row-pos_cam1_pxl.idx_row,2)) * (1/pix_per_m);
                    if(dist < 10.0)
                    {
                        double tempo    = (obj.pxl->idx_col-pos_cam1_pxl.idx_col)/((obj.pxl->idx_row-pos_cam1_pxl.idx_row)+sqrt(pow(obj.pxl->idx_col-pos_cam1_pxl.idx_col,2)+pow(obj.pxl->idx_row-pos_cam1_pxl.idx_row,2)));
                        double ang_diff = 360 -(rad_to_deg(2 * atan(tempo)) + 180);
                        double ang_sens = masimulation.hdg + vect_sensor_prm[0].hdg;
                        
                        double angle;
                        if(ang_sens - ang_diff > 0)
                        {
                            if(ang_sens - ang_diff > 180) angle = 360 - (ang_sens - ang_diff);
                            else{angle = -(ang_sens - ang_diff);}
                        }
                        else
                        {
                            if(ang_sens - ang_diff < -180) angle = -(360 - (ang_diff - ang_sens));
                            else{ angle = ang_diff - ang_sens;}
                        }
                        // std::cout << "NEW MESURE " << ang_sens << " ANGLE DIFF " << ang_diff << "FINAL " << angle << std::endl;

                        // WITH ERROR
                        dist += distribution_sensor(generator_sensor);

                        if(abs(angle) < 90)
                        {cam1_str += "o|" + std::to_string(dist) + "|" + std::to_string(-angle) + "|";}
                    }
                }
                set_redis_var(&redis, "ENV_CAM1_OBJECTS", cam1_str);
                // END CAM1
                // CAM2
                Geographic_point pos_cam2 = get_new_position(masimulation.point, masimulation.hdg + vect_sensor_prm[1].pos_pol->y, vect_sensor_prm[1].pos_pol->x);
                position_pxl pos_cam2_pxl = get_pixel_pos(ref_border, map_current_copy, &pos_cam1);

                std::string cam2_str = std::to_string(get_curr_timestamp()) + "|";
                for(auto obj : obj_vector)
                {
                    double dist = sqrt(pow(obj.pxl->idx_col-pos_cam2_pxl.idx_col,2)+pow(obj.pxl->idx_row-pos_cam2_pxl.idx_row,2)) * (1/pix_per_m);
                    if(dist < 10.0)
                    {
                        double tempo    = (obj.pxl->idx_col-pos_cam2_pxl.idx_col)/((obj.pxl->idx_row-pos_cam2_pxl.idx_row)+sqrt(pow(obj.pxl->idx_col-pos_cam1_pxl.idx_col,2)+pow(obj.pxl->idx_row-pos_cam1_pxl.idx_row,2)));
                        double ang_diff = 360 -(rad_to_deg(2 * atan(tempo)) + 180);
                        double ang_sens = masimulation.hdg + vect_sensor_prm[1].hdg;
                        
                        double angle;
                        if(ang_sens - ang_diff > 0)
                        {
                            if(ang_sens - ang_diff > 180) angle = 360 - (ang_sens - ang_diff);
                            else{angle = -(ang_sens - ang_diff);}
                        }
                        else
                        {
                            if(ang_sens - ang_diff < -180) angle = -(360 - (ang_diff - ang_sens));
                            else{ angle = ang_diff - ang_sens;}
                        }
                        // std::cout << "NEW MESURE " << ang_sens << " ANGLE DIFF " << ang_diff << "FINAL " << angle << std::endl;

                        // WITH ERROR
                        dist += distribution_sensor(generator_sensor);

                        if(abs(angle) < 90)
                        { cam2_str += "o|" + std::to_string(dist) + "|" + std::to_string(-angle) + "|";}
                    }
                }
                set_redis_var(&redis, "ENV_CAM2_OBJECTS", cam2_str);
                // END CAM2

                // UPDATE LOCAL. (lon=x, lat=y) WITHOUT ERROR
                // masimulation.lpoint->longitude = masimulation.lpoint->longitude + distance * cos(deg_to_rad(masimulation.lhdg) - bearing);
                // masimulation.lpoint->latitude  = masimulation.lpoint->latitude  + distance * sin(deg_to_rad(masimulation.lhdg) - bearing);
                // masimulation.lhdg              = rad_to_deg(deg_to_rad(masimulation.lhdg) - bearing);
                // debug_str = std::to_string(get_curr_timestamp()) + "|";
                // debug_str += std::to_string(masimulation.lpoint->longitude) + "|";
                // debug_str += std::to_string(masimulation.lpoint->latitude) + "|";
                // debug_str += std::to_string(masimulation.lhdg) + "|";
                // set_redis_var(&redis, "NAV_LOCAL_POSITION", debug_str);

                // UPDATE LOCAL. (lon=x, lat=y) UN MAX D'ERROR
                masimulation.lpoint->longitude = masimulation.lpoint->longitude + distance * cos(deg_to_rad(masimulation.lhdg) - bearing) + distribution_lx(generator_lx)/20;
                masimulation.lpoint->latitude  = masimulation.lpoint->latitude  + distance * sin(deg_to_rad(masimulation.lhdg) - bearing) + distribution_ly(generator_ly)/20;
                masimulation.lhdg              = rad_to_deg(deg_to_rad(masimulation.lhdg) - bearing + distribution_lhdg(generator_lhdg)/20);
                debug_str = std::to_string(get_curr_timestamp()) + "|";
                debug_str += std::to_string(masimulation.lpoint->longitude) + "|";
                debug_str += std::to_string(masimulation.lpoint->latitude) + "|";
                debug_str += std::to_string(masimulation.lhdg) + "|";
                set_redis_var(&redis, "NAV_LOCAL_POSITION", debug_str);

                // SIM BOX OPEN OR CLOSING.
                std::vector<std::string> new_vect_state_mcu_cargo;
                get_redis_multi_str(&redis, "MISSION_HARD_CARGO", new_vect_state_mcu_cargo);

                for(int i = 1; i < new_vect_state_mcu_cargo.size(); i++)
                {
                    if(new_vect_state_mcu_cargo[i].compare(previous_vect_state_mcu_cargo[i]) != 0)
                    {
                        if(new_vect_state_mcu_cargo[i].compare("OPEN") == 0)
                        {
                            timestamp_open = get_curr_timestamp();
                            open_timestamp_on = true;
                            if(i == 1) pub_redis_var(&redis, "EVENT", get_event_str(66, "BOX_OPEN", "1"));
                            if(i == 2) pub_redis_var(&redis, "EVENT", get_event_str(66, "BOX_OPEN", "2"));
                            if(i == 3) pub_redis_var(&redis, "EVENT", get_event_str(66, "BOX_OPEN", "3"));
                        }
                        if(new_vect_state_mcu_cargo[i].compare("CLOSE") == 0)
                        {
                            if(i == 1) pub_redis_var(&redis, "EVENT", get_event_str(66, "BOX_CLOSE", "1"));
                            if(i == 2) pub_redis_var(&redis, "EVENT", get_event_str(66, "BOX_CLOSE", "2"));
                            if(i == 3) pub_redis_var(&redis, "EVENT", get_event_str(66, "BOX_CLOSE", "3"));
                        }
                    }

                    previous_vect_state_mcu_cargo[i] = new_vect_state_mcu_cargo[i];
                }

                // std::cout << open_timestamp_on << std::endl;

                // SIM CLOSING BOX
                if(open_timestamp_on && time_is_over(get_curr_timestamp(), timestamp_open, 5000))
                {
                    open_timestamp_on = false;
                    set_redis_var(&redis, "MISSION_HARD_CARGO", std::to_string(get_curr_timestamp()) + "|CLOSE|CLOSE|CLOSE|");
                }
            }

        // }
    }
}