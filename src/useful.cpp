#include "useful.h"
#include <fstream>

void Read_YAML_file(sw::redis::Redis* redis, std::string path, std::vector<Geographic_point>* ref_border)
{
    cv::FileStorage fsSettings(path, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Can't open the file " << path << std::endl;
        return;
    }

    std::string information;

    read_yaml(redis, &fsSettings, "p1_longitude");
    read_yaml(redis, &fsSettings, "p1_latitude");
    read_yaml(redis, &fsSettings, "p2_longitude");
    read_yaml(redis, &fsSettings, "p2_latitude");
    Geographic_point p1(std::stod(get_redis_str(redis, "p1_longitude")),std::stod(get_redis_str(redis, "p1_latitude")));
    Geographic_point p2(std::stod(get_redis_str(redis, "p2_longitude")),std::stod(get_redis_str(redis, "p2_latitude")));

    ref_border->push_back(p1);
    ref_border->push_back(p2);

    read_yaml(redis, &fsSettings, "SIM_MAP_JPG_PATH");
    read_yaml(redis, &fsSettings, "SIM_HMD_TXT_PATH");
    read_yaml(redis, &fsSettings, "SIM_START_LONGITUDE");
    read_yaml(redis, &fsSettings, "SIM_START_LATITUDE");
    read_yaml(redis, &fsSettings, "SIM_START_HDG");
    read_yaml(redis, &fsSettings, "SIM_GLOBAL_PATH");

    read_yaml(redis, &fsSettings, "SIM_AUTO_PT_FUTUR");
    read_yaml(redis, &fsSettings, "SIM_AUTO_PROJECT_PT_FUTUR");
    read_yaml(redis, &fsSettings, "SIM_AUTO_PT_TARGET");
    read_yaml(redis, &fsSettings, "SIM_AUTO_PT_ICC");
    read_yaml(redis, &fsSettings, "SIM_AUTO_RADIUS_ICC");
    read_yaml(redis, &fsSettings, "SIM_AUTO_PROJECT_PT_DESTINATION");

    read_yaml(redis, &fsSettings, "SIM_GPS_POS_ERR_M");
    read_yaml(redis, &fsSettings, "SIM_GPS_HDG_ERR_M");

    read_yaml(redis, &fsSettings, "SIM_AUTO_PT_ICC_NEW");
    read_yaml(redis, &fsSettings, "SIM_AUTO_RADIUS_ICC_NEW");

    read_yaml(redis, &fsSettings, "SIM_GPS_LX_ERR_M");
    read_yaml(redis, &fsSettings, "SIM_GPS_LY_ERR_M");
    read_yaml(redis, &fsSettings, "SIM_GPS_LHDG_ERR_M");
    read_yaml(redis, &fsSettings, "SIM_SENSOR_ERR_M");
}

int64_t get_curr_timestamp()
{
    int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>((std::chrono::high_resolution_clock::now()).time_since_epoch()).count();

    return timestamp;
}

std::string get_event_str(int ID_event, std::string event_description, std::string event_info)
{
    return std::to_string(get_curr_timestamp()) + "|" + std::to_string(ID_event) + "|" + event_description + "|" + event_info + "|";
}

void set_redis_var(sw::redis::Redis* redis, std::string channel, std::string value)
{
    redis->set(channel, value);
}

void pub_redis_var(sw::redis::Redis* redis, std::string channel, std::string value)
{
    redis->publish(channel, value);
}

std::string get_redis_str(sw::redis::Redis* redis, std::string channel)
{
    return *(redis->get(channel));
}

int get_redis_multi_str(sw::redis::Redis* redis, std::string channel, std::vector<std::string>& stockage)
{
    stockage.clear();
    
    std::string multi_str = *(redis->get(channel));

    std::string T;
    std::stringstream X(multi_str);

    int number_of_data = 0;

    while(std::getline(X, T, '|'))
    {
        stockage.push_back(T);
        number_of_data++;
    } 

    return number_of_data;
}

int get_multi_str(std::string str, std::vector<std::string>& vec_str)
{
    vec_str.clear();

    std::string T;
    std::stringstream X(str);

    int number_of_data = 0;

    while(std::getline(X, T, '|'))
    {
        vec_str.push_back(T);
        number_of_data++;
    } 

    return number_of_data;
}

void read_yaml(sw::redis::Redis* redis, cv::FileStorage* file_mng, std::string channel)
{
    std::string read_data;
    (*file_mng)[channel] >> read_data;
    redis->set(channel, read_data);
}

double frequency_to_ms(int frequency)
{
    return 1000 / frequency;
}

bool time_is_over(int64_t curr_timestamp, int64_t ref_timestamp, int64_t max_duration_ms)
{
    // std::cout << curr_timestamp << " " << ref_timestamp << " " << curr_timestamp-ref_timestamp << std::endl;
    // std::string temp;
    // std::cin >> temp;
    if(curr_timestamp - ref_timestamp > max_duration_ms) return true;
    return false;
}

void print_redis(sw::redis::Redis* redis, std::string channel_str)
{
    int max_size = 30;
    int size_channel_title = channel_str.length();

    std::string format_channel_str = "";
    format_channel_str += channel_str;

    for(int i = size_channel_title; i < max_size; i++) format_channel_str += " ";
    format_channel_str += " = ";

    format_channel_str += get_redis_str(redis, channel_str);
    std::cout << format_channel_str << std::endl;
}

std::string get_standard_robot_id_str(sw::redis::Redis* redis)
{
    std::string official_id_str = get_redis_str(redis, "ROBOT_INFO_ID") + "-";
    official_id_str += get_redis_str(redis, "ROBOT_INFO_PSEUDO") + "-";
    official_id_str += get_redis_str(redis, "ROBOT_INFO_MODEL") + "-";
    official_id_str += get_redis_str(redis, "ROBOT_INFO_EXPLOITATION");
    return official_id_str;
}

bool compare_redis_var(sw::redis::Redis* redis, std::string channel, std::string compare)
{
    if(get_redis_str(redis, channel).compare(compare) == 0) return true;
    return false;
}

void Read_TXT_file(std::string path, std::vector<Data_node>& vector_node, std::vector<Data_road>& road_vector)
{ 
    vector_node.clear();
    road_vector.clear();

    std::ifstream file(path);
    std::string str; 

    std::string data_type;

    while (std::getline(file, str))
    {
        std::vector<std::string> vect_str;

        get_multi_str(str, vect_str);
        if(vect_str.size() == 1) data_type = vect_str[0];
        else
        {
            if(data_type.compare("NODE") == 0)
            {
                Data_node new_data(std::stoi(vect_str[0]), std::stod(vect_str[1]), std::stod(vect_str[2]));
                vector_node.push_back(new_data);
            }

            if(data_type.compare("ROAD") == 0)
            {
                Data_node* tempo_save_A;
                Data_node* tempo_save_B;
                for(int i = 0; i < vector_node.size(); i++)
                {
                    if(vector_node[i].node_ID == std::stoi(vect_str[1]))
                    {
                        tempo_save_A = &vector_node[i];
                    }
                    if(vector_node[i].node_ID == std::stoi(vect_str[2]))
                    {
                        tempo_save_B = &vector_node[i];
                    }
                }
                Data_road new_road(std::stoi(vect_str[0]), tempo_save_A, tempo_save_B);

                if(std::stoi(vect_str[6]) == 1) new_road.available = true;
                else{new_road.available = false;}

                new_road.deg_to_A = std::stod(vect_str[3]);
                new_road.deg_to_B = std::stod(vect_str[4]);
                new_road.length = std::stod(vect_str[5]);
                new_road.max_speed = std::stod(vect_str[7]);
                road_vector.push_back(new_road);
            }
        }
    }
}

void Read_JPG_file(std::string path, cv::Mat& img)
{
    img = imread(path, cv::IMREAD_COLOR);
    if(img.empty())
    {
        std::cout << "Could not read the image: " << path << std::endl;
    }
}

void Init_data_map(cv::Mat& map_current, cv::Mat& map_data)
{
    cv::Mat new_map_data(map_current.rows, map_current.cols, CV_16UC1, cv::Scalar(0));
    map_data = new_map_data;
}

void Project_all_element(std::vector<Geographic_point>& ref_border, std::vector<Data_node>& node_vector, cv::Mat& map_current, cv::Mat& map_data, std::vector<Data_road>& road_vector, bool speed_view)
{    
    // Draw road.
    for(auto& road : road_vector)
    { 
        if(speed_view)
        {
            if(road.max_speed < 6) cv::line(map_current, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(180, 180, 180), 10, cv::LINE_8);
            if(road.max_speed > 6 && road.max_speed < 8) cv::line(map_current, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(102, 102, 255), 10, cv::LINE_8);
            if(road.max_speed > 8) cv::line(map_current, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(51, 51, 255), 10, cv::LINE_8);
        }
        if(road.available)
        {
            cv::line(map_current, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(182,242,176), 4, cv::LINE_8);
        }
        else
        {
            cv::line(map_current, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(255, 0, 127), 4, cv::LINE_8);
        }
        cv::line(map_data   , cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(road.road_ID), 4, cv::LINE_8);
    }

    // Draw node.
    for(auto& node : node_vector)
    {   
        node.col_idx = ((node.point.longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
        node.row_idx = (double)(map_current.rows) - (((node.point.latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));
        
        cv::circle(map_current, cv::Point((int)(node.col_idx),(int)(node.row_idx)),5,      cv::Scalar(0,0,250), cv::FILLED, 1,0);
        cv::circle(map_data,    cv::Point((int)(node.col_idx),(int)(node.row_idx)),5, cv::Scalar(node.node_ID), cv::FILLED, 1,0);
    }
}

void project_geo_element(std::vector<Geographic_point>& ref_border, cv::Mat& map_current, int element_type, Geographic_point* position, double hdg)
{
    double col_idx, row_idx, col_idx2, row_idx2;

    if(element_type == 1)
    {
        // Classic robot drawing.
        col_idx = ((position->longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
        row_idx = (double)(map_current.rows) - (((position->latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));
        cv::circle(map_current, cv::Point((int)(col_idx),(int)(row_idx)),4, cv::Scalar(0,222,255), cv::FILLED, 1,0);

        Geographic_point orientation_robot = get_new_position(position, hdg, 1);

        col_idx2 = ((orientation_robot.longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
        row_idx2 = (double)(map_current.rows) - (((orientation_robot.latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));

        // std::cout << "PIXEL DIST:" << sqrt(pow(col_idx-col_idx2,2)+pow(row_idx-row_idx2,2)) << std::endl;
        // std::cout << "METER DIST:" << get_angular_distance(position, &orientation_robot) << std::endl;

        cv::line(map_current, cv::Point((int)(col_idx),(int)(row_idx)), cv::Point((int)(col_idx2),(int)(row_idx2)), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
    }
    if(element_type == 2)
    {
        // Classic point.
        // Pour les points hdg devient une couleur.
        col_idx = ((position->longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
        row_idx = (double)(map_current.rows) - (((position->latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));
        
        if(hdg == 1.0) cv::circle(map_current, cv::Point((int)(col_idx),(int)(row_idx)),4, cv::Scalar(0,255,0), cv::FILLED, 1,0);
        if(hdg == 2.0) cv::circle(map_current, cv::Point((int)(col_idx),(int)(row_idx)),4, cv::Scalar(255,200,150), cv::FILLED, 1,0);
        if(hdg == 3.0) cv::circle(map_current, cv::Point((int)(col_idx),(int)(row_idx)),4, cv::Scalar(255,0,127), cv::FILLED, 1,0);
        if(hdg == 4.0) cv::circle(map_current, cv::Point((int)(col_idx),(int)(row_idx)),5, cv::Scalar(255,255,128), cv::FILLED, 1,0);
    }
    if(element_type == 3)
    {
        // Un cercle.
        // Pour les cercles hdg devient le rayon.
        col_idx = ((position->longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
        row_idx = (double)(map_current.rows) - (((position->latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));

        cv::circle(map_current, cv::Point((int)(col_idx),(int)(row_idx)), (int)(hdg), cv::Scalar(178, 102, 255), 1, cv::LineTypes::LINE_8);
    }
    if(element_type == 4)
    {
        // Un cercle orange
        // Pour les cercles hdg devient le rayon.
        col_idx = ((position->longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
        row_idx = (double)(map_current.rows) - (((position->latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));

        cv::circle(map_current, cv::Point((int)(col_idx),(int)(row_idx)), (int)(hdg), cv::Scalar(102, 178, 255), 1, cv::LineTypes::LINE_8);
    }
    if(element_type == 5)
    {
        // ERROR GPS VISUALISATION.
        col_idx = ((position->longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
        row_idx = (double)(map_current.rows) - (((position->latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));
        cv::circle(map_current, cv::Point((int)(col_idx),(int)(row_idx)),3, cv::Scalar(255,255,51), cv::FILLED, 1,0);

        Geographic_point orientation_robot = get_new_position(position, hdg, 1);

        col_idx2 = ((orientation_robot.longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
        row_idx2 = (double)(map_current.rows) - (((orientation_robot.latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));

        // std::cout << "PIXEL DIST:" << sqrt(pow(col_idx-col_idx2,2)+pow(row_idx-row_idx2,2)) << std::endl;
        // std::cout << "METER DIST:" << get_angular_distance(position, &orientation_robot) << std::endl;

        cv::line(map_current, cv::Point((int)(col_idx),(int)(row_idx)), cv::Point((int)(col_idx2),(int)(row_idx2)), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
    }
}

void project_multi_geo_element(std::vector<Geographic_point>& ref_border, cv::Mat& map_current, int element_type, Geographic_point* positionA, Geographic_point* positionB)
{
    double col_idx, row_idx, col_idx2, row_idx2;

    if(element_type == 1)
    {
        // CURRENT ROAD.
        col_idx  = ((positionA->longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
        row_idx  = (double)(map_current.rows) - (((positionA->latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));
        col_idx2 = ((positionB->longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
        row_idx2 = (double)(map_current.rows) - (((positionB->latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));
    
        cv::line(map_current, cv::Point((int)(col_idx),(int)(row_idx)), cv::Point((int)(col_idx2),(int)(row_idx2)), cv::Scalar(0,255,255), 7, cv::LINE_8);

        cv::circle(map_current, cv::Point((int)(col_idx),(int)(row_idx)),5, cv::Scalar(0,0,255), cv::FILLED, 1,0);
        cv::circle(map_current, cv::Point((int)(col_idx2),(int)(row_idx2)),5, cv::Scalar(0,0,255), cv::FILLED, 1,0);
    }

    if(element_type == 2)
    {
        // CURRENT ROAD.
        col_idx  = ((positionA->longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
        row_idx  = (double)(map_current.rows) - (((positionA->latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));
        col_idx2 = ((positionB->longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
        row_idx2 = (double)(map_current.rows) - (((positionB->latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));
    
        cv::line(map_current, cv::Point((int)(col_idx),(int)(row_idx)), cv::Point((int)(col_idx2),(int)(row_idx2)), cv::Scalar(105,0,0), 7, cv::LINE_8);

        cv::circle(map_current, cv::Point((int)(col_idx),(int)(row_idx)),5, cv::Scalar(0,0,255), cv::FILLED, 1,0);
        cv::circle(map_current, cv::Point((int)(col_idx2),(int)(row_idx2)),5, cv::Scalar(0,0,255), cv::FILLED, 1,0);
    }
}

Geographic_point get_new_position(Geographic_point* start_position, double bearing, double distance)
{
    // [!] VRAI FORMULE.
    // http://www.movable-type.co.uk/scripts/latlong.html?from=48.7819900,-122.2936380&to=48.7761100,-122.3395200
    // double ang_distance = distance;///6371000;
    // double start_lat_deg = deg_to_rad(start_position->latitude);
    // double start_lon_deg = deg_to_rad(start_position->longitude);
    // double latitude  = asin(sin(start_lat_deg) * cos(ang_distance) + cos(start_lat_deg) * sin(ang_distance) * cos(bearing));
    // double longitude = start_lon_deg + atan2(sin(bearing)*sin(ang_distance)*cos(start_lat_deg), cos(ang_distance) - sin(start_lat_deg) * sin(latitude));
    
    // [!] Approche correct pour la carte mais pas robuste car diff long lat depend de l'endroit sur terre?
    // https://www.youtube.com/watch?v=IVz4f36xwUs


    long double departure = distance*0.00001*0.9 * sin(deg_to_rad(bearing));
    long double latitude  = distance*0.00001*0.9 * cos(deg_to_rad(bearing));

    // std::cout << "DEP:" << departure << " LAT:" << latitude << std::endl;

    long double lon_f = start_position->longitude + (departure*1.52);
    long double lat_f = start_position->latitude + latitude;
    
    Geographic_point orientation_robot_position = Geographic_point(lon_f, lat_f);
    return orientation_robot_position;
}

long double deg_to_rad(const long double degree)
{
    long double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}

double get_angular_distance(Geographic_point* pointA, Geographic_point* pointB)
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

    // [?] c represente la distance angulaire.
    // return c

    return R * c;
}

double rad_to_deg(double rad)
{
    double deg = rad * 180 / M_PI;
    if(deg > 360) deg = deg - 360;
    if(deg < 0)   deg = deg + 360;
    return deg;
}

position_pxl get_pixel_pos(std::vector<Geographic_point>& ref_border, cv::Mat& map_current, Geographic_point* position)
{
    double col_idx, row_idx;
    col_idx = ((position->longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
    row_idx = (double)(map_current.rows) - (((position->latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));
    return position_pxl(col_idx, row_idx);
}

void update_sensor_prm(sw::redis::Redis* redis, std::vector<Sensor_prm>& vect_sensor_prm)
{
    Sensor_prm cam1 = Sensor_prm(std::stod(get_redis_str(redis, "HARD_CAM1_DX")), std::stod(get_redis_str(redis, "HARD_CAM1_DY")), 0 , std::stod(get_redis_str(redis, "HARD_CAM1_ANGLE")));
    Sensor_prm cam2 = Sensor_prm(std::stod(get_redis_str(redis, "HARD_CAM2_DX")), std::stod(get_redis_str(redis, "HARD_CAM2_DY")), 1 , std::stod(get_redis_str(redis, "HARD_CAM2_ANGLE")));
    Sensor_prm lid1 = Sensor_prm(std::stod(get_redis_str(redis, "HARD_LID1_DX")), std::stod(get_redis_str(redis, "HARD_LID1_DY")), 10, std::stod(get_redis_str(redis, "HARD_LID1_ANGLE")));
    Sensor_prm lid2 = Sensor_prm(std::stod(get_redis_str(redis, "HARD_LID2_DX")), std::stod(get_redis_str(redis, "HARD_LID2_DY")), 11, std::stod(get_redis_str(redis, "HARD_LID2_ANGLE")));
    vect_sensor_prm.push_back(cam1);
    vect_sensor_prm.push_back(cam2);
    vect_sensor_prm.push_back(lid1);
    vect_sensor_prm.push_back(lid2);
}