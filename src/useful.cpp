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
    size_t size = 10;

    Geographic_point p1(0,0);
    fsSettings["p1_longitude"] >> information;
    p1.longitude = std::stod(information, &size);
    fsSettings["p1_latitude"] >> information;
    p1.latitude = std::stod(information);

    Geographic_point p2(0,0);
    fsSettings["p2_longitude"] >> information;
    p2.longitude = std::stod(information);
    fsSettings["p2_latitude"] >> information;
    p2.latitude = std::stod(information);

    ref_border->push_back(p1);
    ref_border->push_back(p2);

    read_yaml(redis, &fsSettings, "SIM_MAP_JPG_PATH");
    read_yaml(redis, &fsSettings, "SIM_HMD_TXT_PATH");
    read_yaml(redis, &fsSettings, "SIM_START_LONGITUDE");
    read_yaml(redis, &fsSettings, "SIM_START_LATITUDE");
    read_yaml(redis, &fsSettings, "SIM_START_HDG");
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