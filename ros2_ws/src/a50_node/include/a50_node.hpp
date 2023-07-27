/* 
 * @author Zix
 * A50 sensor is the love of my life and gives a lot of information
 * That data can be grabbed from a web server (that is on the DVL) using tcp/ip (sockets baby)
 * If you type nc -v 192.168.1.4 16171 in the command line you can see the data (assuming your ip has the same subnet mask)
 * Here we will take that information, parse the json for what we want, and regurgitate it out
 * Of course in C++ because I'm tired of trash languages in this code base 
 */

/* NOTE: If it's not working on a new environment you may need to install this library: sudo apt-get install nlohmann-json3-dev */

#ifndef A50_H
#define A50_H

#include <memory>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <nlohmann/json.hpp>

#include "control_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vector_operations.hpp"

class A50Node : public rclcpp::Node
{
public:
    explicit A50Node();

private:
    const char* TCP_IP = "192.168.1.4";  
    const int   TCP_PORT = 16171;           
    static const int   BUFFER_SIZE = 4096;
    int sock_ = 0;
    struct sockaddr_in serv_addr_;
    char buffer_[BUFFER_SIZE] = {0};
    Interface::state_pub_t position_publisher_;
    Interface::state_pub_t orientation_publisher_;
    Interface::ros_timer_t get_data_timer_;

    void resetDeadReckoning();
    void getA50Data();
    int connectToSocket(int sock, struct sockaddr_in& serv_addr);
    std::vector<float> parseJson(nlohmann::json& json_dict);
    void publishData(std::vector<float>& a50_data);
};

#endif // A50_H
