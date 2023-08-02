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
    Interface::sub_state_sub_t                  submarine_state_sub_;


    void resetDeadReckoning();
    void getA50Data();
    int connectToSocket(int sock, struct sockaddr_in& serv_addr);
    std::vector<float> parseJson(nlohmann::json& json_dict);
    void publishData(std::vector<float>& a50_data);
};

#endif // A50_H
