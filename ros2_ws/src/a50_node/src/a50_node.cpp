/* 
 * @author Zix
 * A50 sensor is the love of my life and gives a lot of information
 * That data can be grabbed from a web server (that is on the DVL) using tcp/ip (sockets baby)
 * If you type nc -v 192.168.1.4 16171 in the command line you can see the data (assuming your ip has the same subnet mask)
 * Here we will take that information, parse the json for what we want, and regurgitate it out
 * Of course in C++ because I'm tired of trash languages in this code base 
 */

/* NOTE: If it's not working on a new environment you may need to install this library: sudo apt-get install nlohmann-json3-dev */


#include "a50_node.hpp"
#include <chrono>
#include <iostream>

using namespace std;
using json = nlohmann::json;

A50Node::A50Node(): Node("a50_node")
{
    position_publisher_ = this->create_publisher<scion_types::msg::State>("a50_state_data", 10);
    orientation_publisher_ = this->create_publisher<scion_types::msg::State>("ahrs_orientation_data", 10);
    this->resetDeadReckoning();
    sock_ = connectToSocket(sock_, serv_addr_);
    get_data_timer_ = this->create_wall_timer(50ms, std::bind(&A50Node::getA50Data, this));
    submarine_state_sub_ = this->create_subscription<scion_types::msg::SubState>("submarine_state", 10, [this](const scion_types::msg::SubState::SharedPtr msg)
    {
        if (msg->host_mode == 0) 
        {
            exit(EXIT_SUCCESS);
        }
    });
}

// Send reset command to the socket
void A50Node::resetDeadReckoning() 
{
    int sock = 0;
    struct sockaddr_in serv_addr;

    sock = connectToSocket(sock, serv_addr);
    string json_command = R"({"command" : "reset_dead_reckoning"})";
    ssize_t bytesWrite = send(sock, json_command.c_str(), json_command.length(), 0);
    
    if (bytesWrite != static_cast<ssize_t>(json_command.length())) {
        cerr << "Failed to send data." << endl;
    }
    close(sock);
}

/* This is the loop that runs as long as the node is spun (no timer) */
void A50Node::getA50Data()
{
    try {
        ssize_t bytesRead = recv(this->sock_, this->buffer_, this->BUFFER_SIZE, 0);
        string json_stream(this->buffer_, bytesRead);
        json json_dict = json::parse(json_stream);
        vector<float> a50_data = parseJson(json_dict);
        if (!a50_data.empty()) {publishData(a50_data);}        //Sometimes it gives bad data so be careful
    }
    catch(...) {
        close(this->sock_);
        this->sock_ = connectToSocket(this->sock_, this->serv_addr_);
    }
}

/* Tries to connect to the address of the DVL to grab the data*/
int A50Node::connectToSocket(int sock, struct sockaddr_in& serv_addr) 
{
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        throw std::runtime_error("Socket creation error");
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(TCP_PORT);

    if (inet_pton(AF_INET, TCP_IP, &serv_addr.sin_addr) <= 0) {
    throw std::runtime_error("Invalid address/ Address not supported");
    }

    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    throw std::runtime_error("Connection Failed");
    }
    return sock;
}

/* Using  nlohmann library makes it easy to grab the fields we want from the json*/
vector<float> A50Node::parseJson(json& json_dict) 
{
    float x, y, z, yaw, pitch, roll;
    try {
        x = json_dict["x"];
        y = json_dict["y"];
        z = json_dict["z"];
        yaw = json_dict["yaw"];
        pitch = json_dict["pitch"];
        roll = json_dict["roll"];
    } catch (...) {
        return vector<float>{};
    }
    return vector<float>{yaw, pitch, roll, x, y, z};
}

void A50Node::publishData(vector<float>& a50_data) 
{
    auto orientation = scion_types::msg::State();
    orientation.state = {a50_data[0], a50_data[1], a50_data[2]};
    orientation_publisher_->publish(orientation);

    auto position = scion_types::msg::State();
    position.state = {a50_data[3], a50_data[4], a50_data[5]};
    position_publisher_->publish(position);

    RCLCPP_INFO(this->get_logger(), "Publishing State Data:\n yaw: %f\n pitch: %f\n roll: %f",
            a50_data[0], a50_data[1], a50_data[2]);
    RCLCPP_INFO(this->get_logger(), "Publishing State Data:\n x: %f\n y: %f\n z: %f",
            a50_data[3], a50_data[4], a50_data[5]);
}
