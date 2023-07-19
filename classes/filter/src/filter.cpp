/*  
 * @author Zix using Joseph's algorithm
 * Run test output by typing these commands into terminal in the classes/filter directory
        - cmake CMakeList.txt
        - make
        - ./filter
    
    This class can be used as a general filter that can filter any data stream and even a multitude of data streams (think multiple points)
    This object can be constructed in any file that needs to use it.
 */ 

#include "filter.hpp"
#include "vector_operations.hpp"
#include <fstream>
#include <iostream>
#include <cstring>
#include <memory>

using namespace std;


/* Constructor
 * @param targets: pass number of points you want to filter, this way one filter can track multiple points (4 corners of bounding box for example)
 * @param coeff_file: pass name of file that holds coefficients specific to system in question 
 *
 * *targets* number of static deques will be allocated with size being number of coefficients in coeff file
 */
Filter::Filter(size_t targets, std::string& coeff_file)
{
    read_coeffs_from_file(this->coefficients, coeff_file);
    for (size_t i = 0; i < targets; i++) 
    {
        deque<float> data_stream(this->coefficients.size(), 0);
        this->data_streams.push_back(data_stream);
    }
}

/* 
 * Specify the specific target that the filtering belongs to for history, do this by passing index of target into member data_streams
 * Pass in unfiltered data point, recieve filtered data point (this will work once buffer is filled and history gathered, note this will create some delay)
 */
float Filter::smooth(deque<float>& static_deque, float in_x)
{
    float tmp_x = 0;
    static_deque.pop_back();
    static_deque.push_front(in_x);
    for (size_t i = 0; i < static_deque.size(); ++i) {
        tmp_x += (this->coefficients[i] * static_deque[i]);
    }
    return tmp_x;
}

/* 
 * Private function to populate coeffs from the passed in file
 */
void Filter::read_coeffs_from_file(std::vector<float>& coeffs, const std::string& filename)
{
    std::ifstream coeffs_file(filename);
    if (!coeffs_file) {
        std::cout << "Failed to open coefficients file." << std::endl;
        return;
    }
    float element;
    while (coeffs_file >> element) 
    {
        coeffs.push_back(element);
    }
}

void Filter::print_coefficients()
{
    for (float coeff : this->coefficients)
    {
        cout << coeff << endl;
    }
}


// @Test - Tracks two different points, fills buffer and then prints input and output data stream with filtering
// int main()
// {
//     size_t data_streams = 2;
//     string coeff_file = "coefficients.txt";

//     unique_ptr<Filter> moving_average_filter = make_unique<Filter>(data_streams, coeff_file);
//     moving_average_filter->print_coefficients();

//     vector<float> fill_buffer_data {10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
//     vector<float> test_sensor_data {11, 12, 13, 12, 9, 7, 4, 5, 8, 14, 11, 12, 10, 12, 8, 7, 2, 7, 7, 8, 9, 14, 10, 15};
//     vector<float> test_sensor_data2 {7, 2, 7, 7, 8, 9, 14, 10, 15, 9, 7, 4, 5, 8, 14, 11, 12, 10, 1, 11, 12, 13, 12, 9};
//     vector<float> test_output_data;
//     vector<float> test_output_data2;

//     for (float input : fill_buffer_data)
//     {
//         moving_average_filter->smooth(moving_average_filter->data_streams[0], input);
//         moving_average_filter->smooth(moving_average_filter->data_streams[1], input);
//     }
//     for (float input : test_sensor_data) 
//     {
//         test_output_data.push_back(moving_average_filter->smooth(moving_average_filter->data_streams[0], input));
//     }
//     for (float input : test_sensor_data2) 
//     {
//         test_output_data2.push_back(moving_average_filter->smooth(moving_average_filter->data_streams[1], input));
//     }

//     cout << endl << "Input Data 1: ";
//     printVector(test_sensor_data);
//     cout << "Output Data 1: ";
//     printVector(test_output_data);

//     cout << endl << "Input Data 2: ";
//     printVector(test_sensor_data2);
//     cout << "Output Data 2: ";
//     printVector(test_output_data2);

// }
