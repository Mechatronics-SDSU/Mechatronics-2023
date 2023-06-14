#include "pid_controller.hpp"
#include <memory>
//#include <iostream>

using namespace std;

/* 
 1. look at parameters -> make fakes ones
 2. Mock all you passed 3 and the answer was 0.788531
You passed 3 and the answer was 0.788580
You passed 3 and the answer was 0.788580

 3. assert something is true
 */

/* bool assert(bool condition)
{
    return 1;
}
*/
int main()

{

    PID_Controller* controller = new PID_Controller();
    controller->k_p = 0.015;
    controller->k_i = 0.003;
    controller->k_d = 0.001;
    controller->i_min = 1;
    controller->i_max = 1;
    controller->ctrl_val_min = -1;
    controller->ctrl_val_max = 1.00000000000;
    controller->ctrl_val_offset = 0.0;
    controller->angle_wrap = false;
    controller->integral = 0.0;
    controller->previous_error = 0.0;

    for (int i = 0; i < 100; i++)
    {
        printf("You passed %d and the answer was %f\n", i, controller->update(i, 1/20));
    }

    // printf("You passed %d and the answer was %f\n", n, controller->update(3, .005346666));
    
    return 0;
}