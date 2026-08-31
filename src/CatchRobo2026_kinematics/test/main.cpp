#include <iostream>

#include "./include/posrot_vector.h"
#include "./include/robot_kinematics.h"


int main() {
    float trg_pos[6] = {980, 225,111,0,0,0};
    float joint_angle[4];

    robot_kinematics robot;


    robot.inverse_kinematics(trg_pos, joint_angle);

    std::cout <<"0:"<< joint_angle[0] <<"\n1:"<<joint_angle[1] <<"\n2:"<<joint_angle[2] << std::endl;
    return 0;
}
