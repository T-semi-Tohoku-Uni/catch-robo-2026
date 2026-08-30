//
// Created by yuta on 2023/09/02.
//

#ifndef CATCHROBO2023_ROBOT_KINEMATICS_H
#define CATCHROBO2023_ROBOT_KINEMATICS_H

//#include <Eigen>
#include "posrot_vector.h"


#define PI 3.14159265

const float robot_pos[6] = {675, -130, 228, 0, 0, 0};



class robot_kinematics {
    /*
     * The unit of length is [mm], and the unit of angle is [rad].
     * Field origin is set at the front-left corner on the top surface of the field
     * Robot origin is set at the point where the orientation axis of th0 and th1 cross(675, -130, 228)
     * The origin of hand coordinate is set at the point that hand rotation axis and the bottom surface of the endfactor_adapter cross.
     * Positive of field Y axle is set toward opponent.
     * Positive of field X axle is set toward right facing opponent.
     * Joints and links are numbered from the base (0,1,2)
     * Origin of Joint0 is set toward positive field Y axle
     * Origins of Joint1 and 2 are set toward positive field Z axle
     *
     *  o_____________y
     *  |
     *  |
     *  |
     * R|
     *  |
     *  |
     *  |
     *  |_____________
     *  x
     *
     *
     *       /\
     *      /  \
     *     /    \ l1
     * l0 /      \
     *   /        \  l2
     *  /          \_____
     *                |
     *                |l3
     *
     */
private:

    float link_len[4];//mm
    //float joint_angle_now[4];
    //float joint_angle_trg[4]
    float joint_angle_lim[4][2];//rad, {inf, sup}


    //float r_posrot_trg[6];
    //float r_posrot_now[6];


    //float posrot_trg[6];
    //float posrot_now[6];
    /* posrot_vec posrot_trg;
     * posrot_vec posrot_now;
     */
    void convert_field2robot(float*, float*);


public:
    robot_kinematics();
    void inverse_kinematics(float*, float*);
    void forward_kinematics(float*, float*);





};


#endif //CATCHROBO2023_ROBOT_KINEMATICS_H
