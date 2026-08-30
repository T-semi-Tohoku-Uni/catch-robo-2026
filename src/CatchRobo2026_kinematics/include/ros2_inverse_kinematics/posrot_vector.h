//
// Created by yuta on 2023/09/03.
//

#ifndef CATCHROBO2023_POSROT_VECTOR_H
#define CATCHROBO2023_POSROT_VECTOR_H

#define X 0
#define Y 1
#define Z 2
#define PHI 3
#define THE 4
#define PSI 5
/*
     * The unit of length is [mm], and the unit of angle is [rad].
     * Field origin is set on the top surface of the center of the plate_connector
     * The origin of hand coordinate is set at the point that hand rotation axis and the bottom surface of the endfactor_adapter cross.
     * Positive of field Y axle is set toward opponent.
     * Positive of field X axle is set toward right facing opponent.
     */

struct posrot_vec{
    float x;
    float y;
    float z;

    //euler angle;
    float phi;
    float thi;
    float psi;
};

#endif //CATCHROBO2023_POSROT_VECTOR_H
