#include <iostream>
#include "iomanip"
#include "dualQuaternion.h"

int main()
{
    quaternion<float> q(glm::sqrt(2.0f)/2.0f,glm::vec3(0.5f,0.0f,0.5f));
    quaternion<float> p(0.0f,2.0f,0.0f,0.0f);

    std::cout<<"\t\t"<<std::setw(10)<<"s\t"<<"x\t"<<"y\t"<<"z\t"<<std::endl;
    std::cout<<"p \t= \t"<<std::setw(10)<<p<<std::endl;
    std::cout<<"q \t= \t"<<std::setw(10)<<q<<std::endl;

    std::cout<<std::endl;
    std::cout<<"Rotation with quaternion: q*p*q_inv"<<std::endl;
    std::cout<<"p' \t= \t"<<std::setw(10)<<q*p*invert(q)<<std::endl;

    glm::mat3x3 R = convert(q);
    glm::vec3 p1(p.vector());
    glm::vec3 p2 = glm::vec3(   R[0][0]*p1[0]+R[0][1]*p1[1]+R[0][2]*p1[2],
                                R[1][0]*p1[0]+R[1][1]*p1[1]+R[1][2]*p1[2],
                                R[2][0]*p1[0]+R[2][1]*p1[1]+R[2][2]*p1[2]);

    std::cout<<std::endl;
    std::cout<<"Rotation with O3 matrix from quaternion: q->R"<<std::endl;
    std::cout<<"R*p \t= \t\t"<<std::setw(10)<<p2.x<<'\t'<<p2.y<<'\t'<<p2.z<<std::endl;

    quaternion<float> q1 = convert(R);

    std::cout<<std::endl;
    std::cout<<"Rotation with quaternion from O3 matrix: q->R"<<std::endl;
    std::cout<<"R->q \t= \t"<<std::setw(10)<<q1<<std::endl;
    std::cout<<"p' \t= \t"<<std::setw(10)<<q1*p*invert(q1)<<std::endl;

    std::cout<<std::endl;
    std::cout<<"Convert from quaternion to Euler Angles"<<std::endl;
    glm::vec3 EAngles = convertToEulerAngles(q);
    std::cout<<"E Ang \t= \t"<<EAngles.x<<'\t'<<EAngles.y<<'\t'<<EAngles.z<<std::endl;

    std::cout<<std::endl;
    std::cout<<"Convert from Euler Angles to quaternion"<<std::endl;
    quaternion<float> q2 = convert(EAngles.x,EAngles.y,EAngles.z);
    std::cout<<"q \t= \t"<<std::setw(10)<<q2<<std::endl;

    std::cout<<std::endl;
    std::cout<<"Convert from Angle and axis to quaternion"<<std::endl;
    quaternion<float> q3 = convert(glm::radians(90.0f),glm::vec3(glm::sqrt(2.0f)/2.0f,0.0f,glm::sqrt(2.0f)/2.0f));
    std::cout<<"q \t= \t"<<std::setw(10)<<q3<<std::endl;

    std::cout<<std::endl;
    std::cout<<"Convert from quaternion to Angle and axis"<<std::endl;
    quaternion<float> q4 = convertToAnglesAndAxis(q3);
    std::cout<<"q \t= \t"<<std::setw(10)<<q4<<std::endl;

    std::cout<<std::endl;
    std::cout<<"dual quat"<<std::endl;
    glm::vec4 pos(5.0f,0.0f,5.0f,1.0f);

    dualQuaternion<float> P1 = convert(
        convert(glm::radians(90.0f),glm::vec3(0.0f,0.0f,1.0f)),
        quaternion<float>(0.0f,5.0f,0.0f,0.0f));
    dualQuaternion<float> P2= convert(
        quaternion<float>(1.0f,0.0f,0.0f,0.0f),
        quaternion<float>(0.0f,pos.x,pos.y,pos.z));

    std::cout<<"dual quat1 \t\t = "<<P1<<std::endl;
    std::cout<<"position (dual quat2) \t = "<<P2<<std::endl;

    std::cout<<"transformation = "<<P1*P2*conjugate(P1)<<std::endl;

    glm::mat4x4 SE3 = convert(P1);

    glm::vec4 newPos = glm::vec4(   SE3[0][0]*pos[0]+SE3[0][1]*pos[1]+SE3[0][2]*pos[2]+SE3[0][3]*pos[3],
                                    SE3[1][0]*pos[0]+SE3[1][1]*pos[1]+SE3[1][2]*pos[2]+SE3[1][3]*pos[3],
                                    SE3[2][0]*pos[0]+SE3[2][1]*pos[1]+SE3[2][2]*pos[2]+SE3[2][3]*pos[3],
                                    SE3[3][0]*pos[0]+SE3[3][1]*pos[1]+SE3[3][2]*pos[2]+SE3[3][3]*pos[3]);

    std::cout<<"transformation by SE3 from quat1 "<<std::endl;
    std::cout<<newPos.x<<'\t'<<newPos.y<<'\t'<<newPos.z<<'\t'<<std::endl;

    dualQuaternion<float> P3 = convert(SE3);

    std::cout<<"inverse transformation from SE3 to quat1= "<<std::endl;
    std::cout<<P3<<std::endl;

    return 0;
}
