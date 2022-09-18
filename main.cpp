#include <iostream>
#include <vector>
#include "iomanip"

#include "dualQuaternion.h"

struct indices{
    int32_t index0;
    int32_t index1;
    int32_t index2;
};

indices checkVector(const std::vector<glm::vec3>& x);
dualQuaternion<float> getTranformation(const std::vector<glm::vec3>& x, const std::vector<glm::vec3>& x_);

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


    std::cout<<std::endl;
    std::cout<<"=========================================="<<std::endl;

    std::vector<glm::vec3> points(3);
    std::vector<glm::vec3> points_(3);

    points[0] = {1.0f, 2.0f, 3.0f};
    points[1] = {4.0f, 3.0f, 5.0f};
    points[2] = {1.0f,-2.0f,-3.0f};

    dualQuaternion<float> Q = convert(convert(glm::radians(45.0f),glm::normalize(glm::vec3(1.0f,1.0f,1.0f))),{0.0f,2.0f,-2.0f,1.0f});

    for(size_t i=0;i<points.size();i++)
    {
        dualQuaternion<float> pos({1.0f,0.0f,0.0f,0.0f},{0.0f,0.5f*points[i].x,0.5f*points[i].y,0.5f*points[i].z});
        dualQuaternion<float> pos_ = Q*pos*conjugate(Q);
        points_[i] = pos_.translation().vector();
    }

    std::cout<<"originally given transformation = "<<Q<<std::endl;
    std::cout<<"reconstructed transformation    = "<<getTranformation(points,points_)<<std::endl;

    return 0;
}

indices checkVector(const std::vector<glm::vec3>& x)
{
    indices id = {-1,-1,-1};
    if(x.size()>0){
        id.index0 = 0;

        uint32_t i = id.index0;
        while(i<x.size() && x[id.index0] == x[i]){
            i++;
        }

        if(i<x.size()){
            id.index1 = i;

            while(i<x.size() &&
                  (x[i].x-x[0].x)/(x[id.index1].x-x[0].x) == (x[i].y-x[0].y)/(x[id.index1].y-x[0].y) &&
                  (x[i].y-x[0].y)/(x[id.index1].y-x[0].y) == (x[i].z-x[0].z)/(x[id.index1].z-x[0].z)
            ){
                i++;
            }

            if(i<x.size()){
                id.index2 = i;
            }
        }
    }

    return id;
}

dualQuaternion<float> getTranformation(const std::vector<glm::vec3>& x, const std::vector<glm::vec3>& x_)
{
    dualQuaternion<float> Q;

    indices ind = checkVector(x);

    if(ind.index0!=-1&&ind.index1!=-1&&ind.index2!=-1)
    {

        glm::vec3 x1 = x[ind.index0];        glm::vec3 x1_ = x_[ind.index0];
        glm::vec3 x2 = x[ind.index1];        glm::vec3 x2_ = x_[ind.index1];
        glm::vec3 x3 = x[ind.index2];        glm::vec3 x3_ = x_[ind.index2];

        glm::vec3 t = x1 - x1_;

        glm::vec3 u = x2 - x1;      glm::vec3 u_ = x2_ - x1_;

        float cosTheta = glm::dot(u,u_)/(glm::length(u)*glm::length(u_));
        float theta = glm::acos(cosTheta);

        x1_ += t;
        x2_ += t;
        x3_ += t;

        float det = glm::determinant(glm::mat3x3( x1.x, x1.y, x1.z, x2.x, x2.y, x2.z, x2_.x, x2_.y, x2_.z));
        float a   = glm::determinant(glm::mat3x3(-1.0f, x1.y, x1.z,-1.0f, x2.y, x2.z, -1.0f, x2_.y, x2_.z));
        float b   = glm::determinant(glm::mat3x3( x1.x,-1.0f, x1.z, x2.x,-1.0f, x2.z, x2_.x, -1.0f, x2_.z));
        float c   = glm::determinant(glm::mat3x3( x1.x, x1.y,-1.0f, x2.x, x2.y,-1.0f, x2_.x, x2_.y, -1.0f));

        a /= det;
        b /= det;
        c /= det;

        float invNorma = 1.0f/glm::sqrt(a*a+b*b+c*c);
        glm::vec3 n = {invNorma*a,invNorma*b,invNorma*c};

                   x1_ -= x1;
        x2 -= x1;  x2_ -= x1;
        x3 -= x1;  x3_ -= x1;
        x1 = {0.0f,0.0f,0.0f};

        quaternion<float> q1 = convert(theta,n);

        x1_ = quaternion<float>( q1 * quaternion<float>(0.0f,x1_) * conjugate(q1) ).vector();
        x2_ = quaternion<float>( q1 * quaternion<float>(0.0f,x2_) * conjugate(q1) ).vector();
        x3_ = quaternion<float>( q1 * quaternion<float>(0.0f,x3_) * conjugate(q1) ).vector();

                  u = x2;                u_ = x2_;
        glm::vec3 v = x3;      glm::vec3 v_ = x3_;

        float cosPsi = glm::dot(u,v)/glm::length(u)/glm::length(v);
        glm::vec3 l = cosPsi * glm::length(v) * glm::normalize(u);
        glm::vec3 m  = normalize(v - l);
        glm::vec3 m_ = normalize(v_ - l);

        float cosPhi = glm::dot(m,m_);
        float phi = -glm::acos(cosPhi);

        quaternion<float> q2 = convert(phi,normalize(u));

        quaternion<float> q = q1.invert()*q2.invert();

        glm::vec3 xrot = quaternion<float>( q * quaternion<float>(0.0f,x[0]) * conjugate(q) ).vector();
        glm::vec3 tr = (x_[0] - xrot)/2.0f;

        Q = convert(q,{0.0f,tr});
    }

    return Q;
}
