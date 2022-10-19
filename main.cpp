#include <iostream>
#include <vector>
#include "iomanip"

#include "dualQuaternion.h"

struct indices{
    int32_t index0;
    int32_t index1;
    int32_t index2;
};

template<typename type> indices checkVector(const std::vector<glm::vec<3,type,glm::defaultp>>& x);
template<typename type> dualQuaternion<type> getTranformation(const std::vector<glm::vec<3,type,glm::defaultp>>& x, const std::vector<glm::vec<3,type,glm::defaultp>>& x_);

using Type = double;

int main()
{
    quaternion<Type> q(glm::sqrt(Type(2))/Type(2),glm::vec<3,Type,glm::defaultp>(Type(0.5),Type(0.0),Type(0.5)));
    quaternion<Type> p(Type(0),Type(2),Type(0),Type(0));

    std::cout<<"\t\t"<<std::setw(10)<<"s\t"<<"x\t"<<"y\t"<<"z\t"<<std::endl;
    std::cout<<"p \t= \t"<<std::setw(10)<<p<<std::endl;
    std::cout<<"q \t= \t"<<std::setw(10)<<q<<std::endl;

    std::cout<<std::endl;
    std::cout<<"Rotation with quaternion: q*p*q_inv"<<std::endl;
    std::cout<<"p' \t= \t"<<std::setw(10)<<q*p*invert(q)<<std::endl;

    glm::mat<3,3,Type,glm::defaultp> R = convert(q);
    glm::vec<3,Type,glm::defaultp> p1(p.vector());
    glm::vec<3,Type,glm::defaultp> p2 = glm::vec<3,Type,glm::defaultp>(   R[0][0]*p1[0]+R[0][1]*p1[1]+R[0][2]*p1[2],
                                                                          R[1][0]*p1[0]+R[1][1]*p1[1]+R[1][2]*p1[2],
                                                                          R[2][0]*p1[0]+R[2][1]*p1[1]+R[2][2]*p1[2]);

    std::cout<<std::endl;
    std::cout<<"Rotation with O3 matrix from quaternion: q->R"<<std::endl;
    std::cout<<"R*p \t= \t\t"<<std::setw(10)<<p2.x<<'\t'<<p2.y<<'\t'<<p2.z<<std::endl;

    quaternion<Type> q1 = convert(R);

    std::cout<<std::endl;
    std::cout<<"Rotation with quaternion from O3 matrix: q->R"<<std::endl;
    std::cout<<"R->q \t= \t"<<std::setw(10)<<q1<<std::endl;
    std::cout<<"p' \t= \t"<<std::setw(10)<<q1*p*invert(q1)<<std::endl;

    std::cout<<std::endl;
    std::cout<<"Convert from quaternion to Euler Angles"<<std::endl;
    glm::vec<3,Type,glm::defaultp> EAngles = convertToEulerAngles(q);
    std::cout<<"E Ang \t= \t"<<EAngles.x<<'\t'<<EAngles.y<<'\t'<<EAngles.z<<std::endl;

    std::cout<<std::endl;
    std::cout<<"Convert from Euler Angles to quaternion"<<std::endl;
    quaternion<Type> q2 = convert(EAngles.x,EAngles.y,EAngles.z);
    std::cout<<"q \t= \t"<<std::setw(10)<<q2<<std::endl;

    std::cout<<std::endl;
    std::cout<<"Convert from Angle and axis to quaternion"<<std::endl;
    quaternion<Type> q3 = convert(glm::radians(Type(90)),glm::vec<3,Type,glm::defaultp>(glm::sqrt(Type(2))/Type(2),Type(0),glm::sqrt(Type(2))/Type(2)));
    std::cout<<"q \t= \t"<<std::setw(10)<<q3<<std::endl;

    std::cout<<std::endl;
    std::cout<<"Convert from quaternion to Angle and axis"<<std::endl;
    quaternion<Type> q4 = convertToAnglesAndAxis(q3);
    std::cout<<"q \t= \t"<<std::setw(10)<<q4<<std::endl;

    std::cout<<std::endl;
    std::cout<<"dual quat"<<std::endl;
    glm::vec<4,Type,glm::defaultp> pos(Type(5),Type(0),Type(5),Type(1));

    dualQuaternion<Type> P1 = convert(
        convert(glm::radians(Type(90)),glm::vec<3,Type,glm::defaultp>(Type(0),Type(0),Type(1))),
        quaternion<Type>(Type(0),Type(5),Type(0),Type(0)));
    dualQuaternion<Type> P2= dualQuaternion<Type>(
        quaternion<Type>(Type(1),Type(0),Type(0),Type(0)),
        quaternion<Type>(Type(0),pos.x,pos.y,pos.z));

    std::cout<<"dual quat1 \t\t = "<<P1<<std::endl;
    std::cout<<"position (dual quat2) \t = "<<P2<<std::endl;

    std::cout<<"transformation = "<<P1*P2*conjugate(P1)<<std::endl;

    glm::mat<4,4,Type,glm::defaultp> SE3 = convert(P1);

    glm::vec<4,Type,glm::defaultp> newPos = glm::vec<4,Type,glm::defaultp>(   SE3[0][0]*pos[0]+SE3[0][1]*pos[1]+SE3[0][2]*pos[2]+SE3[0][3]*pos[3],
                                                                              SE3[1][0]*pos[0]+SE3[1][1]*pos[1]+SE3[1][2]*pos[2]+SE3[1][3]*pos[3],
                                                                              SE3[2][0]*pos[0]+SE3[2][1]*pos[1]+SE3[2][2]*pos[2]+SE3[2][3]*pos[3],
                                                                              SE3[3][0]*pos[0]+SE3[3][1]*pos[1]+SE3[3][2]*pos[2]+SE3[3][3]*pos[3]);

    std::cout<<"transformation by SE3 from quat1 "<<std::endl;
    std::cout<<newPos.x<<'\t'<<newPos.y<<'\t'<<newPos.z<<'\t'<<std::endl;

    dualQuaternion<Type> P3 = convert(SE3);

    std::cout<<"inverse transformation from SE3 to quat1= "<<std::endl;
    std::cout<<P3<<std::endl;


    std::cout<<std::endl;
    std::cout<<"=========================================="<<std::endl;

    std::vector<glm::vec<3,Type,glm::defaultp>> points(3);
    std::vector<glm::vec<3,Type,glm::defaultp>> points_(3);

    points[0] = {Type(1), Type(2), Type(3)};
    points[1] = {Type(4), Type(3), Type(5)};
    points[2] = {Type(1),-Type(2),-Type(3)};

    dualQuaternion<Type> Q = convert(convert(glm::radians(Type(45)),glm::normalize(glm::vec<3,Type,glm::defaultp>(Type(1),Type(1),Type(1)))),{Type(0),Type(2),-Type(2),Type(1)});

    for(size_t i=0;i<points.size();i++)
    {
        dualQuaternion<Type> pos({Type(1),Type(0),Type(0),Type(0)},{Type(0),Type(0.5)*points[i].x,Type(0.5)*points[i].y,Type(0.5)*points[i].z});
        dualQuaternion<Type> pos_ = Q*pos*conjugate(Q);
        points_[i] = pos_.translation().vector();
    }

    std::cout<<"originally given transformation = "<<Q<<std::endl;
    std::cout<<"reconstructed transformation    = "<<getTranformation(points,points_)<<std::endl;

    return 0;
}

template <typename type>
indices checkVector(const std::vector<glm::vec<3,type,glm::defaultp>>& x)
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

template <typename type>
dualQuaternion<type> getTranformation(const std::vector<glm::vec<3,type,glm::defaultp>>& x, const std::vector<glm::vec<3,type,glm::defaultp>>& x_)
{
    dualQuaternion<type> Q;

    indices ind = checkVector(x);

    if(ind.index0!=-1&&ind.index1!=-1&&ind.index2!=-1)
    {

        glm::vec<3,type,glm::defaultp> x1 = x[ind.index0];        glm::vec<3,type,glm::defaultp> x1_ = x_[ind.index0];
        glm::vec<3,type,glm::defaultp> x2 = x[ind.index1];        glm::vec<3,type,glm::defaultp> x2_ = x_[ind.index1];
        glm::vec<3,type,glm::defaultp> x3 = x[ind.index2];        glm::vec<3,type,glm::defaultp> x3_ = x_[ind.index2];

        glm::vec<3,type,glm::defaultp> t = x1 - x1_;

        glm::vec<3,type,glm::defaultp> u = x2 - x1;      glm::vec<3,type,glm::defaultp> u_ = x2_ - x1_;

        type cosTheta = glm::dot(u,u_)/(glm::length(u)*glm::length(u_));
        type theta = glm::acos(cosTheta);

        x1_ += t;
        x2_ += t;
        x3_ += t;

        type det = glm::determinant(glm::mat3x3( x1.x, x1.y, x1.z, x2.x, x2.y, x2.z, x2_.x, x2_.y, x2_.z));
        type a   = glm::determinant(glm::mat3x3(-type(1), x1.y, x1.z,-type(1), x2.y, x2.z, -type(1), x2_.y, x2_.z));
        type b   = glm::determinant(glm::mat3x3( x1.x,-type(1), x1.z, x2.x,-type(1), x2.z, x2_.x, -type(1), x2_.z));
        type c   = glm::determinant(glm::mat3x3( x1.x, x1.y,-type(1), x2.x, x2.y,-type(1), x2_.x, x2_.y, -type(1)));

        a /= det;
        b /= det;
        c /= det;

        type invNorma = 1.0f/glm::sqrt(a*a+b*b+c*c);
        glm::vec<3,type,glm::defaultp> n = {invNorma*a,invNorma*b,invNorma*c};

                   x1_ -= x1;
        x2 -= x1;  x2_ -= x1;
        x3 -= x1;  x3_ -= x1;
        x1 = {type(0),type(0),type(0)};

        quaternion<type> q1 = convert(theta,n);

        x1_ = quaternion<type>( q1 * quaternion<type>(0.0f,x1_) * conjugate(q1) ).vector();
        x2_ = quaternion<type>( q1 * quaternion<type>(0.0f,x2_) * conjugate(q1) ).vector();
        x3_ = quaternion<type>( q1 * quaternion<type>(0.0f,x3_) * conjugate(q1) ).vector();

                                       u = x2;                                     u_ = x2_;
        glm::vec<3,type,glm::defaultp> v = x3;      glm::vec<3,type,glm::defaultp> v_ = x3_;

        type cosPsi = glm::dot(u,v)/glm::length(u)/glm::length(v);
        glm::vec<3,type,glm::defaultp> l = cosPsi * glm::length(v) * glm::normalize(u);
        glm::vec<3,type,glm::defaultp> m  = normalize(v - l);
        glm::vec<3,type,glm::defaultp> m_ = normalize(v_ - l);

        type cosPhi = glm::dot(m,m_);
        type phi = -glm::acos(cosPhi);

        quaternion<type> q2 = convert(phi,normalize(u));

        quaternion<type> q = q1.invert()*q2.invert();

        glm::vec<3,type,glm::defaultp> xrot = quaternion<type>( q * quaternion<type>(type(0),x[0]) * conjugate(q) ).vector();
        glm::vec<3,type,glm::defaultp> tr = (x_[0] - xrot)/type(2);

        Q = convert(q,{0.0f,tr});
    }

    return Q;
}
