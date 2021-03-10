#ifndef MATRIX_MATH_HPP
#define MATRIX_MATH_HPP
#include <Eigen/Core>


namespace FBUSEKF
{

static double Signum(double x)
{
    if(x < 0)
        return -1;
    else
        return 1;
}

static double Absolute(double x)
{
    if(x < 0)
        return -x;
    else
        return x;
}


static Eigen::Matrix3d SkewSymmetricMatrix(Eigen::Vector3d vec)
{
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
    mat(0,1) = -vec(2);
    mat(0,2) = vec(1);
    mat(1,0) = vec(2);
    mat(1,2) = -vec(0);
    mat(2,0) = -vec(1);
    mat(2,1) = vec(0);
    return mat;
}

static Eigen::Matrix4d QuaternionLeftProductMatrix(Eigen::Quaterniond q)
{
    double qw = q.w();
    double qx = q.x();
    double qy = q.y();
    double qz = q.z();
    Eigen::Matrix4d mat = qw * Eigen::Matrix4d::Identity();
    mat(0,1) = -qx;
    mat(0,2) = -qy;
    mat(0,3) = -qz;

    mat(1,0) = qx;
    mat(1,2) = -qz;
    mat(1,3) = qy;

    mat(2,0) = qy;
    mat(2,1) = qz;
    mat(2,3) = -qx;

    mat(3,0) = qz;
    mat(3,1) = -qy;
    mat(3,2) = qx;

    return mat;
};

static Eigen::Matrix4d QuaternionRightProductMatrix(Eigen::Quaterniond q)
{
    double qw = q.w();
    double qx = q.x();
    double qy = q.y();
    double qz = q.z();
    Eigen::Matrix4d mat = qw * Eigen::Matrix4d::Identity();
    mat(0,1) = -qx;
    mat(0,2) = -qy;
    mat(0,3) = -qz;

    mat(1,0) = qx;
    mat(1,2) = qz;
    mat(1,3) = -qy;

    mat(2,0) = qy;
    mat(2,1) = -qz;
    mat(2,3) = qx;
    
    mat(3,0) = qz;
    mat(3,1) = qy;
    mat(3,2) = -qx;
    
    return mat;
};

static Eigen::Quaterniond VectorToQuaterniond(Eigen::Vector3d v)
{
    double vnorm = v.norm();
    double qw = cos(vnorm/2);
    double qx = v(0)/vnorm*sin(vnorm/2);
    double qy = v(1)/vnorm*sin(vnorm/2);
    double qz = v(2)/vnorm*sin(vnorm/2);
    Eigen::Quaterniond Q(qw, qx, qy, qz);
    return Q;
};

}
#endif