#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& w) {
  Eigen::Matrix3d w_hat;
  w_hat(0, 0) = 0;
  w_hat(0, 1) = -w(2);
  w_hat(0, 2) = w(1);
  w_hat(1, 0) = w(2);
  w_hat(1, 1) = 0;
  w_hat(1, 2) = -w(0);
  w_hat(2, 0) = -w(1);
  w_hat(2, 1) = w(0);
  w_hat(2, 2) = 0;
  return w_hat;
}

/*
 * @brief Normalize the given quaternion to unit quaternion.
 */
void quaternionNormalize(Eigen::Vector4d& q) {
  double norm = q.norm();
  q = q / norm;
  return;
}

/*
 * @brief Perform q1 * q2
 */
Eigen::Vector4d quaternionMultiplication(
    const Eigen::Vector4d& q1,
    const Eigen::Vector4d& q2) {
  Eigen::Matrix4d L;
  L(0, 0) =  q1(0); L(0, 1) = -q1(1); L(0, 2) = -q1(2); L(0, 3) =  -q1(3);
  L(1, 0) =  q1(1); L(1, 1) =  q1(0); L(1, 2) = -q1(3); L(1, 3) =  q1(2);
  L(2, 0) =  q1(2); L(2, 1) =  q1(3); L(2, 2) =  q1(0); L(2, 3) =  -q1(1);
  L(3, 0) =  q1(3); L(3, 1) = -q1(2); L(3, 2) =  q1(1); L(3, 3) =  q1(0);

  Eigen::Vector4d q = L * q2;
  quaternionNormalize(q);
  return q;
}

/*
 * @brief Convert the vector part of a quaternion to a
 *    full quaternion.
 * @note This function is useful to convert delta quaternion
 *    which is usually a 3x1 vector to a full quaternion.
 *    For more details, check Section 3.2 "Kalman Filter Update" in
 *    "Indirect Kalman Filter for 3D Attitude Estimation:
 *    A Tutorial for quaternion Algebra".
 */
Eigen::Vector4d smallAngleQuaternion(
    const Eigen::Vector3d& dtheta) {

  Eigen::Vector3d dq = dtheta / 2.0;
  Eigen::Vector4d q;
  double dq_square_norm = dq.squaredNorm();

  if (dq_square_norm <= 1) {
    q.head<3>() = dq;
    q(3) = std::sqrt(1-dq_square_norm);
  } else {
    q.head<3>() = dq;
    q(3) = 1;
    q = q / std::sqrt(1+dq_square_norm);
  }

  return q;
}

/*
 * @brief Convert a quaternion to the corresponding rotation matrix
 * @note Pay attention to the convention used. The function follows the
 *    conversion in "Indirect Kalman Filter for 3D Attitude Estimation:
 *    A Tutorial for Quaternion Algebra", Equation (78).
 *
 *    The input quaternion should be in the form
 *      [q1, q2, q3, q4(scalar)]^T
 */
Eigen::Matrix3d quaternionToRotation(
    const Eigen::Vector4d& q) {
  Eigen::Matrix3d R;
  double qw = q[0];
  double qx = q[1];
  double qy = q[2];
  double qz = q[3];
  R(0,0) = qw*qw + qx*qx - qy*qy -qz*qy;
  R(0,1) = 2*(qx*qy - qw*qz);
  R(0,2) = 2*(qx*qz + qw*qy);
  R(1,0) = 2*(qx*qy + qw*qz);
  R(1,1) = qw*qw - qx*qx + qy*qy -qz*qz;
  R(1,2) = 2*(qy*qz - qw*qx);
  R(2,0) = 2*(qx*qz - qw*qy);
  R(2,1) = 2*(qy*qz + qw*qx);
  R(2,2) = qw*qw - qx*qx - qy*qy + qz*qz;

  //TODO: Is it necessary to use the approximation equation
  //    (Equation (87)) when the rotation angle is small?
  return R;
}

/*
 * @brief Convert a rotation matrix to a quaternion.
 * @note Pay attention to the convention used. The function follows the
 *    conversion in "Indirect Kalman Filter for 3D Attitude Estimation:
 *    A Tutorial for Quaternion Algebra", Equation (78).
 *
 *    The input quaternion should be in the form
 *      [q1, q2, q3, q4(scalar)]^T
 */
Eigen::Vector4d rotationToQuaternion(
    const Eigen::Matrix3d& R) {
  Eigen::Vector4d score;
  score(0) = R(0, 0);
  score(1) = R(1, 1);
  score(2) = R(2, 2);
  score(3) = R.trace();

  int max_row = 0, max_col = 0;
  score.maxCoeff(&max_row, &max_col);

  Eigen::Vector4d q = Eigen::Vector4d::Zero();
  if (max_row == 0) {
    q(0) = std::sqrt(1+2*R(0, 0)-R.trace()) / 2.0;
    q(1) = (R(0, 1)+R(1, 0)) / (4*q(0));
    q(2) = (R(0, 2)+R(2, 0)) / (4*q(0));
    q(3) = (R(1, 2)-R(2, 1)) / (4*q(0));
  } else if (max_row == 1) {
    q(1) = std::sqrt(1+2*R(1, 1)-R.trace()) / 2.0;
    q(0) = (R(0, 1)+R(1, 0)) / (4*q(1));
    q(2) = (R(1, 2)+R(2, 1)) / (4*q(1));
    q(3) = (R(2, 0)-R(0, 2)) / (4*q(1));
  } else if (max_row == 2) {
    q(2) = std::sqrt(1+2*R(2, 2)-R.trace()) / 2.0;
    q(0) = (R(0, 2)+R(2, 0)) / (4*q(2));
    q(1) = (R(1, 2)+R(2, 1)) / (4*q(2));
    q(3) = (R(0, 1)-R(1, 0)) / (4*q(2));
  } else {
    q(3) = std::sqrt(1+R.trace()) / 2.0;
    q(0) = (R(1, 2)-R(2, 1)) / (4*q(3));
    q(1) = (R(2, 0)-R(0, 2)) / (4*q(3));
    q(2) = (R(0, 1)-R(1, 0)) / (4*q(3));
  }

  if (q(3) < 0) q = -q;
  quaternionNormalize(q);
  return q;
}


int main(int agec, char **argv)
{
    
    Eigen::Quaterniond q1(0.8, 0.65, 0.3, 0.55);
    Eigen::Quaterniond q2(0.4, 0.23, 0.85, 0.12);
    q1.normalize();
    q2.normalize();

    Eigen::Vector4d v1;
    v1 << 0.8, 0.65, 0.3, 0.55;
    Eigen::Vector4d v2;
    v2 << 0.4, 0.23, 0.85, 0.12;
    quaternionNormalize(v1);
    quaternionNormalize(v2);

    std::cout << "q1:" <<  q1.w() << ", " <<  q1.x() << ", " <<  q1.y()<< ", " <<  q1.z() << std::endl;
    std::cout << "v1:" << v1[0] << ", " << v1[1] << ", " << v1[2] << ", " << v1[3] << std::endl;

    std::cout << "q2:" <<  q2.w() << ", " <<  q2.x() << ", " <<  q2.y()<< ", " <<  q2.z() << std::endl;
    std::cout << "v2:" << v2[0] << ", " << v2[1] << ", " << v2[2] << ", " << v2[3] << std::endl;

    // Product
    std::cout << "Product" << std::endl;
    Eigen::Quaterniond q3 = q1*q2;
    q3.normalize();
    Eigen::Vector4d v3 = quaternionMultiplication(v1, v2);
    std::cout << "q3:" <<  q3.w() << ", " <<  q3.x() << ", " <<  q3.y()<< ", " <<  q3.z() << std::endl;
    std::cout << "v3:" << v3[0] << ", " << v3[1] << ", " << v3[2] << ", " << v3[3] << std::endl;

    // Conjugate
    std::cout << "Conjugate" << std::endl;
    Eigen::Quaterniond q4 = q3.conjugate();
    Eigen::Vector4d v4 = v3;
    v4.block<3,1>(1,0) = -v4.block<3,1>(1,0);
    std::cout << "q4:" <<  q4.w() << ", " <<  q4.x() << ", " <<  q4.y()<< ", " <<  q4.z() << std::endl;
    std::cout << "v4:" << v4[0] << ", " << v4[1] << ", " << v4[2] << ", " << v4[3] << std::endl;

    // Rotate matrix
    Eigen::Matrix3d R1 = q3.toRotationMatrix();
    Eigen::Matrix3d R2 = quaternionToRotation(v3);
    std::cout << "R1: " << std::endl;
    std::cout << R1 << std::endl;
    std::cout << "R2: " << std::endl;
    std::cout << R2 << std::endl;

    // Axis angle To Quaternion
    Eigen::Vector3d dTheta;
    dTheta << 0.5, 0.1, -0.66;
    double dThetaNorm = dTheta.norm();
    Eigen::AngleAxisd deltaAngleHalf(dThetaNorm, dTheta /dThetaNorm);
    Eigen::Quaterniond q5(deltaAngleHalf);
    Eigen::Vector4d v5;
    v5[0] = cos(dThetaNorm/2);
    v5.block<3,1>(1,0) = dTheta/dThetaNorm * sin(dThetaNorm/2);
    std::cout << "q5:" <<  q5.w() << ", " <<  q5.x() << ", " <<  q5.y()<< ", " <<  q5.z() << std::endl;
    std::cout << "v5:" << v5[0] << ", " << v5[1] << ", " << v5[2] << ", " << v5[3] << std::endl;

    //, , , 
    double x=-0.632165;
    double y=0.774542;
    double z=-0.00236924;
    double w=0.0201277;

    // double x=-0.638667;
    // double y=0.769303;
    // double z=0.00586753;
    // double w=0.0141894;

    Eigen::Quaterniond qq(w, x, y, z);
    std::cout << "RR: " << std::endl;
    std::cout << qq.toRotationMatrix() << std::endl;

    //
    std::cout << qq.coeffs() << std::endl;
    std::cout << qq.w() << "," << qq.x() << std::endl;

    Eigen::Matrix3d D;
    D << 1, 2, 3, -4, 5, -6, 7, 8, -9;
    std::cout << D << std::endl;
    std::cout << D.determinant() << std::endl;
}