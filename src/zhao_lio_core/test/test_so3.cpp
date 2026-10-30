#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "zhao_lio/so3.h"

using namespace zhao_lio;

TEST(So3, ExpQuarterTurnAboutZ) {
  const Eigen::Matrix3d R = so3_exp(Eigen::Vector3d(0, 0, M_PI / 2));
  EXPECT_TRUE((R * Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitY(), 1e-12));
}

TEST(So3, ExpIsOrthonormalAndMatchesAngleAxis) {
  const Eigen::Vector3d v(0.3, -0.7, 1.1);
  const Eigen::Matrix3d R = so3_exp(v);
  EXPECT_TRUE((R * R.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_TRUE(R.isApprox(Eigen::AngleAxisd(v.norm(), v.normalized()).toRotationMatrix(), 1e-12));
}

TEST(So3, TinyAngleIsFirstOrder) {
  const Eigen::Vector3d v(1e-9, 0, 0);
  EXPECT_TRUE(so3_exp(v).isApprox(Eigen::Matrix3d::Identity() + hat(v), 1e-15));
}

TEST(So3, JacobianMapsAxisToItself) {
  const Eigen::Vector3d v(0.2, 0.5, -0.4);
  EXPECT_TRUE((so3_A(v) * v).isApprox(v, 1e-12));
  EXPECT_TRUE(so3_A(Eigen::Vector3d::Zero()).isApprox(Eigen::Matrix3d::Identity()));
}
