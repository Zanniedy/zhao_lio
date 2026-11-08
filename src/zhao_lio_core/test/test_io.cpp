#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "zhao_lio/io.h"

TEST(Io, WritePcdBinary) {
  const std::vector<Eigen::Vector3f> pts = {{1.f, 2.f, 3.f}, {-4.5f, 0.f, 7.25f}};
  const std::string path = ::testing::TempDir() + "zhao_lio_test.pcd";
  ASSERT_TRUE(zhao_lio::write_pcd(path, pts));

  std::ifstream f(path, std::ios::binary);
  std::string line, header;
  while (std::getline(f, line)) {
    header += line + "\n";
    if (line.rfind("DATA", 0) == 0) break;
  }
  EXPECT_NE(header.find("FIELDS x y z\n"), std::string::npos);
  EXPECT_NE(header.find("POINTS 2\n"), std::string::npos);
  EXPECT_NE(header.find("DATA binary"), std::string::npos);
  float buf[6];
  f.read(reinterpret_cast<char*>(buf), sizeof(buf));
  ASSERT_EQ(f.gcount(), static_cast<std::streamsize>(sizeof(buf)));
  EXPECT_FLOAT_EQ(buf[0], 1.f);
  EXPECT_FLOAT_EQ(buf[5], 7.25f);
  f.peek();
  EXPECT_TRUE(f.eof());  // nothing after the points
  std::remove(path.c_str());
}

TEST(Io, WritePcdBadPath) {
  EXPECT_FALSE(zhao_lio::write_pcd("/nonexistent_dir/x.pcd", {}));
}
