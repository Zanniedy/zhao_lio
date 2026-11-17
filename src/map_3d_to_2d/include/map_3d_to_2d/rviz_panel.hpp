#pragma once

#include <memory>

#include <QDoubleSpinBox>
#include <QSlider>
#include <QLabel>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/string.hpp"

namespace map_3d_to_2d
{

class RvizPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RvizPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

Q_SIGNALS:
  void rangeReceived(double minimum, double maximum, double ground);
  void statusReceived(QString text);

private Q_SLOTS:
  void onMinSliderChanged(int value);
  void onMaxSliderChanged(int value);
  void onMinSpinChanged(double value);
  void onMaxSpinChanged(double value);

private:
  void setRange(double z_min, double z_max, bool publish = true);
  void publishRange();

  QDoubleSpinBox * min_spin_{nullptr};
  QDoubleSpinBox * max_spin_{nullptr};
  QSlider * min_slider_{nullptr};
  QSlider * max_slider_{nullptr};
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr range_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr range_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  QLabel * ground_label_{nullptr};
  QLabel * status_label_{nullptr};
  double z_min_{0.1};
  double z_max_{1.5};
};

}  // namespace map_3d_to_2d
