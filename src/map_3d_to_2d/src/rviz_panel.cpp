#include "map_3d_to_2d/rviz_panel.hpp"

#include <algorithm>
#include <cmath>

#include <QFormLayout>
#include <QLabel>
#include <QSignalBlocker>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace map_3d_to_2d
{

RvizPanel::RvizPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QFormLayout;
  min_spin_ = new QDoubleSpinBox(this);
  max_spin_ = new QDoubleSpinBox(this);
  min_slider_ = new QSlider(Qt::Horizontal, this);
  max_slider_ = new QSlider(Qt::Horizontal, this);
  for (auto * spin : {min_spin_, max_spin_}) {
    spin->setRange(-20.0, 20.0);
    spin->setDecimals(2);
    spin->setSingleStep(0.01);
    spin->setSuffix(" m");
  }
  for (auto * slider : {min_slider_, max_slider_}) {
    slider->setRange(-2000, 2000);
    slider->setSingleStep(1);
    slider->setPageStep(10);
  }
  ground_label_ = new QLabel("Waiting for mapper configuration", this);
  status_label_ = new QLabel("Waiting for mapper", this);
  status_label_->setWordWrap(true);
  layout->addRow("Ground Z", ground_label_);
  layout->addRow(new QLabel("Min above ground", this), min_spin_);
  layout->addRow(new QLabel("Z minimum slider", this), min_slider_);
  layout->addRow(new QLabel("Max above ground", this), max_spin_);
  layout->addRow(new QLabel("Z maximum slider", this), max_slider_);
  layout->addRow(status_label_);
  setLayout(layout);

  connect(min_slider_, &QSlider::valueChanged, this, &RvizPanel::onMinSliderChanged);
  connect(max_slider_, &QSlider::valueChanged, this, &RvizPanel::onMaxSliderChanged);
  connect(min_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &RvizPanel::onMinSpinChanged);
  connect(max_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &RvizPanel::onMaxSpinChanged);
  setRange(z_min_, z_max_);
  min_spin_->setEnabled(false);
  max_spin_->setEnabled(false);
  min_slider_->setEnabled(false);
  max_slider_->setEnabled(false);
  connect(this, &RvizPanel::rangeReceived, this,
    [this](double minimum, double maximum, double ground) {
      setRange(minimum, maximum, false);
      ground_label_->setText(QString::number(ground, 'f', 3) + " m");
      min_spin_->setEnabled(true);
      max_spin_->setEnabled(true);
      min_slider_->setEnabled(true);
      max_slider_->setEnabled(true);
    }, Qt::QueuedConnection);
  connect(this, &RvizPanel::statusReceived, status_label_, &QLabel::setText,
    Qt::QueuedConnection);
}

void RvizPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    return;
  }
  node_ = abstraction->get_raw_node();
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  range_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>(
    "/map_3d_to_2d/set_range", rclcpp::QoS(1).reliable());
  range_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
    "/map_3d_to_2d/range", qos,
    [this](geometry_msgs::msg::Vector3::ConstSharedPtr message) {
      Q_EMIT rangeReceived(message->x, message->y, message->z);
    });
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/map_3d_to_2d/status", qos,
    [this](std_msgs::msg::String::ConstSharedPtr message) {
      Q_EMIT statusReceived(QString::fromStdString(message->data));
    });
}

void RvizPanel::onMinSliderChanged(int value)
{
  setRange(static_cast<double>(value) / 100.0, z_max_);
}

void RvizPanel::onMaxSliderChanged(int value)
{
  setRange(z_min_, static_cast<double>(value) / 100.0);
}

void RvizPanel::onMinSpinChanged(double value)
{
  setRange(value, z_max_);
}

void RvizPanel::onMaxSpinChanged(double value)
{
  setRange(z_min_, value);
}

void RvizPanel::setRange(double z_min, double z_max, bool publish)
{
  if (!std::isfinite(z_min) || !std::isfinite(z_max)) {
    return;
  }
  if (publish) {
    z_min = std::clamp(z_min, min_spin_->minimum(), min_spin_->maximum());
    z_max = std::clamp(z_max, max_spin_->minimum(), max_spin_->maximum());
  }
  if (z_min > z_max) {
    if (std::abs(z_min - z_min_) > std::abs(z_max - z_max_)) {
      z_min = z_max;
    } else {
      z_max = z_min;
    }
  }
  z_min_ = z_min;
  z_max_ = z_max;
  {
    const QSignalBlocker block(min_spin_);
    min_spin_->setRange(std::min(-20.0, z_min_), std::max(20.0, z_max_));
    min_spin_->setValue(z_min_);
  }
  {
    const QSignalBlocker block(max_spin_);
    max_spin_->setRange(std::min(-20.0, z_min_), std::max(20.0, z_max_));
    max_spin_->setValue(z_max_);
  }
  {
    const QSignalBlocker block(min_slider_);
    min_slider_->setValue(static_cast<int>(std::lround(z_min_ * 100.0)));
  }
  {
    const QSignalBlocker block(max_slider_);
    max_slider_->setValue(static_cast<int>(std::lround(z_max_ * 100.0)));
  }
  if (publish) {
    publishRange();
  }
}

void RvizPanel::publishRange()
{
  if (!range_pub_) {
    return;
  }
  geometry_msgs::msg::Vector3 message;
  message.x = z_min_;
  message.y = z_max_;
  range_pub_->publish(message);
}

void RvizPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void RvizPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

}  // namespace map_3d_to_2d

PLUGINLIB_EXPORT_CLASS(map_3d_to_2d::RvizPanel, rviz_common::Panel)
