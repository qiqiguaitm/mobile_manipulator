#include <rviz/panel.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

namespace arm_planner
{

class StopButtonPanel : public rviz::Panel
{
Q_OBJECT
public:
  StopButtonPanel(QWidget* parent = 0);
  
protected Q_SLOTS:
  void onStopButtonClicked();
  
protected:
  QPushButton* stop_button_;
  QLabel* status_label_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

StopButtonPanel::StopButtonPanel(QWidget* parent)
  : rviz::Panel(parent)
{
  // 创建UI组件
  QVBoxLayout* layout = new QVBoxLayout;
  
  stop_button_ = new QPushButton("EMERGENCY STOP");
  stop_button_->setStyleSheet("QPushButton { background-color: red; color: white; font-weight: bold; font-size: 14px; }");
  stop_button_->setMinimumHeight(50);
  
  status_label_ = new QLabel("Ready");
  status_label_->setStyleSheet("QLabel { color: green; }");
  
  layout->addWidget(stop_button_);
  layout->addWidget(status_label_);
  setLayout(layout);
  
  // 连接信号
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(onStopButtonClicked()));
  
  // 初始化MoveIt接口
  try {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("arm");
    ROS_INFO("StopButtonPanel: MoveGroup interface initialized");
  } catch (const std::exception& e) {
    ROS_ERROR("StopButtonPanel: Failed to initialize MoveGroup: %s", e.what());
    status_label_->setText("MoveGroup Error");
    status_label_->setStyleSheet("QLabel { color: red; }");
  }
}

void StopButtonPanel::onStopButtonClicked()
{
  if (!move_group_) {
    ROS_ERROR("StopButtonPanel: MoveGroup not initialized");
    return;
  }
  
  try {
    move_group_->stop();
    move_group_->clearPoseTargets();
    
    status_label_->setText("STOPPED");
    status_label_->setStyleSheet("QLabel { color: red; }");
    
    ROS_WARN("Emergency stop executed from RViz panel!");
    
    // 1秒后恢复状态
    QTimer::singleShot(1000, [this]() {
      status_label_->setText("Ready");
      status_label_->setStyleSheet("QLabel { color: green; }");
    });
    
  } catch (const std::exception& e) {
    ROS_ERROR("StopButtonPanel: Failed to stop arm: %s", e.what());
    status_label_->setText("Stop Error");
    status_label_->setStyleSheet("QLabel { color: red; }");
  }
}

} // namespace arm_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(arm_planner::StopButtonPanel, rviz::Panel)