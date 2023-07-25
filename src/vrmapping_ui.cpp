#include "vrmapping_ui.h"

namespace vrmapping_ui
{

  vrmapping_panel::vrmapping_panel(QWidget *parent) : rviz::Panel(parent)
  {

    global_planner_by_id_client_ =
        nh.serviceClient<vrmapping_msgs::VarmappingPubTarget>("/VRMap/PlannerByPosition");

    QVBoxLayout *v_box_layout = new QVBoxLayout;

    PlannerByGlobalPositionButton_ = new QPushButton;

    PlannerByGlobalPositionButton_->setText("Run Global Planner By Position");

    QVBoxLayout *global_vbox_layout = new QVBoxLayout;
    QHBoxLayout *global_hbox_layout = new QHBoxLayout;

    QLabel *text_label_ptr = new QLabel("Target Position:");

    TargetPositionXLineEdit_ = new QLineEdit();
    TargetPositionYLineEdit_ = new QLineEdit();
    TargetPositionZLineEdit_ = new QLineEdit();

    global_hbox_layout->addWidget(text_label_ptr);

    global_hbox_layout->addWidget(TargetPositionXLineEdit_);
    global_hbox_layout->addWidget(TargetPositionYLineEdit_);
    global_hbox_layout->addWidget(TargetPositionZLineEdit_);
    // global_hbox_layout->addWidget(PlannerByGlobalPositionButton_);
    global_vbox_layout->addLayout(global_hbox_layout);
    global_vbox_layout->addWidget(PlannerByGlobalPositionButton_);
    v_box_layout->addLayout(global_vbox_layout);
    // v_box_layout->addWidget(PlannerByGlobalPositionButton_);

    setLayout(v_box_layout);

    connect(PlannerByGlobalPositionButton_, SIGNAL(clicked()), this,
            SLOT(on_global_planner_by_id_click()));
  }

  void vrmapping_panel::on_global_planner_by_id_click()
  {
    // retrieve ID as a string
    std::string InStringX = TargetPositionXLineEdit_->text().toStdString();
    TargetPositionXLineEdit_->clear();
    std::string InStringY = TargetPositionYLineEdit_->text().toStdString();
    TargetPositionYLineEdit_->clear();
    std::string InStringZ = TargetPositionZLineEdit_->text().toStdString();
    TargetPositionZLineEdit_->clear();
    geometry_msgs::Point TargetPointTem_;
    if (InStringX.empty())
    {
      TargetPointTem_.x = 0;
    }
    else
    {
      // try to convert to an double value
      try
      {
        TargetPointTem_.x = std::stod(InStringX);
      }
      catch (const std::out_of_range &exc)
      {
        ROS_ERROR("[vrmapping_UI_Info] - Invalid X coordinate: %s", InStringX.c_str());
        return;
      }
      catch (const std::invalid_argument &exc)
      {
        ROS_ERROR("[vrmapping_UI_Info] - Invalid Invalid X coordinate: %s", InStringX.c_str());
        return;
      }
    }

    if (InStringY.empty())
    {
      TargetPointTem_.y = 0;
    }
    else
    {
      // try to convert to an double value
      try
      {
        TargetPointTem_.y = std::stod(InStringY);
      }
      catch (const std::out_of_range &exc)
      {
        ROS_ERROR("[vrmapping_UI_Info] - Invalid Y coordinate: %s", InStringY.c_str());
        return;
      }
      catch (const std::invalid_argument &exc)
      {
        ROS_ERROR("[vrmapping_UI_Info] - Invalid Invalid Y coordinate: %s", InStringY.c_str());
        return;
      }
    }

    if (InStringZ.empty())
    {
      TargetPointTem_.z = 0;
    }
    else
    {
      // try to convert to an double value
      try
      {
        TargetPointTem_.z = std::stod(InStringZ);
      }
      catch (const std::out_of_range &exc)
      {
        ROS_ERROR("[vrmapping_UI_Info] - Invalid Z coordinate: %s", InStringZ.c_str());
        return;
      }
      catch (const std::invalid_argument &exc)
      {
        ROS_ERROR("[vrmapping_UI_Info] - Invalid Invalid Z coordinate: %s", InStringZ.c_str());
        return;
      }
    }

    // check bounds on integer

    // we got an ID!!!!!!!!!
    ROS_INFO("[vrmapping_UI_Info] :Global Planner receive target position : %f, %f, %f", TargetPointTem_.x,
             TargetPointTem_.y,
             TargetPointTem_.z);

    vrmapping_msgs::VarmappingPubTarget plan_srv;
    plan_srv.request.TargetPosition = TargetPointTem_;
    if (!global_planner_by_id_client_.call(plan_srv))
    {
      ROS_ERROR("[vrmapping_UI_Info] Service call failed: %s",
                global_planner_by_id_client_.getService().c_str());
    }
    else
    {
      if (plan_srv.response.result)
      {
        ROS_INFO("[vrmapping_UI_Info] Service call succeed: %s",
                 global_planner_by_id_client_.getService().c_str());
      }
      else
      {
        ROS_ERROR("[vrmapping_UI_Info] Service call failed: %s. Reponse false.",
                  global_planner_by_id_client_.getService().c_str());
      }
    }
  }

  void vrmapping_panel::save(rviz::Config config) const
  {
    rviz::Panel::save(config);
  }
  void vrmapping_panel::load(const rviz::Config &config)
  {
    rviz::Panel::load(config);
  }

} // namespace gbplanner_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vrmapping_ui::vrmapping_panel, rviz::Panel)
