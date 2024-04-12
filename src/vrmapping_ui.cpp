#include "vrmapping_ui.h"

namespace vrmapping_ui
{

  vrmapping_panel::vrmapping_panel(QWidget *parent) : rviz::Panel(parent)
  {

    GlobalPlannerByPositionClient_ =
        nh.serviceClient<vrmapping_msgs::VarmappingPubTarget>("/VRMap/PlannerByPosition");

    InitializationClient_ =
        nh.serviceClient<std_srvs::Trigger>("/VRMap/Initialization");

    PlannerStartClient_ = nh.serviceClient<std_srvs::Trigger>("/VRMap/StartExploration");

    PlannerStopClient_ = nh.serviceClient<std_srvs::Trigger>("/VRMap/StopExploration");

    ChangeUnitreeA1ControlStateClient_ = nh.serviceClient<std_srvs::Trigger>("/GazeboA1ROS/ControlA1StateChange");

    UnitreeA1ChangeToNavigationModeClient_ = nh.serviceClient<std_srvs::Trigger>("/GazeboA1ROS/ChangeToNavigationMode");

    UnitreeA1ChangeToJoyModeClient_ = nh.serviceClient<std_srvs::Trigger>("/GazeboA1ROS/ChangeToJoyMode");

    QVBoxLayout *v_box_layout = new QVBoxLayout;

    PlannerByGlobalPositionButton_ = new QPushButton;
    PlannerByGlobalPositionButton_->setText("Run Global Planner By Position");

    PlannerInitializationButton_ = new QPushButton;
    PlannerInitializationButton_->setText("Initialization");

    PlannerStartButton_ = new QPushButton;
    PlannerStartButton_->setText("Start Exploration");

    PlannerStopButton_ = new QPushButton;
    PlannerStopButton_->setText("Stop Exploration");

    ChangeUnitreeA1ControlStateButton_ = new QPushButton;
    ChangeUnitreeA1ControlStateButton_->setText("Change Unitree A1 Control State");

    UnitreeA1ChangeToNavigationModeButton_ = new QPushButton;
    UnitreeA1ChangeToNavigationModeButton_->setText("Unitree A1 Change To Navigation Mode");

    UnitreeA1ChangeToJoyModeButton_ = new QPushButton;
    UnitreeA1ChangeToJoyModeButton_->setText("Unitree A1 Change To Joy Mode");

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
    global_hbox_layout->addWidget(PlannerByGlobalPositionButton_);
    global_vbox_layout->addLayout(global_hbox_layout);

    // global_vbox_layout->addWidget(PlannerByGlobalPositionButton_);
    QHBoxLayout *global_hbox_for_second_layout_ = new QHBoxLayout;
    global_hbox_for_second_layout_->addWidget(PlannerInitializationButton_);
    global_hbox_for_second_layout_->addWidget(PlannerStartButton_);
    global_hbox_for_second_layout_->addWidget(PlannerStopButton_);
    global_vbox_layout->addLayout(global_hbox_for_second_layout_);
    global_vbox_layout->addWidget(ChangeUnitreeA1ControlStateButton_);
    global_vbox_layout->addWidget(UnitreeA1ChangeToNavigationModeButton_);
    global_vbox_layout->addWidget(UnitreeA1ChangeToJoyModeButton_);

    // global_vbox_layout->addWidget(PlannerInitializationButton_);
    v_box_layout->addLayout(global_vbox_layout);
    // v_box_layout->addWidget(PlannerByGlobalPositionButton_);

    setLayout(v_box_layout);

    connect(PlannerByGlobalPositionButton_, SIGNAL(clicked()), this,
            SLOT(on_global_planner_by_id_click()));

    connect(PlannerInitializationButton_, SIGNAL(clicked()), this,
            SLOT(on_initialization_click()));

    connect(PlannerStartButton_, SIGNAL(clicked()), this,
            SLOT(on_start_click()));

    connect(PlannerStopButton_, SIGNAL(clicked()), this,
            SLOT(on_stop_click()));

    connect(ChangeUnitreeA1ControlStateButton_, SIGNAL(clicked()), this,
            SLOT(on_ChangeUnitreeA1ControlState_click()));

    connect(UnitreeA1ChangeToNavigationModeButton_, SIGNAL(clicked()), this,
            SLOT(on_UnitreeA1ChangeToNavigationMode_click()));

    connect(UnitreeA1ChangeToJoyModeButton_, SIGNAL(clicked()), this,
            SLOT(on_UnitreeA1ChangeToJoyMode_click()));
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
        ROS_ERROR("[VRMapping-UI_Info]: - Invalid X coordinate: %s", InStringX.c_str());
        return;
      }
      catch (const std::invalid_argument &exc)
      {
        ROS_ERROR("[VRMapping-UI_Info]: - Invalid Invalid X coordinate: %s", InStringX.c_str());
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
        ROS_ERROR("[VRMapping-UI_Info]: - Invalid Y coordinate: %s", InStringY.c_str());
        return;
      }
      catch (const std::invalid_argument &exc)
      {
        ROS_ERROR("[VRMapping-UI_Info]: - Invalid Invalid Y coordinate: %s", InStringY.c_str());
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
        ROS_ERROR("[VRMapping-UI_Info]: - Invalid Z coordinate: %s", InStringZ.c_str());
        return;
      }
      catch (const std::invalid_argument &exc)
      {
        ROS_ERROR("[VRMapping-UI_Info]: - Invalid Invalid Z coordinate: %s", InStringZ.c_str());
        return;
      }
    }

    // check bounds on integer

    // we got an ID!!!!!!!!!
    ROS_INFO("[VRMapping-UI_Info]: :Global Planner receive target position : %f, %f, %f", TargetPointTem_.x,
             TargetPointTem_.y,
             TargetPointTem_.z);

    vrmapping_msgs::VarmappingPubTarget plan_srv;
    plan_srv.request.TargetPosition = TargetPointTem_;
    if (!GlobalPlannerByPositionClient_.call(plan_srv))
    {
      ROS_ERROR("[VRMapping-UI_Info]: Service call failed: %s",
                GlobalPlannerByPositionClient_.getService().c_str());
    }
    else
    {
      if (plan_srv.response.result)
      {
        ROS_INFO("[VRMapping-UI_Info]: Service call succeed: %s",
                 GlobalPlannerByPositionClient_.getService().c_str());
      }
      else
      {
        ROS_ERROR("[VRMapping-UI_Info]: Service call failed: %s. Reponse false.",
                  GlobalPlannerByPositionClient_.getService().c_str());
      }
    }
  }

  void vrmapping_panel::on_initialization_click()
  {
    std_srvs::Trigger srv;
    if (!InitializationClient_.call(srv))
    {
      ROS_ERROR("[VRMapping-UI_Info]: Service call failed: %s",
                InitializationClient_.getService().c_str());
    }
    else
    {
      ROS_INFO("[VRMapping-UI_Info]: Service call succeed: %s",
               InitializationClient_.getService().c_str());
    }
  }

  void vrmapping_panel::on_start_click()
  {
    std_srvs::Trigger srv;
    if (!PlannerStartClient_.call(srv))
    {
      ROS_ERROR("[VRMapping-UI_Info]: Service call failed: %s",
                PlannerStartClient_.getService().c_str());
    }
    else
    {
      ROS_INFO("[VRMapping-UI_Info]: Service call succeed: %s",
               PlannerStartClient_.getService().c_str());
    }
  }

  void vrmapping_panel::on_stop_click()
  {
    std_srvs::Trigger srv;
    if (!PlannerStopClient_.call(srv))
    {
      ROS_ERROR("[VRMapping-UI_Info]: Service call failed: %s",
                PlannerStopClient_.getService().c_str());
    }
    else
    {
      ROS_INFO("[VRMapping-UI_Info]: Service call succeed: %s",
               PlannerStopClient_.getService().c_str());
    }
  }

  void vrmapping_panel::on_ChangeUnitreeA1ControlState_click()
  {
    std_srvs::Trigger srv;
    if (!ChangeUnitreeA1ControlStateClient_.call(srv))
    {
      ROS_ERROR("[VRMapping-UI_Info]: Service call failed: %s",
                ChangeUnitreeA1ControlStateClient_.getService().c_str());
    }
    else
    {
      ROS_INFO("[VRMapping-UI_Info]: Service call succeed: %s",
               ChangeUnitreeA1ControlStateClient_.getService().c_str());
    }
  }

  void vrmapping_panel::on_UnitreeA1ChangeToNavigationMode_click()
  {
    std_srvs::Trigger srv;
    if (!UnitreeA1ChangeToNavigationModeClient_.call(srv))
    {
      ROS_ERROR("[VRMapping-UI_Info]: Service call failed: %s",
                UnitreeA1ChangeToNavigationModeClient_.getService().c_str());
    }
    else
    {
      ROS_INFO("[VRMapping-UI_Info]: Service call succeed: %s",
               UnitreeA1ChangeToNavigationModeClient_.getService().c_str());
    }
  }

  void vrmapping_panel::on_UnitreeA1ChangeToJoyMode_click()
  {
    std_srvs::Trigger srv;
    if (!UnitreeA1ChangeToJoyModeClient_.call(srv))
    {
      ROS_ERROR("[VRMapping-UI_Info]: Service call failed: %s",
                UnitreeA1ChangeToJoyModeClient_.getService().c_str());
    }
    else
    {
      ROS_INFO("[VRMapping-UI_Info]: Service call succeed: %s",
               UnitreeA1ChangeToJoyModeClient_.getService().c_str());
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
