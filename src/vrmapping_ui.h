#ifndef VRMAPPING_UI_H
#define VRMAPPING_UI_H

#include <stdio.h>

#include <vrmapping_msgs/VarmappingPubTarget.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Point.h>

#ifndef Q_MOC_RUN
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <rviz/panel.h>
#endif

class QLineEdit;
class QPushButton;

namespace vrmapping_ui
{
  class vrmapping_panel : public rviz::Panel
  {
    Q_OBJECT
  public:
    vrmapping_panel(QWidget *parent = 0);
    virtual void load(const rviz::Config &config);
    virtual void save(rviz::Config config) const;

  public Q_SLOTS:

    void on_global_planner_by_id_click();
    void on_initialization_click();
    void on_start_click();
    void on_stop_click();

  protected Q_SLOTS:

  protected:
    QPushButton *PlannerByGlobalPositionButton_;
    QLineEdit *TargetPositionXLineEdit_;
    QLineEdit *TargetPositionYLineEdit_;
    QLineEdit *TargetPositionZLineEdit_;
    ros::ServiceClient GlobalPlannerByPositionClient_;

    QPushButton *PlannerInitializationButton_;
    ros::ServiceClient InitializationClient_;

    QPushButton *PlannerStartButton_;
    ros::ServiceClient PlannerStartClient_;

    QPushButton *PlannerStopButton_;
    ros::ServiceClient PlannerStopClient_;

    ros::NodeHandle nh;
  };

} // namespace vrmapping_ui

#endif // VRMAPPING_UI_H
