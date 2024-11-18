/* rviz_panel.cpp

 * Copyright (C) 2023 SS47816

 * Rviz Panel for controling goal poses 
 
**/

#include <pluginlib/class_list_macros.hpp>
#include "interactive_tools/rviz_panel.hpp"

PLUGINLIB_EXPORT_CLASS(rviz_panel::simplePanel, rviz::Panel)

namespace rviz_panel
{
    simplePanel::simplePanel(QWidget * parent)
    :   rviz::Panel(parent),
        ui_(std::make_shared<Ui::TaskControlPanel>())
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);

        // Define ROS publisher
        this->pub_waypoint_cmd_ = nh_.advertise<std_msgs::String>("/rviz_panel/waypoint_cmd", 1);
        // this->pub_respawn_ = nh_.advertise<std_msgs::Int16>("/rviz_panel/respawn_objects", 1);

        // Connect the clicked signals to slots
        connect(ui_->pushButton_add, SIGNAL(clicked()), this, SLOT(on_button_add_clicked()));
        connect(ui_->pushButton_remove, SIGNAL(clicked()), this, SLOT(on_button_remove_clicked()));
        connect(ui_->pushButton_start, SIGNAL(clicked()), this, SLOT(on_button_start_clicked()));
        connect(ui_->pushButton_clear, SIGNAL(clicked()), this, SLOT(on_button_clear_clicked()));

        // Initialization
        waypoint_cmd_msg_.data = "";
    }

    // Buttons
    void simplePanel::on_button_add_clicked()
    {
        ROS_INFO_STREAM("Adding waypoints to the mission... Please use the 2D Nav Goal to select waypoint(s)");
        ui_->label_status->setText("Adding waypoints from 2D Nav Goal");
        this->waypoint_cmd_msg_.data = "Add";
        this->pub_waypoint_cmd_.publish(this->waypoint_cmd_msg_);
    }
    void simplePanel::on_button_remove_clicked()
    {
        ROS_INFO_STREAM("Removing waypoints from the mission...");
        ui_->label_status->setText("The last waypoint has been removed from the mission");
        this->waypoint_cmd_msg_.data = "Remove";
        this->pub_waypoint_cmd_.publish(this->waypoint_cmd_msg_);
    }
    void simplePanel::on_button_start_clicked()
    {
        ROS_INFO_STREAM("Starting mission...");
        ui_->label_status->setText("Starting Mission");
        this->waypoint_cmd_msg_.data = "Start";
        this->pub_waypoint_cmd_.publish(this->waypoint_cmd_msg_);
    }
    void simplePanel::on_button_clear_clicked()
    {
        ROS_INFO_STREAM("Clearing all waypoints...");
        ui_->label_status->setText("All waypoint has been cleared");
        this->waypoint_cmd_msg_.data = "Clear";
        this->pub_waypoint_cmd_.publish(this->waypoint_cmd_msg_);
    }

    // void simplePanel::on_button_regen_clicked()
    // {
    //     ROS_INFO_STREAM("Respawning Random Objects");
    //     ui_->label_status->setText("Please select a goal pose");
    //     this->regen_cmd_msg_.data = 1;
    //     this->pub_respawn_.publish(this->regen_cmd_msg_);
    // }
    // void simplePanel::on_button_clear_clicked()
    // {
    //     ROS_INFO_STREAM("Clearing Random Objects");
    //     ui_->label_status->setText("Please select a goal pose");
    //     this->regen_cmd_msg_.data = 0;
    //     this->pub_respawn_.publish(this->regen_cmd_msg_);
    // }

    /**
     *  Save all configuration data from this panel to the given
     *  Config object. It is important here that you call save()
     *  on the parent class so the class id and panel name get saved.
     */
    void simplePanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    /**
     *  Load all configuration data for this panel from the given Config object.
     */
    void simplePanel::load(const rviz::Config & config)
    {
        rviz::Panel::load(config);
    }
} // namespace rviz_panel
