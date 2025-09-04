#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

#include <QApplication>
#include <QLabel>
#include <QPixmap>
#include <QThread>
#include <QMetaObject>
#include <QWidget>

#include <cstdlib>
#include <ctime>
#include <vector>

#include <QDesktopWidget>
#include <unordered_map>

#include <yolov10_ros_msgs/PersonMarkerData.h>
#include <std_msgs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>

QApplication *app_ptr = nullptr;
interactive_markers::InteractiveMarkerServer *server_ptr = nullptr;
QWidget *image_window = nullptr;
tf2_ros::Buffer tfBuffer;
ros::Publisher cooldown_pub;
std::unordered_map<std::string, std::size_t> marker_index_map;
std::string last_clicked_marker = "";

void showImage(const std::string &image_path)
{
    if (!app_ptr)
    {
        ROS_ERROR("QApplication이 초기화되지 않았습니다!");
        return;
    }

    QPixmap pixmap(QString::fromStdString(image_path));
    if (pixmap.isNull())
    {
        ROS_ERROR("이미지를 로드할 수 없습니다: %s", image_path.c_str());
        return;
    }

    if (image_window)
    {
        image_window->close();
        delete image_window;
        image_window = nullptr;
    }

    image_window = new QWidget();
    QLabel *label = new QLabel(image_window);
    label->setPixmap(pixmap);
    label->setScaledContents(true);
    label->resize(600, 400);

    image_window->resize(600, 400);
    image_window->setWindowTitle("Detected Image");
    image_window->show();

    QDesktopWidget desktop;
    int screenWidth = desktop.screenGeometry().width();
    int screenHeight = desktop.screenGeometry().height();

    int windowWidth = image_window->width();
    int windowHeight = image_window->height();

    int x = screenWidth - windowWidth - 20;
    int y = 20;

    image_window->move(x, y);
    image_window->show();
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
    {
        std::string marker_name = feedback->marker_name;

        if (!last_clicked_marker.empty() && last_clicked_marker != marker_name)
        {
            visualization_msgs::InteractiveMarker prev_marker;
            if (server_ptr->get(last_clicked_marker, prev_marker))
            {
                if (!prev_marker.controls.empty() && !prev_marker.controls[0].markers.empty())
                {
                    visualization_msgs::Marker &m = prev_marker.controls[0].markers[0];
                    m.color.r = 1.0;
                    m.color.g = 0.0;
                    m.color.b = 0.0;
                    m.color.a = 0.5;
                    m.scale.x = 0.2;
                    m.scale.y = 0.2;
                    m.scale.z = 0.2;

                    server_ptr->insert(prev_marker);
                }
            }
        }

        visualization_msgs::InteractiveMarker clicked_marker;
        if (server_ptr->get(marker_name, clicked_marker))
        {
            if (!clicked_marker.controls.empty() && !clicked_marker.controls[0].markers.empty())
            {
                visualization_msgs::Marker &marker = clicked_marker.controls[0].markers[0];
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 0.7;
                marker.scale.x = 0.3;
                marker.scale.y = 0.3;
                marker.scale.z = 0.3;

                server_ptr->insert(clicked_marker);
                server_ptr->applyChanges();
            }
        }

        last_clicked_marker = marker_name;

        ROS_INFO("Clicked Marker: %s at (%.2f, %.2f, %.2f)",
                 marker_name.c_str(),
                 feedback->pose.position.x,
                 feedback->pose.position.y,
                 feedback->pose.position.z);

        std::string image_path = "/home/user/person_captures/person_" + marker_name + ".jpg";

        QMetaObject::invokeMethod(app_ptr, [image_path]() {
            showImage(image_path);
        }, Qt::QueuedConnection);
    }
}

nav_msgs::Path latest_map_path;

void mapPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    latest_map_path = *msg;
    for (const auto& kv : marker_index_map)
    {
        const std::string& name = kv.first;
        std::size_t idx = kv.second;

        if (idx >= latest_map_path.poses.size())
            continue;

        visualization_msgs::InteractiveMarker im;
        if (server_ptr->get(name, im))
        {
            im.pose = latest_map_path.poses[idx].pose;
            server_ptr->insert(im);
        }
    }
    server_ptr->applyChanges();
}

void personMarkerCallback(const yolov10_ros_msgs::PersonMarkerData::ConstPtr& msg)
{
    static ros::Time last_marker_time = ros::Time(0);
    ros::Time current_time = ros::Time::now();

    if ((current_time - last_marker_time).toSec() < 5.0)
    {
        ROS_INFO("marker adding waiting... after %.2fsec ", 5.0 - (current_time - last_marker_time).toSec());
        return;
    }

    if (latest_map_path.poses.empty())
    {
        ROS_WARN("mapPath is empty, cannot set marker");
        return;
    }

    geometry_msgs::PoseStamped latest_pose = latest_map_path.poses.back();
    std::string marker_name = std::to_string(msg->image_index);

    visualization_msgs::InteractiveMarker interactiveMarker;
    interactiveMarker.header.frame_id = latest_pose.header.frame_id;
    interactiveMarker.name = marker_name;
    interactiveMarker.pose = latest_pose.pose;
    interactiveMarker.scale = 1.0;

    visualization_msgs::InteractiveMarkerControl control;
    control.name = "click_control";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    visualization_msgs::Marker sphereMarker;
    sphereMarker.type = visualization_msgs::Marker::SPHERE;
    sphereMarker.scale.x = 0.2;
    sphereMarker.scale.y = 0.2;
    sphereMarker.scale.z = 0.2;
    sphereMarker.color.r = 1.0;
    sphereMarker.color.g = 0.0;
    sphereMarker.color.b = 0.0;
    sphereMarker.color.a = 0.5;
    control.markers.push_back(sphereMarker);

    visualization_msgs::Marker arrowMarker;
    arrowMarker.type = visualization_msgs::Marker::ARROW;
    arrowMarker.scale.x = 0.3;
    arrowMarker.scale.y = 0.05;
    arrowMarker.scale.z = 0.05;
    arrowMarker.color.r = 1.0;
    arrowMarker.color.g = 0.0;
    arrowMarker.color.b = 0.0;
    arrowMarker.color.a = 0.8;
//    arrowMarker.pose = latest_pose.pose;
    control.markers.push_back(arrowMarker);

    interactiveMarker.controls.push_back(control);

    if (server_ptr)
    {
        server_ptr->insert(interactiveMarker, processFeedback);
        server_ptr->applyChanges();
        marker_index_map[marker_name] = latest_map_path.poses.size() - 1;

        // 🔁 퍼블리시 쿨다운 메시지
        std_msgs::Empty empty_msg;
        cooldown_pub.publish(empty_msg);
    }

    last_marker_time = current_time;

    ROS_INFO("Marker added at RTAB-Map position: %s (%.2f, %.2f, %.2f)",
             marker_name.c_str(),
             latest_pose.pose.position.x,
             latest_pose.pose.position.y,
             latest_pose.pose.position.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_node");
    ros::NodeHandle nh;

    tf2_ros::TransformListener tfListener(tfBuffer);

    int fake_argc = 1;
    char *fake_argv[] = {(char *)"ros_qt_node"};
    QApplication app(fake_argc, fake_argv);
    app.setQuitOnLastWindowClosed(false);
    app_ptr = &app;

    std::srand(std::time(0));

    server_ptr = new interactive_markers::InteractiveMarkerServer("marker_server");
    cooldown_pub = nh.advertise<std_msgs::Empty>("/cooldown_start", 10);

    ros::Subscriber sub = nh.subscribe("/person_marker_data", 10, personMarkerCallback);
    ros::Subscriber map_path_sub = nh.subscribe("/rtabmap/mapPath", 10, mapPathCallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    return app.exec();
}