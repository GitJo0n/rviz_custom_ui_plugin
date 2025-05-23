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

// #include <geometry_msgs/Point.h>  // 사람 위치 받을 때 사용
#include <yolov10_ros_msgs/PersonMarkerData.h> // 예시 경로
// #include <geometry_msgs/Point.h> // 기존 Point는 더 이상 직접 사용 안 함

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Path.h>


QApplication *app_ptr = nullptr;
interactive_markers::InteractiveMarkerServer *server_ptr = nullptr;
const int MAX_MARKERS = 10;
QWidget *image_window = nullptr;
tf2_ros::Buffer tfBuffer;

// recently clicked marker
std::string last_clicked_marker = "";


// 이미지 표시 함수
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

    // 이전에 열렸던 이미지 창이 있다면 닫기
    if (image_window)
    {
        image_window->close();
        delete image_window;
        image_window = nullptr;
    }

    // 새 이미지 창 생성
    image_window = new QWidget();
    QLabel *label = new QLabel(image_window);
    label->setPixmap(pixmap);
    label->setScaledContents(true);
    label->resize(600, 400);

    image_window->resize(600, 400);
    image_window->setWindowTitle("Detected Image");
    image_window->show();

    // 화면 해상도 기준 위치 설정
    QDesktopWidget desktop;
    int screenWidth = desktop.screenGeometry().width();
    int screenHeight = desktop.screenGeometry().height();

    int windowWidth = image_window->width();
    int windowHeight = image_window->height();

    int x = screenWidth - windowWidth - 20;  // 오른쪽 여백 20px
    int y = 20;                              // 상단 여백 20px

    image_window->move(x, y);  // 위치 지정
    image_window->show();

}


// 클릭 이벤트 콜백 함수
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
    {
        std::string marker_name = feedback->marker_name;

        // 이전 클릭 마커를 빨간색으로 되돌리기
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
                    m.scale.x = 0.1; // 크기 조정
                    m.scale.y = 0.1;
                    m.scale.z = 0.1;

                    server_ptr->insert(prev_marker);
                }
            }
        }

        // 현재 클릭된 마커를 초록색으로 변경
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
                marker.scale.x = 0.2; // 크기 조정
                marker.scale.y = 0.2;
                marker.scale.z = 0.2;

                server_ptr->insert(clicked_marker);
                server_ptr->applyChanges();  // 변경 적용
            }
        }

        // 마지막 클릭 마커 이름 업데이트
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

ros::Time last_marker_time = ros::Time(0);
// 콜백 함수: 사람 인식 시 마커 추가
// 콜백 함수 변경
// 콜백 함수 변경 (mapPath 좌표를 사용)
nav_msgs::Path latest_map_path;

// /rtabmap/mapPath 토픽의 callback 함수
void mapPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    latest_map_path = *msg;
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

    // 최신 좌표를 가져옴
    geometry_msgs::PoseStamped latest_pose = latest_map_path.poses.back();

    std::string marker_name = std::to_string(msg->image_index);

    visualization_msgs::InteractiveMarker interactiveMarker;
    interactiveMarker.header.frame_id = latest_pose.header.frame_id; // rtabmap mapPath의 frame_id 사용
    interactiveMarker.name = marker_name;
    interactiveMarker.pose = latest_pose.pose;
    interactiveMarker.scale = 1.0;

    visualization_msgs::InteractiveMarkerControl control;
    control.name = "click_control";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    visualization_msgs::Marker sphereMarker;
    sphereMarker.type = visualization_msgs::Marker::SPHERE;
    sphereMarker.scale.x = 0.1;
    sphereMarker.scale.y = 0.1;
    sphereMarker.scale.z = 0.1;
    sphereMarker.color.r = 1.0;
    sphereMarker.color.g = 0.0;
    sphereMarker.color.b = 0.0;
    sphereMarker.color.a = 0.5;

    control.markers.push_back(sphereMarker);
    interactiveMarker.controls.push_back(control);

    if (server_ptr)
    {
        ros::Publisher cooldown_pub = nh.advertise<std_msgs::Empty>("/cooldown_start", 10);
        server_ptr->insert(interactiveMarker, processFeedback);
        // 마커 추가 직후
        std_msgs::Empty msg;
        cooldown_pub.publish(msg);
        server_ptr->applyChanges();
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

    // TF listener
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Qt
    int fake_argc = 1;
    char *fake_argv[] = {(char *)"ros_qt_node"};
    QApplication app(fake_argc, fake_argv);
    app.setQuitOnLastWindowClosed(false);
    app_ptr = &app;

    std::srand(std::time(0));

    server_ptr = new interactive_markers::InteractiveMarkerServer("marker_server");

    // Subscriber (기존 personMarkerCallback)
    ros::Subscriber sub = nh.subscribe("/person_marker_data", 10, personMarkerCallback);

    // 새로 추가한 mapPath Subscriber
    ros::Subscriber map_path_sub = nh.subscribe("/rtabmap/mapPath", 10, mapPathCallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    return app.exec();
}
