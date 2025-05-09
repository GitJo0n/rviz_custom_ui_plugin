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

QApplication *app_ptr = nullptr;
interactive_markers::InteractiveMarkerServer *server_ptr = nullptr;
const int MAX_MARKERS = 10;
QWidget *image_window = nullptr;

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
void personMarkerCallback(const yolov10_ros_msgs::PersonMarkerData::ConstPtr& msg)
{
    static int marker_count = 0; // marker_count는 이제 여기서 관리 안해도 될 수 있음 (아래 설명 참고)
    static ros::Time last_marker_time = ros::Time(0);

    ros::Time current_time = ros::Time::now();
    if ((current_time - last_marker_time).toSec() < 5.0) // 5초 쿨타임은 유지
    {
        ROS_INFO("marker adding waiting... after %.2fsec ", 5.0 - (current_time - last_marker_time).toSec());
        return;
    }

    // MAX_MARKERS 제한은 유지할 수 있음 (선택 사항)
    // if (marker_count >= MAX_MARKERS) { ... return; }

    // --- 중요: 메시지에서 이미지 인덱스를 가져와 마커 이름으로 사용 ---
    std::string marker_name = std::to_string(msg->image_index);

    visualization_msgs::InteractiveMarker interactiveMarker;
    interactiveMarker.header.frame_id = "map";
    interactiveMarker.name = marker_name; // 받은 인덱스를 이름으로 사용
    interactiveMarker.pose.position = msg->position; // 받은 좌표 사용
    interactiveMarker.scale = 1.0;

    // ... (컨트롤 및 마커 설정은 기존과 동일) ...
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
    interactiveMarker.controls.push_back(control);
    // interactiveMarker.controls.clear(); // 필요 없음

    if (server_ptr)
    {
        server_ptr->insert(interactiveMarker, processFeedback);
        server_ptr->applyChanges();
    }

    // marker_count++; // Python에서 보낸 인덱스를 사용하므로 여기서 카운트할 필요 없음
    last_marker_time = current_time;

    ROS_INFO("Person marker added: %s (%.2f, %.2f, %.2f)",
             marker_name.c_str(), msg->position.x, msg->position.y, msg->position.z);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_node");
    ros::NodeHandle nh;

    // Qt 애플리케이션 초기화
    int fake_argc = 1;
    char *fake_argv[] = {(char *)"ros_qt_node"};
    QApplication app(fake_argc, fake_argv);
    app.setQuitOnLastWindowClosed(false); 
    app_ptr = &app;


    std::srand(std::time(0));

    server_ptr = new interactive_markers::InteractiveMarkerServer("marker_server");

    // Subscriber 변경
    // ros::Subscriber sub = nh.subscribe("/person_detected", 10, personCallback); // 기존 코드 주석 처리
    ros::Subscriber sub = nh.subscribe("/person_marker_data", 10, personMarkerCallback); // 새 토픽 및 콜백 구독

    ros::AsyncSpinner spinner(1);
    spinner.start();

    return app.exec();
}
