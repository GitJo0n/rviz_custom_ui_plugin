#include <QApplication>
#include <QDockWidget>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QTimer>
#include <QVBoxLayout>
#include <QMainWindow>
#include <QScreen>
#include <QFrame> 

#include <rviz/visualization_frame.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <rviz/visualization_manager.h>
#include <rviz/display.h>
#include <rviz/display_group.h>
#include <rviz/properties/property.h> 


int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    rviz::VisualizationFrame* frame = new rviz::VisualizationFrame();
    frame->initialize();

    rviz::VisualizationManager* manager = frame->getManager();
    rviz::DisplayGroup* root_display_group = manager->getRootDisplayGroup();

    // Fixed Frame 설정
    rviz::Property* global_options = root_display_group->subProp("Global Options");
    if (global_options)
        global_options->subProp("Fixed Frame")->setValue("map");

    // 기존 툴바 및 독 패널 숨김
    for (QToolBar* toolbar : frame->findChildren<QToolBar*>()) toolbar->hide();
    for (QDockWidget* dock : frame->findChildren<QDockWidget*>()) dock->hide();

    // ============= Interactive Marker Display 추가 =============
    rviz::Display* interactive_marker_display = manager->createDisplay(
        "rviz/InteractiveMarkers", "Interactive Markers", true);
    if (interactive_marker_display) { 
        root_display_group->addDisplay(interactive_marker_display);

        QTimer::singleShot(0, [interactive_marker_display]() {
            if (interactive_marker_display) {
                interactive_marker_display->subProp("Update Topic")->setValue("/marker_server/update");
                interactive_marker_display->subProp("Feedback Topic")->setValue("/marker_server/feedback");
            }
        });
    }


    // ============= YOLO 이미지 패널 추가 =============
    rviz::Display* image_display = manager->createDisplay("rviz/Image", "Camera", true);
    if (image_display) { 
        root_display_group->addDisplay(image_display);

        image_display->subProp("Image Topic")->setValue("/yolov10/detection_image");
        image_display->subProp("Queue Size")->setValue(10);
        image_display->subProp("Transport Hint")->setValue("raw");

        rviz::RenderPanel* image_render_panel = image_display->findChild<rviz::RenderPanel*>();
        if (image_render_panel) {
            QDockWidget* image_dock = new QDockWidget("YOLO Detection View"); // 창 제목 

            // 1. 테두리를 적용할 QFrame 생성
            QFrame* styled_content_frame = new QFrame();
            styled_content_frame->setObjectName("yoloImageFrame"); 
            styled_content_frame->setStyleSheet(
                "QFrame#yoloImageFrame {"
                "  border: 4px solid orange;"    // 테두리: 4픽셀, 색
                "  border-radius: 8px;"         // 모서리 둥글게
                "  background-color: black;"     // 프레임 배경색 (이미지 주변)
                "  padding: 0px;"               // 프레임 안쪽 여백 (0으로 하면 테두리가 RenderPanel에 바로 붙음)
                "}"
            );

            // 2. image_render_panel을 QFrame의 레이아웃에 추가
            //    image_render_panel의 부모는 이 과정에서 styled_content_frame으로 설정됨
            QVBoxLayout* frame_layout = new QVBoxLayout(styled_content_frame);
            frame_layout->setContentsMargins(0, 0, 0, 0); // RenderPanel이 프레임을 꽉 채우도록
            frame_layout->addWidget(image_render_panel);
            // styled_content_frame->setLayout(frame_layout); // 생성자에 부모 위젯 전달 시 자동 설정됨

            // 3. 스타일이 적용된 QFrame을 QDockWidget의 내용물로 설정
            image_dock->setWidget(styled_content_frame);

            // QDockWidget 설정 (기존과 동일)
            image_dock->setFloating(true);
            image_dock->setAllowedAreas(Qt::NoDockWidgetArea);
            image_dock->setWindowFlags(Qt::Window | Qt::FramelessWindowHint); // 프레임 없는 창
            image_dock->resize(510, 510); // 테두리 두께 등을 고려하여 약간 조정 가능

            QScreen* screen = QGuiApplication::primaryScreen(); // QGuiApplication 사용 시 #include <QGuiApplication> 필요 (QApplication이 QGuiApplication 상속)
            if (screen) { // null 체크 추가
                QRect screenGeometry = screen->geometry();
                image_dock->move(50, screenGeometry.height() - 550 - 10); // 위치 약간 조정 가능
            }


            QTimer::singleShot(500, [image_dock]() { // 타이머 지연을 약간 늘려 안정성 확보 가능
                if (image_dock) { // null 체크 추가
                    image_dock->setFloating(true); // 이미 설정했지만, 타이밍 이슈 방지 위해 한 번 더
                    image_dock->show(); // show() 대신 setHidden(false)도 가능
                    image_dock->raise();
                }
            });
             // image_dock->show(); // QTimer 내부에서 show를 하므로 여기서 미리 호출할 필요 없을 수 있음. 또는 여기서 호출하고 타이머에선 raise()만.
        }
    }


    // ============= PointCloud2 디스플레이 추가 =============
    rviz::Display* pointcloud_display = manager->createDisplay("rviz/PointCloud2", "PointCloud2", true);
    if (pointcloud_display) { // null 체크 추가
        root_display_group->addDisplay(pointcloud_display);

        pointcloud_display->subProp("Topic")->setValue("/camera/depth/color/points");
        pointcloud_display->subProp("Queue Size")->setValue(10);
        pointcloud_display->subProp("Style")->setValue("Points");
        pointcloud_display->subProp("Size (m)")->setValue(0.01);
    }

    // ============= MapCloud 디스플레이 추가 =============
    rviz::Display* mapcloud_display = manager->createDisplay("rtabmap_rviz_plugins/MapCloud", "MapCloud", true);
    if (mapcloud_display) { // null 체크 추가
        root_display_group->addDisplay(mapcloud_display);

        QTimer::singleShot(100, [mapcloud_display]() {
            if (mapcloud_display) {
                mapcloud_display->subProp("Topic")->setValue("/rtabmap/mapData");
                mapcloud_display->subProp("Style")->setValue("Points");
                mapcloud_display->subProp("Size (Pixels)")->setValue(3);
                mapcloud_display->subProp("Cloud decimation")->setValue(1);
                mapcloud_display->subProp("Cloud max depth (m)")->setValue(7.0); // 실수형으로 명시
                mapcloud_display->subProp("Cloud voxel size (m)")->setValue(0.01);
                mapcloud_display->subProp("Node filtering radius (m)")->setValue(0.0); // 실수형으로 명시
                mapcloud_display->subProp("Node filtering angle (deg)")->setValue(30.0); // 실수형으로 명시
                mapcloud_display->subProp("Download map")->setValue(true);
                mapcloud_display->subProp("Download graph")->setValue(true);
            }
        });
    }

    // ============= Mappath 디스플레이 추가 =============
    rviz::Display* mapgraph_display = manager->createDisplay("rviz/Path", "Path", true);
    if (mapgraph_display) { // null 체크 추가
        root_display_group->addDisplay(mapgraph_display);

        QTimer::singleShot(0, [mapgraph_display]() {
            if (mapgraph_display) {
                mapgraph_display->subProp("Topic")->setValue("/rtabmap/mapPath");
                mapgraph_display->subProp("Color")->setValue("0; 255; 0"); // QColor 문자열 형식 (세미콜론 후 공백은 rviz 버전에 따라 다를 수 있음)
                mapgraph_display->subProp("Line Style")->setValue("Lines");
                mapgraph_display->subProp("Line Width")->setValue(0.05); // Path의 Line Width는 보통 뷰포트 단위가 아닌 월드 단위일 수 있어 작게 설정. 확인 필요.
            }
        });
    }

    // ============= View 설정 =============
    QTimer::singleShot(0, [frame]() {
        if (frame) { // null 체크 추가
            rviz::ViewManager* view_manager = frame->getManager()->getViewManager();
            if (view_manager) {
                rviz::ViewController* view_controller = view_manager->getCurrent();
                if (view_controller) {
                    view_controller->subProp("Target Frame")->setValue("map"); // Target Frame은 실제 TF에 존재하는 프레임이어야 함
                    view_controller->subProp("Distance")->setValue(10.0); // Orbit 컨트롤러의 경우 Distance로 거리 조절
                    // X, Y, Z 직접 설정보다는 컨트롤러의 특정 속성 (Distance, Focal Point 등)을 활용하는 것이 일반적
                    // view_controller->subProp("X")->setValue(0.0);
                    // view_controller->subProp("Y")->setValue(0.0);
                    // view_controller->subProp("Z")->setValue(10.0);
                    view_controller->subProp("Yaw")->setValue(0.0); // 라디안 값 (0)
                    view_controller->subProp("Pitch")->setValue(1.570796); // 라디안 값 (약 90도)
                    // view_controller->subProp("Roll")->setValue(0.0); // Orbit에는 Roll이 없을 수 있음
                }
            }
        }
    });

    // 전체화면으로 실행
    frame->showFullScreen();
    return app.exec();
}