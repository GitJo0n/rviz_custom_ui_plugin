#include <QApplication>
#include <QDockWidget>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QTimer>
#include <QVBoxLayout>
#include <QMainWindow>
#include <QScreen>

#include <rviz/visualization_frame.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <rviz/visualization_manager.h>
#include <rviz/display.h>
#include <rviz/display_group.h>

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
    root_display_group->addDisplay(interactive_marker_display);

    QTimer::singleShot(0, [interactive_marker_display]() {
        if (interactive_marker_display) {
            interactive_marker_display->subProp("Update Topic")->setValue("/marker_server/update");
            interactive_marker_display->subProp("Feedback Topic")->setValue("/marker_server/feedback");
        }
    });

    // ============= YOLO 이미지 패널 추가 =============
    rviz::Display* image_display = manager->createDisplay("rviz/Image", "Camera", true);
    root_display_group->addDisplay(image_display);

    // 디스플레이 속성 설정은 안전하게 QTimer로 지연
    QTimer::singleShot(0, [image_display]() {
        if (image_display) {
            image_display->subProp("Image Topic")->setValue("/camera/color/image_raw_dd");
            image_display->subProp("Queue Size")->setValue(10);
            image_display->subProp("Transport Hint")->setValue("raw");
        }
    });

    // RenderPanel 추출
    rviz::RenderPanel* image_render_panel = image_display->findChild<rviz::RenderPanel*>();
    if (image_render_panel) {
        // RenderPanel 분리 및 크기 정책 설정
        image_render_panel->setParent(nullptr);
        image_render_panel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    
        // 레이아웃 컨테이너 생성
        QWidget* container = new QWidget();
        QVBoxLayout* layout = new QVBoxLayout(container);
        layout->setContentsMargins(0, 0, 0, 0);  // 패딩 제거
        layout->addWidget(image_render_panel);
    
        container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        // 독 위젯 설정
        QDockWidget* image_dock = new QDockWidget("CAMERA");
        image_dock->setWidget(container);

        image_dock->setFloating(true);
        image_dock->setAllowedAreas(Qt::NoDockWidgetArea);
        image_dock->setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
        image_dock->resize(1000, 1000);  // 원하는 크기 설정

        // 화면 하단에 위치
        QScreen* screen = QGuiApplication::primaryScreen();
        QRect screenGeometry = screen->geometry();
        image_dock->move(50, screenGeometry.height() - 1050);  // 화면 아래에서 1000픽셀 여유

        // 창을 위로 올리고 보이게 함
        QTimer::singleShot(500, [image_dock]() {
            image_dock->setFloating(true);
            image_dock->setHidden(false);
            image_dock->raise();
        });

        image_dock->show();
    }


    // ============= PointCloud2 디스플레이 추가 =============
    rviz::Display* pointcloud_display = manager->createDisplay("rviz/PointCloud2", "PointCloud2", true);
    root_display_group->addDisplay(pointcloud_display);

    pointcloud_display->subProp("Topic")->setValue("/camera/depth/color/points");
    pointcloud_display->subProp("Queue Size")->setValue(10);
    pointcloud_display->subProp("Style")->setValue("Points");
    pointcloud_display->subProp("Size (m)")->setValue(0.01);
    
    // ============= MapCloud 디스플레이 추가 =============
    rviz::Display* mapcloud_display = manager->createDisplay("rtabmap_rviz_plugins/MapCloud", "MapCloud", true);
    root_display_group->addDisplay(mapcloud_display);

    QTimer::singleShot(100, [mapcloud_display]() {
        if (mapcloud_display) {
	    mapcloud_display->subProp("Topic")->setValue("/rtabmap/mapData");
            mapcloud_display->subProp("Style")->setValue("Points");
	    mapcloud_display->subProp("Size (Pixels)")->setValue(3);
	    mapcloud_display->subProp("Cloud decimation")->setValue(1);
	    mapcloud_display->subProp("Cloud max depth (m)")->setValue(3);
	    mapcloud_display->subProp("Cloud voxel size (m)")->setValue(0.01);
	    mapcloud_display->subProp("Node filtering radius (m)")->setValue(0);
	    mapcloud_display->subProp("Node filtering angle (deg)")->setValue(30);
	    mapcloud_display->subProp("Download map")->setValue(true);
	    mapcloud_display->subProp("Download graph")->setValue(true);
        }
    });
    // ============= Mappath 디스플레이 추가 =============
    rviz::Display* mapgraph_display = manager->createDisplay("rviz/Path", "Path", true);
    root_display_group->addDisplay(mapgraph_display);

    QTimer::singleShot(0, [mapgraph_display]() {
        if (mapgraph_display) {
            mapgraph_display->subProp("Topic")->setValue("/rtabmap/mapPath");
            mapgraph_display->subProp("Color")->setValue("0;255;0"); // RGB 문자열
            mapgraph_display->subProp("Line Style")->setValue("Lines"); // Lines 또는 Billboards
            mapgraph_display->subProp("Line Width")->setValue(2.0);
        }
    });
    // ============= View 설정 =============
    QTimer::singleShot(0, [frame]() {
        rviz::ViewManager* view_manager = frame->getManager()->getViewManager();
        if (view_manager) {
            rviz::ViewController* view_controller = view_manager->getCurrent();
            if (view_controller) {
                view_controller->subProp("Target Frame")->setValue("map");
                view_controller->subProp("X")->setValue(0.0);
                view_controller->subProp("Y")->setValue(0.0);
                view_controller->subProp("Z")->setValue(10.0);
                view_controller->subProp("Yaw")->setValue(0.0);
                view_controller->subProp("Pitch")->setValue(1.57);
                view_controller->subProp("Roll")->setValue(0.0);
            }
        }
    });

    // 전체화면으로 실행
    frame->showFullScreen();
    return app.exec();
}
