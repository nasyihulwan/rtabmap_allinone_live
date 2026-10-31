// Milestone (2026-10-30): real ROS2-wired Qt/QML visualizer, built on the
// v3 web_dashboard-style design (visualizer_qt_design/Main.qml). Separate
// package from visualizer/ (the field-tested ImGui app) — this one is new
// and unproven, must not put the ImGui app's field-test readiness at risk.
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QUrl>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "visualizer_qt/cv_image_item.h"
#include "visualizer_qt/ros_bridge.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  QGuiApplication app(argc, argv);

  qmlRegisterType<visualizer_qt::CvImageItem>("VisualizerQt", 1, 0, "CvImageItem");

  // RosBridge/QQmlApplicationEngine must be fully destroyed (their
  // destructors touch rcl resources — subscriptions, the node) before
  // rclcpp::shutdown() runs, or shutdown invalidates the rcl context out
  // from under them and the subscription/guard-condition destructors
  // crash on exit. Scoping them inside this block guarantees that order.
  int ret = 0;
  bool load_failed = false;
  {
    visualizer_qt::RosBridge bridge;

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("bridge", &bridge);

    std::string share_dir = ament_index_cpp::get_package_share_directory("visualizer_qt");
    // meshBaseUrl — QML needs a file:// URL (SceneLoader.source), not a
    // plain path, and the install path differs per machine (this dev
    // box vs. the Orin), so it can't be hardcoded in the QML itself.
    engine.rootContext()->setContextProperty(
      "meshBaseUrl",
      QUrl::fromLocalFile(QString::fromStdString(share_dir + "/meshes/husky_a200/")));
    QUrl qml_url = QUrl::fromLocalFile(QString::fromStdString(share_dir + "/qml/Main.qml"));
    engine.load(qml_url);

    if (engine.rootObjects().isEmpty()) {
      load_failed = true;
    } else {
      ret = app.exec();
    }
  }

  rclcpp::shutdown();
  return load_failed ? -1 : ret;
}
