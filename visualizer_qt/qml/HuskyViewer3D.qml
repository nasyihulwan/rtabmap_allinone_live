import QtQuick 2.15
import QtQuick.Scene3D 2.0
import Qt3D.Core 2.15
import Qt3D.Render 2.15
import Qt3D.Extras 2.15

// Real Husky A200 mesh viewer — the actual clearpath_platform_description
// Collada/.dae files (base_link, top_chassis, wheel x4, bumper x2) and the
// pacs_top_plate.stl, same assets web_dashboard's React/Three.js
// RobotViewer.tsx uses (see rtabmap_allinone_live/web_dashboard/src/
// components/RobotViewer.tsx — this file mirrors its part list and
// positions). Qt3D's SceneLoader reads .dae via the qt3d-assimpsceneimport-
// plugin apt package (user installed both this and qml-module-qt3d
// 2026-10-31, neither ships by default — if this viewer renders blank on
// a fresh machine, check those two packages first).
//
// Coordinate convention: ROS is x=forward, y=left, z=up. Qt3D (like
// Three.js) is Y-up: qt3d(x, z, -y) = ros(x, y, z) — same conversion as
// web_dashboard/src/lib/rosToThree.ts, kept identical on purpose so the
// two viewers' part positions can be cross-checked against each other.
Item {
    id: root

    // clearpath_platform_description/urdf/a200/a200.urdf.xacro
    readonly property real wheelbase: 0.512
    readonly property real track: 0.555
    readonly property real wheelZ: 0.03282

    function rosToQt3d(x, y, z) {
        return Qt.vector3d(x, z, -y);
    }

    // Fixed camera, no drag-to-orbit — this is a readiness status display
    // (user: "panel robot itu harusnya tidak bisa di geser geser"), not
    // an interactive viewer. No OrbitCameraController/InputSettings, no
    // "input" aspect (nothing left that needs it). Also dropped the
    // decorative ground plane that was here before — it added no
    // information (guide's own test: "data apa yang ditampilkan elemen
    // ini? kalau jawabannya tidak ada, elemen itu dibuang"), and read as
    // a confusing white diamond rather than an obvious floor.
    Scene3D {
        anchors.fill: parent
        aspects: ["logic"]
        cameraAspectRatioMode: Scene3D.AutomaticAspectRatio

        Entity {
            id: sceneRoot

            Camera {
                id: camera
                projectionType: CameraLens.PerspectiveProjection
                fieldOfView: 40
                aspectRatio: root.width / Math.max(1, root.height)
                nearPlane: 0.01
                farPlane: 50
                position: Qt.vector3d(1.5, 1.1, 1.5)
                upVector: Qt.vector3d(0, 1, 0)
                viewCenter: Qt.vector3d(0, 0.18, 0)
            }

            components: [
                RenderSettings {
                    activeFrameGraph: ForwardRenderer {
                        camera: camera
                        clearColor: "#f3f5f8"
                    }
                }
            ]

            Entity {
                components: [
                    DirectionalLight { worldDirection: Qt.vector3d(-0.4, -1, -0.3); intensity: 0.9 }
                ]
            }
            Entity {
                components: [
                    PointLight { color: "#ffffff"; intensity: 0.6 },
                    Transform { translation: Qt.vector3d(1, 2, 1) }
                ]
            }

            Entity {
                components: [
                    SceneLoader { source: meshBaseUrl + "base_link.dae" }
                ]
            }
            Entity {
                components: [
                    SceneLoader { source: meshBaseUrl + "top_chassis.dae" }
                ]
            }

            // 4 wheels — same part (wheel.dae) loaded 4 times at the 4
            // corner positions, no spin animation (this is a readiness
            // display, not a drive simulator). NodeInstantiator, not
            // Repeater — Qt3D has no Repeater3D type, NodeInstantiator is
            // Qt3D.Core's model-driven Entity creator.
            NodeInstantiator {
                model: [
                    root.rosToQt3d(root.wheelbase / 2, root.track / 2, root.wheelZ),
                    root.rosToQt3d(root.wheelbase / 2, -root.track / 2, root.wheelZ),
                    root.rosToQt3d(-root.wheelbase / 2, root.track / 2, root.wheelZ),
                    root.rosToQt3d(-root.wheelbase / 2, -root.track / 2, root.wheelZ)
                ]
                delegate: Entity {
                    components: [
                        SceneLoader { source: meshBaseUrl + "wheel.dae" },
                        Transform { translation: modelData }
                    ]
                }
            }

            Entity {
                components: [
                    SceneLoader { source: meshBaseUrl + "bumper.dae" },
                    Transform { translation: root.rosToQt3d(0.481, 0, 0.091) }
                ]
            }
            Entity {
                components: [
                    SceneLoader { source: meshBaseUrl + "bumper.dae" },
                    Transform {
                        translation: root.rosToQt3d(-0.481, 0, 0.091)
                        rotationY: 180
                    }
                ]
            }

            // PACS top plate (.stl) — STL has no inherent up-axis
            // convention; clearpath_platform_description's STL assets are
            // authored Z-up like the rest of the URDF, so the same -90°
            // X-axis correction web_dashboard's STLLoader path needed
            // applies here too (see RobotViewer.tsx's StlMesh comment).
            Entity {
                components: [
                    SceneLoader { source: meshBaseUrl + "pacs_top_plate.stl" },
                    Transform {
                        translation: root.rosToQt3d(0.04825, 0, 0.224)
                        rotationX: -90
                    }
                ]
            }
        }
    }
}
