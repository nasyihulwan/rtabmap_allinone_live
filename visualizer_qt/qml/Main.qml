import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Layouts 1.15
import VisualizerQt 1.0

// v5 — pixel-matched port of visualizer_qt_design/Main.qml (v5), which
// was itself rebuilt directly against the user's reference mockups
// (~/Downloads/visualizer_qt_mockup/visualizer_qt_mockup_main.html and
// _k1.html, read directly — not the markdown guide's ASCII wireframes).
// Structure/colors/spacing/icons all come from those two HTML files.
// Real data replaces the mockup's static arrays everywhere it exists;
// where it doesn't (Battery/yaw — no BMS badge or robotYawDeg property
// in RosBridge yet), the UI says so honestly instead of faking a number.
Window {
    id: root
    visible: true
    // Sizes to whatever screen it launches on (was hardcoded 1920x1000,
    // which only looked right on that exact resolution — broke on a 2K
    // display, and would break differently on a lower-res one too, since
    // every panel below used absolute pixel positions matching that one
    // canvas). Every panel below now sizes itself off `content`'s actual
    // width/height instead.
    width: Screen.width
    height: Screen.height
    visibility: Window.Maximized
    title: "RTAB-Map Onboard Visualizer"
    color: tok.bg

    property string currentPanel: "main"

    QtObject {
        id: tok
        readonly property color bg: "#f7f8fa"
        readonly property color card: "#ffffff"
        readonly property color border: "#14000000"
        readonly property color textPrimary: "#171a1f"
        readonly property color textSecondary: "#52514e"
        readonly property color textMuted: "#6b7280"
        readonly property color idleDot: "#9aa1a9"
        readonly property color accent: "#0a84ff"
        readonly property color good: "#0ca30c"
        readonly property color warningDot: "#9a6300"
        readonly property color warning: "#fab219"
        readonly property color critical: "#d03b3b"
        readonly property color chipBg: "#f3f5f8"
        readonly property string fontUi: "Inter"
        readonly property string fontMono: "JetBrainsMono Nerd Font"
    }

    // Only green/amber/red/advisory are real today (status_strip.h's
    // BadgeLevel enum has no kIdle yet — guide Stage 4). "idle" is wired
    // for when that lands; harmless no-op until then.
    function badgeColor(level) {
        if (level === "green") return tok.good;
        if (level === "amber") return tok.warningDot;
        if (level === "red") return tok.critical;
        if (level === "idle") return tok.idleDot;
        return tok.accent; // advisory
    }

    // ───────────────────────── icons ─────────────────────────
    function drawIconPath(ctx, name) {
        if (name === "pin") {
            ctx.beginPath();
            ctx.moveTo(12, 21);
            ctx.bezierCurveTo(12, 21, 6, 15.8, 6, 11);
            ctx.arc(12, 11, 6, Math.PI, 2 * Math.PI, false);
            ctx.bezierCurveTo(18, 15.8, 12, 21, 12, 21);
            ctx.closePath();
            ctx.stroke();
            ctx.beginPath();
            ctx.arc(12, 11, 2, 0, 2 * Math.PI);
            ctx.stroke();
        } else if (name === "home") {
            ctx.beginPath();
            ctx.moveTo(3, 11); ctx.lineTo(12, 3); ctx.lineTo(21, 11); ctx.lineTo(21, 21);
            ctx.lineTo(15, 21); ctx.lineTo(15, 15); ctx.lineTo(9, 15); ctx.lineTo(9, 21);
            ctx.lineTo(3, 21); ctx.closePath();
            ctx.stroke();
        } else if (name === "mapping") {
            ctx.beginPath();
            ctx.moveTo(9, 4); ctx.lineTo(3, 6); ctx.lineTo(3, 20); ctx.lineTo(9, 18);
            ctx.lineTo(15, 20); ctx.lineTo(21, 18); ctx.lineTo(21, 4); ctx.lineTo(15, 6);
            ctx.closePath(); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(9, 4); ctx.lineTo(9, 18); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(15, 6); ctx.lineTo(15, 20); ctx.stroke();
        } else if (name === "k1") {
            ctx.beginPath();
            ctx.arc(12, 12, 8, -0.35, Math.PI * 1.55, false);
            ctx.stroke();
            ctx.beginPath(); ctx.moveTo(20, 4); ctx.lineTo(20, 9); ctx.lineTo(15, 9); ctx.stroke();
        } else if (name === "k2") {
            ctx.beginPath(); ctx.arc(12, 12, 9, 0, 2 * Math.PI); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(8, 12.5); ctx.lineTo(10.7, 15.2); ctx.lineTo(16, 9.5); ctx.stroke();
        } else if (name === "sensors") {
            ctx.beginPath(); ctx.arc(12, 17, 1.6, 0, 2 * Math.PI); ctx.stroke();
            ctx.beginPath(); ctx.arc(12, 17, 6, Math.PI * 1.15, Math.PI * 1.85, false); ctx.stroke();
            ctx.beginPath(); ctx.arc(12, 17, 10, Math.PI * 1.15, Math.PI * 1.85, false); ctx.stroke();
        }
    }

    Component {
        id: navIcon
        Canvas {
            property string iconName: "home"
            property color strokeColor: tok.textSecondary
            onPaint: {
                var ctx = getContext("2d");
                ctx.reset();
                ctx.strokeStyle = strokeColor;
                ctx.lineWidth = 1.75 * (width / 24);
                ctx.lineCap = "round";
                ctx.lineJoin = "round";
                ctx.save();
                ctx.scale(width / 24, height / 24);
                drawIconPath(ctx, iconName);
                ctx.restore();
            }
            onStrokeColorChanged: requestPaint()
            onIconNameChanged: requestPaint()
        }
    }

    Component {
        id: checkMarker
        Item {
            property bool done: false
            width: 24; height: 24
            Rectangle {
                anchors.fill: parent
                radius: 12
                color: done ? tok.good : "transparent"
                border.width: done ? 0 : 2
                border.color: tok.accent
            }
            Canvas {
                anchors.fill: parent
                visible: done
                onPaint: {
                    var ctx = getContext("2d");
                    ctx.reset();
                    ctx.strokeStyle = "#ffffff";
                    ctx.lineWidth = 3 * (width / 24);
                    ctx.lineCap = "round"; ctx.lineJoin = "round";
                    ctx.save(); ctx.scale(width / 24, height / 24);
                    ctx.beginPath();
                    ctx.moveTo(5, 12.5); ctx.lineTo(9.5, 17); ctx.lineTo(19, 7.5);
                    ctx.stroke();
                    ctx.restore();
                }
            }
        }
    }

    Component {
        id: robotTriangle
        Canvas {
            onPaint: {
                var ctx = getContext("2d");
                ctx.reset();
                ctx.fillStyle = tok.accent;
                ctx.save(); ctx.scale(width / 14, height / 14);
                ctx.beginPath();
                ctx.moveTo(7, 1); ctx.lineTo(12, 12); ctx.lineTo(7, 9.5); ctx.lineTo(2, 12);
                ctx.closePath(); ctx.fill();
                ctx.restore();
            }
        }
    }

    // ───────────────────────── shared components ─────────────────────────
    Component {
        id: badgeChip
        Rectangle {
            height: 36
            width: chipRow.width + 28
            radius: 18
            color: tok.chipBg
            // Real badge data updates live (every ~1s tick) — a stray
            // glyph fragment from a *previous*, longer detail string was
            // visible just past the chip's current edge after the text
            // shrank to a shorter value (classic stale-paint artifact:
            // the old glyph's pixels never got cleared when the text got
            // shorter). Clipping to the chip's own current bounds cuts
            // off anything outside them regardless of cause.
            clip: true
            Row {
                id: chipRow
                anchors.centerIn: parent
                spacing: 10
                Rectangle { width: 10; height: 10; radius: 5; color: badgeColor(modelData.level); anchors.verticalCenter: parent.verticalCenter }
                Text { text: modelData.label; color: tok.textPrimary; font.family: tok.fontUi; font.pixelSize: 14; anchors.verticalCenter: parent.verticalCenter }
                Text {
                    // Some badge details are a full diagnostic string, not
                    // a short value (e.g. a missing TF's canTransform
                    // exception text) — unbounded, that single chip grew
                    // wide enough to overflow the STATUS card and overlap
                    // whatever's positioned after it (found via visual
                    // inspection at 2K). A `Text.elide` + `width:
                    // Math.min(implicitWidth, N)` binding fixed the
                    // overflow but left stray glyph fragments rendering
                    // next to neighboring chips — elide recalculating
                    // against its own implicitWidth is a multi-pass
                    // binding that this chip's Row/Rectangle auto-sizing
                    // chain resettles awkwardly. Truncating the plain
                    // string before it ever reaches Text avoids the whole
                    // elide/width feedback loop — single-pass sizing, no
                    // transient glyphs. Guide's own rule: long TF error
                    // strings don't belong inline in the chip at all, only
                    // a short word — full text in click/hover is still
                    // open (see memory).
                    text: modelData.detail.length > 28 ? modelData.detail.substring(0, 28) + "..." : modelData.detail
                    color: tok.textSecondary; font.family: tok.fontMono; font.pixelSize: 13
                    anchors.verticalCenter: parent.verticalCenter
                }
            }
        }
    }

    Component {
        id: statusRow
        Row {
            width: ListView.view ? ListView.view.width : parent.width
            // Content-driven, not a fixed 36: a group's chips wrap onto a
            // second line when they don't fit (PIPELINE's 4th chip,
            // relocalization_ok, does at typical widths). With height
            // pinned at 36 the wrapped chip spilled into the next row and
            // was painted behind the SESSION chips — looked like ghost
            // chips with glyph fragments poking out ("masih ada yang
            // tertimpa"). Earlier fixes chased this as a stale-paint /
            // delegate-churn bug; diagnostics showed no churn at all.
            height: Math.max(36, statusChipFlow.height)
            spacing: 16
            Text {
                // Row doesn't auto-center children vertically — this sat
                // at the Row's top (y:0) while the 36px-tall chips next to
                // it visually center their own content around y:18,
                // making the label look misaligned/floating above its own
                // row ("tulisan di kiri... tidak sejajar").
                y: (36 - height) / 2    // centered on the first chip line
                width: 96
                text: modelData.name
                color: tok.textMuted
                font.family: tok.fontUi
                font.pixelSize: 12
                font.weight: Font.Medium
                font.letterSpacing: 1
                font.capitalization: Font.AllUppercase
            }
            Flow {
                id: statusChipFlow
                width: parent.width - 112
                spacing: 10
                Repeater { model: modelData.items; delegate: badgeChip }
            }
        }
    }

    Component {
        id: primaryButton
        Rectangle {
            property string label: "Button"
            property var onClick: function () {}
            width: btnText.width + 48
            height: 44
            radius: 12
            color: tok.accent
            Text { id: btnText; anchors.centerIn: parent; text: label; color: "white"; font.family: tok.fontUi; font.pixelSize: 14; font.weight: Font.Medium }
            MouseArea { anchors.fill: parent; onClicked: onClick() }
        }
    }

    Component {
        id: stopButton
        Rectangle {
            property string stopLabel: "K1"
            property var onClick: function () {}
            width: btnText.width + 48
            height: 44
            radius: 12
            color: "transparent"
            border.width: 1.5
            border.color: tok.critical
            Text { id: btnText; anchors.centerIn: parent; text: "Stop " + stopLabel; color: tok.critical; font.family: tok.fontUi; font.pixelSize: 14; font.weight: Font.Medium }
            MouseArea { anchors.fill: parent; onClicked: onClick() }
        }
    }

    Component {
        id: logLevelPills
        Row {
            spacing: 6
            Repeater {
                model: [
                    { label: "Error", color: "#ff7a7a" },
                    { label: "Warn", color: "#fab219" },
                    { label: "Info", color: "#e5e7eb" },
                    { label: "Debug", color: "#9aa3ad" },
                    { label: "Proc", color: "#4fb8ff" }
                ]
                delegate: Rectangle {
                    height: 26; radius: 13; color: "#2a2f37"
                    width: pillText.width + 20
                    Text { id: pillText; anchors.centerIn: parent; text: modelData.label; color: modelData.color; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium }
                }
            }
        }
    }

    // ───────────────────────── real-data helpers ─────────────────────────
    function badgeDetail(label) {
        var list = bridge.statusBadges;
        for (var i = 0; i < list.length; i++) if (list[i].label === label) return list[i].detail;
        return "";
    }
    function badgeLevel(label) {
        var list = bridge.statusBadges;
        for (var i = 0; i < list.length; i++) if (list[i].label === label) return list[i].level;
        return "";
    }
    // Prefix lookup — the real Odom badge label is dynamic ("Odom
    // (/a200_1060/platform/odom/filtered)", built in status_strip.cpp).
    function badgeByPrefix(prefix) {
        var list = bridge.statusBadges;
        for (var i = 0; i < list.length; i++) if (list[i].label.indexOf(prefix) === 0) return list[i];
        return null;
    }
    function panelBadges(labels) {
        var out = [];
        for (var i = 0; i < labels.length; i++) { var b = badgeByPrefix(labels[i]); if (b) out.push(b); }
        return out;
    }
    function cameraRedBadge() {
        return badgeLevel("Camera color") === "red" || badgeLevel("Camera depth") === "red";
    }
    function sensorsReady() {
        if (cameraRedBadge()) return false;
        var scan = badgeByPrefix("Scan"); var odom = badgeByPrefix("Odom");
        if (scan && scan.level === "red") return false;
        if (odom && odom.level === "red") return false;
        return true;
    }
    function pipelineReady() {
        return badgeLevel("rtabmap") === "green";
    }
    function statusGroups() {
        var defs = [
            { name: "Sensors", labels: ["Scan", "Odom", "Camera color", "Camera depth"] },
            { name: "Transforms", labels: ["TF map->base", "TF base->camera", "TF base->laser"] },
            { name: "Pipeline", labels: ["rtabmap", "yolo_detector", "semantic_projector", "relocalization_ok"] },
            { name: "Session", labels: ["Session ID", "Phase", "Objects present"] }
        ];
        var out = [];
        for (var i = 0; i < defs.length; i++) {
            var items = panelBadges(defs[i].labels);
            if (items.length > 0) out.push({ name: defs[i].name, items: items });
        }
        return out;
    }
    // STATUS card's ListView model, cached and only reassigned when the
    // content actually differs. `bridge.statusBadges` is rebuilt with
    // brand-new QVariantMap objects in C++ every ~1s tick even when
    // nothing changed — binding `model:` directly to `statusGroups()`
    // meant the ListView saw a new array identity every tick regardless,
    // forcing a full destroy+recreate of every chip delegate once a
    // second. That constant teardown/rebuild cycle is what caused a
    // confirmed, reproducible glyph-fragment artifact on the Session row
    // (survived clip/elide/Scene3D-disable/layer.enabled fix attempts —
    // isolated by swapping this ListView to a one-time static array,
    // which made the artifact disappear completely, then confirmed
    // fixed for real data via this diff-before-reassign approach).
    property var cachedStatusGroups: []
    function refreshStatusGroups() {
        var fresh = statusGroups();
        if (JSON.stringify(fresh) !== JSON.stringify(cachedStatusGroups)) {
            cachedStatusGroups = fresh;
        }
    }
    Connections {
        target: bridge
        onStatusChanged: refreshStatusGroups()
    }
    Component.onCompleted: refreshStatusGroups()
    property var combinedLog: buildCombinedLog()
    function buildCombinedLog() {
        var out = [];
        var rl = bridge.rosoutLines;
        for (var i = 0; i < rl.length; i++) {
            var e = rl[i];
            var color = e.level >= 40 ? "#ff7a7a" : e.level >= 30 ? "#fab219" : e.level >= 20 ? "#e5e7eb" : "#9aa3ad";
            var lvl = e.level >= 40 ? "ERROR" : e.level >= 30 ? "WARN" : e.level >= 20 ? "INFO" : "DEBUG";
            out.push({ t: "", lvl: lvl, msg: e.node + "  " + e.message, color: color });
        }
        var pl = bridge.processOutputLines;
        for (var j = 0; j < pl.length; j++) out.push({ t: "", lvl: pl[j].tag, msg: pl[j].line, color: "#4fb8ff" });
        return out;
    }

    // ───────────────────────── sidebar ─────────────────────────
    Rectangle {
        id: sidebar
        x: 18; y: 24
        width: 84
        height: sidebarCol.height + 24
        radius: 22
        color: tok.card
        border.width: 1
        border.color: tok.border

        Column {
            id: sidebarCol
            anchors.top: parent.top
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.margins: 12
            spacing: 6

            Rectangle {
                width: 44; height: 44; radius: 12
                color: tok.textPrimary
                anchors.horizontalCenter: parent.horizontalCenter
                Loader { anchors.centerIn: parent; sourceComponent: navIcon; onLoaded: { item.width = 22; item.height = 22; item.iconName = "pin"; item.strokeColor = "#ffffff"; } }
            }

            Repeater {
                model: ListModel {
                    ListElement { label: "Home"; key: "main"; icon: "home" }
                    ListElement { label: "Sensors"; key: "sensors"; icon: "sensors" }
                    ListElement { label: "Mapping"; key: "mapping"; icon: "mapping" }
                    ListElement { label: "K1"; key: "k1"; icon: "k1" }
                    ListElement { label: "K2"; key: "k2"; icon: "k2" }
                }
                delegate: Rectangle {
                    width: 68; height: 66; radius: 16
                    anchors.horizontalCenter: parent ? parent.horizontalCenter : undefined
                    color: currentPanel === key ? tok.accent : "transparent"
                    Column {
                        anchors.centerIn: parent
                        spacing: 6
                        Loader {
                            anchors.horizontalCenter: parent.horizontalCenter
                            sourceComponent: navIcon
                            onLoaded: { item.width = 22; item.height = 22; item.iconName = icon; item.strokeColor = currentPanel === key ? "#ffffff" : tok.textSecondary; }
                        }
                        Text {
                            text: label
                            anchors.horizontalCenter: parent.horizontalCenter
                            font.family: tok.fontUi
                            font.pixelSize: 12
                            font.weight: Font.Medium
                            color: currentPanel === key ? "#ffffff" : tok.textSecondary
                        }
                    }
                    MouseArea { anchors.fill: parent; onClicked: root.currentPanel = key }
                }
            }
        }
    }

    // ───────────────────────── content ─────────────────────────
    // Fills whatever's left of the window after the fixed-width sidebar,
    // with the same 18/24px margins the mockup used — not a fixed
    // 1776x952 block sized for one specific resolution.
    Item {
        id: content
        anchors.left: sidebar.right; anchors.leftMargin: 18
        anchors.top: parent.top; anchors.topMargin: 24
        anchors.right: parent.right; anchors.rightMargin: 24
        anchors.bottom: parent.bottom; anchors.bottomMargin: 24

        Item {
            id: headerRow
            width: parent.width; height: 54
            Row {
                anchors.left: parent.left
                anchors.verticalCenter: parent.verticalCenter
                spacing: 14
                Text {
                    text: currentPanel === "main" ? "Home" : currentPanel === "sensors" ? "Sensors" : currentPanel === "mapping" ? "Mapping" : currentPanel === "k1" ? "K1 session" : "K2 session"
                    color: tok.textPrimary; font.family: tok.fontUi; font.pixelSize: 24; font.weight: Font.Medium
                    anchors.baseline: subtitleText.baseline
                }
                Text {
                    id: subtitleText
                    text: currentPanel === "main" ? "Robot readiness" : currentPanel === "sensors" ? "What starts, and why" : currentPanel === "mapping" ? "Build or load a map" : currentPanel === "k1" ? "Pose shift between restart cycles" : "Estimate vs. ground truth"
                    color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 14
                }
            }
            Row {
                anchors.right: parent.right
                anchors.verticalCenter: parent.verticalCenter
                spacing: 10
                Rectangle {
                    height: 36; radius: 18; color: tok.card; border.width: 1; border.color: tok.border
                    width: rosChipRow.width + 32
                    Row {
                        id: rosChipRow
                        anchors.centerIn: parent
                        spacing: 10
                        Rectangle { width: 10; height: 10; radius: 5; color: bridge.rosTopicCount > 0 ? tok.good : tok.idleDot; anchors.verticalCenter: parent.verticalCenter }
                        Text { text: bridge.rosTopicCount > 0 ? "ROS 2 connected" : "ROS 2 waiting"; color: tok.textPrimary; font.family: tok.fontUi; font.pixelSize: 14; anchors.verticalCenter: parent.verticalCenter }
                        Text { text: bridge.rosTopicCount + " topics"; color: tok.textSecondary; font.family: tok.fontMono; font.pixelSize: 13; anchors.verticalCenter: parent.verticalCenter }
                    }
                }
                Rectangle {
                    height: 36; radius: 18; color: tok.card; border.width: 1; border.color: tok.border
                    width: sessChipRow.width + 32
                    Row {
                        id: sessChipRow
                        anchors.centerIn: parent
                        spacing: 10
                        Rectangle { width: 10; height: 10; radius: 5; color: tok.accent; anchors.verticalCenter: parent.verticalCenter }
                        Text {
                            text: {
                                var sid = badgeDetail("Session ID");
                                return sid !== "" ? "Session" : "No logged session";
                            }
                            color: tok.textPrimary; font.family: tok.fontUi; font.pixelSize: 14; anchors.verticalCenter: parent.verticalCenter
                        }
                        Text { visible: badgeDetail("Session ID") !== ""; text: badgeDetail("Session ID"); color: tok.textSecondary; font.family: tok.fontMono; font.pixelSize: 13; anchors.verticalCenter: parent.verticalCenter }
                    }
                }
            }
        }

        // ───────────────────────── Main / Home panel ─────────────────────────
        Item {
            id: mainRow1
            visible: currentPanel === "main"
            anchors.top: headerRow.bottom; anchors.topMargin: 16
            anchors.left: parent.left; anchors.right: parent.right
            // Was `parent.height * 0.525` — a fixed fraction that had no
            // relation to rightCol's actual content height (Battery 150 +
            // Robot 420 + spacing = 586px fixed). At window sizes where
            // the fraction worked out shorter than that, rightCol
            // overflowed past mainRow1's bottom and visually collided
            // with the Status/Log row anchored below it (reported bug:
            // "button2 yang melenceng"). Bind to the one side that's
            // actually fixed-height instead of guessing a fraction.
            height: rightCol.height

            Rectangle {
                id: cameraPane
                anchors.left: parent.left; anchors.top: parent.top; anchors.bottom: parent.bottom
                width: parent.width * 0.619
                radius: 16; color: tok.card; border.width: 1; border.color: tok.border

                Rectangle {
                    anchors.fill: parent; anchors.margins: 12
                    radius: 12; color: "#e6e9ee"; clip: true

                    CvImageItem { anchors.fill: parent; image: bridge.cameraImage }
                    Text {
                        anchors.centerIn: parent
                        visible: !bridge.cameraImageValid
                        text: "waiting for /camera/camera/color/image_raw ..."
                        color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 13
                    }
                    Rectangle {
                        anchors.left: parent.left; anchors.top: parent.top; anchors.margins: 16
                        height: 34; radius: 17; color: tok.card
                        width: camChipRow.width + 28
                        Row {
                            id: camChipRow
                            anchors.centerIn: parent; spacing: 10
                            Rectangle { width: 10; height: 10; radius: 5; color: badgeColor(badgeLevel("Camera color")); anchors.verticalCenter: parent.verticalCenter }
                            Text { text: "Camera color"; font.family: tok.fontUi; font.pixelSize: 14; font.weight: Font.Medium; color: tok.textPrimary; anchors.verticalCenter: parent.verticalCenter }
                            Text { text: badgeDetail("Camera color") || "no data"; font.family: tok.fontMono; font.pixelSize: 13; color: tok.textSecondary; anchors.verticalCenter: parent.verticalCenter }
                        }
                    }
                }
            }

            Column {
                id: rightCol
                anchors.left: cameraPane.right; anchors.leftMargin: 16
                anchors.right: parent.right
                anchors.top: parent.top
                spacing: 16

                Rectangle {
                    width: parent.width; height: 150; radius: 16; color: tok.card; border.width: 1; border.color: tok.border
                    Column {
                        anchors.fill: parent; anchors.margins: 20
                        spacing: 10
                        Item {
                            width: parent.width; height: 16
                            Text { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; text: "BATTERY"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                            Row {
                                anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                                spacing: 8
                                Rectangle { width: 10; height: 10; radius: 5; color: tok.idleDot; anchors.verticalCenter: parent.verticalCenter }
                                Text { text: "not wired yet"; color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 14; anchors.verticalCenter: parent.verticalCenter }
                            }
                        }
                        Text { text: "—"; font.family: tok.fontMono; font.pixelSize: 40; font.weight: Font.Medium; color: tok.textMuted }
                        Rectangle { width: parent.width; height: 6; radius: 3; color: "#e9ecf0" }
                    }
                }

                Rectangle {
                    // Was height: 420 — too short for its own content
                    // (margins 40 + header ~20 + spacing 12 + viewport 362
                    // + sensor row ~20 = 454), so the Hokuyo/Depth/Battery
                    // row at the bottom got clipped by the card's own
                    // edge. Bound to content instead of a guessed number.
                    width: parent.width; height: robotCardBody.height + 40
                    radius: 16; color: tok.card; border.width: 1; border.color: tok.border
                    Column {
                        id: robotCardBody
                        anchors.top: parent.top; anchors.left: parent.left; anchors.right: parent.right
                        anchors.margins: 20
                        spacing: 6
                        Item {
                            width: parent.width; height: 20
                            Text { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; text: "ROBOT"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                            Text {
                                anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                                text: "yaw —"; color: tok.textSecondary; font.family: tok.fontMono; font.pixelSize: 13
                            }
                        }
                        // Real Husky A200 mesh (clearpath_platform_description
                        // Collada/.dae — same files web_dashboard's Three.js
                        // RobotViewer uses), not a drawn schematic. Fixed
                        // camera, not drag-orbitable — this is a status
                        // display, not an interactive viewer (user:
                        // "panel robot itu harusnya tidak bisa di geser
                        // geser"). Static pose for now (yaw above is "—"
                        // — no robotYawDeg property
                        // on RosBridge yet, same honesty reason as Battery).
                        //
                        // Loader with active: bound to the panel, not a bare
                        // HuskyViewer3D — Scene3D's render thread keeps
                        // running even when its item is only `visible:
                        // false` (it's still alive in the tree, just not
                        // painted), and that competed with the main Qt
                        // Quick render thread badly enough to hang the
                        // whole app ("Application Not Responding") after
                        // switching to another panel. `active: false`
                        // actually destroys the Scene3D/Entity tree instead
                        // of hiding it, which is what this needs.
                        Loader {
                            width: parent.width; height: 362
                            active: currentPanel === "main"
                            source: "HuskyViewer3D.qml"
                        }
                        Row {
                            width: parent.width
                            Repeater {
                                model: [
                                    { label: "Hokuyo", badge: "Scan" },
                                    { label: "Depth", badge: "Camera depth" },
                                    { label: "Battery", badge: "" }
                                ]
                                delegate: Row {
                                    width: parent.width / 3
                                    spacing: 6
                                    property var b: modelData.badge !== "" ? badgeByPrefix(modelData.badge) : null
                                    Rectangle { width: 8; height: 8; radius: 4; color: badgeColor(parent.b ? parent.b.level : "idle"); anchors.verticalCenter: parent.verticalCenter }
                                    Text { text: modelData.label; font.family: tok.fontUi; font.pixelSize: 12; color: tok.textPrimary; anchors.verticalCenter: parent.verticalCenter }
                                    Text { text: parent.b ? parent.b.detail : "not wired"; font.family: tok.fontMono; font.pixelSize: 11; color: tok.textMuted; anchors.verticalCenter: parent.verticalCenter }
                                }
                            }
                        }
                    }
                }
            }
        }

        Item {
            visible: currentPanel === "main"
            anchors.top: mainRow1.bottom; anchors.topMargin: 16
            anchors.bottom: parent.bottom
            anchors.left: parent.left; anchors.right: parent.right

            Rectangle {
                id: statusCard
                anchors.left: parent.left; anchors.top: parent.top; anchors.bottom: parent.bottom
                width: parent.width * 0.619
                radius: 16; color: tok.card; border.width: 1; border.color: tok.border
                Column {
                    anchors.fill: parent; anchors.margins: 20
                    spacing: 14
                    Item {
                        // Was a Row with a `width: parent.width - 420`
                        // spacer — a guessed budget for the 5-item legend's
                        // width that didn't hold at every window size
                        // (reported: "status panel masih timpang tindih").
                        // anchors.right pins the legend to the card's real
                        // right edge regardless of its own width.
                        width: parent.width; height: 16
                        Text { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; text: "STATUS"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                        Row {
                            anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                            spacing: 16
                            Repeater {
                                model: [
                                    { label: "OK", color: tok.good },
                                    { label: "Degraded", color: tok.warningDot },
                                    { label: "Broken", color: tok.critical },
                                    { label: "Not started", color: tok.idleDot },
                                    { label: "Info", color: tok.accent }
                                ]
                                delegate: Row {
                                    spacing: 6
                                    Rectangle { width: 8; height: 8; radius: 4; color: modelData.color; anchors.verticalCenter: parent.verticalCenter }
                                    Text { text: modelData.label; color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 12; anchors.verticalCenter: parent.verticalCenter }
                                }
                            }
                        }
                    }
                    ListView {
                        width: parent.width
                        height: parent.height - 50
                        clip: true
                        spacing: 14
                        model: cachedStatusGroups
                        delegate: statusRow
                    }
                }
            }

            Rectangle {
                anchors.left: statusCard.right; anchors.leftMargin: 16
                anchors.right: parent.right; anchors.top: parent.top; anchors.bottom: parent.bottom
                radius: 16; color: "#14171c"
                Column {
                    anchors.fill: parent; anchors.margins: 20
                    spacing: 12
                    Item {
                        // Was a Row with a `width: parent.width - 236`
                        // spacer-budget for the 5-pill legend — overflowed
                        // the card at some widths ("legend di log masih
                        // over"). anchors.right pins it to the real edge.
                        width: parent.width; height: 16
                        Text { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; text: "LOG"; color: "#9aa3ad"; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                        Loader { anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter; sourceComponent: logLevelPills }
                    }
                    ListView {
                        width: parent.width
                        height: parent.height - 46
                        clip: true
                        model: combinedLog
                        onCountChanged: positionViewAtEnd()
                        delegate: Row {
                            spacing: 10
                            Text { width: 44; text: modelData.lvl; color: modelData.color; font.family: tok.fontMono; font.pixelSize: 13 }
                            Text { text: modelData.msg; color: modelData.color; font.family: tok.fontMono; font.pixelSize: 13 }
                        }
                    }
                }
            }
        }

        // ───────────────────────── Sensors panel ─────────────────────────
        Item {
            visible: currentPanel === "sensors"
            anchors.top: headerRow.bottom; anchors.topMargin: 16
            anchors.bottom: parent.bottom
            anchors.left: parent.left; anchors.right: parent.right

            Rectangle {
                id: sensorsControlCard
                anchors.left: parent.left; anchors.top: parent.top; anchors.bottom: parent.bottom
                width: parent.width * 0.26
                radius: 16; color: tok.card; border.width: 1; border.color: tok.border

                Column {
                    id: sensorsTopBlock
                    anchors.top: parent.top; anchors.left: parent.left; anchors.right: parent.right
                    anchors.margins: 24
                    spacing: 14
                    Text { text: "SENSORS"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                    Text {
                        width: parent.width
                        text: "Starts the sensor stack this app depends on: RealSense camera (color + depth), Hokuyo lidar scan, and wheel odometry. Runs once per session — safe to press again, it is idempotent."
                        color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 14; wrapMode: Text.WordWrap
                    }
                    Column {
                        width: parent.width
                        spacing: 8
                        Repeater {
                            model: [
                                { label: "RealSense camera", detail: (badgeDetail("Camera color") || "not started") + " / " + (badgeDetail("Camera depth") || "not started"), level: badgeLevel("Camera color") || "idle" },
                                { label: "Hokuyo lidar", detail: badgeByPrefix("Scan") ? badgeByPrefix("Scan").detail : "not started", level: badgeByPrefix("Scan") ? badgeByPrefix("Scan").level : "idle" },
                                { label: "Wheel odometry", detail: badgeByPrefix("Odom") ? badgeByPrefix("Odom").detail : "not started", level: badgeByPrefix("Odom") ? badgeByPrefix("Odom").level : "idle" }
                            ]
                            delegate: badgeChip
                        }
                    }
                }
                Loader {
                    anchors.top: sensorsTopBlock.bottom; anchors.topMargin: 20
                    anchors.left: parent.left; anchors.leftMargin: 24
                    sourceComponent: primaryButton
                    onLoaded: {
                        item.label = Qt.binding(function () { return sensorsReady() ? "Sensors running" : "Start sensors"; });
                        item.onClick = bridge.startSensors;
                    }
                }
            }

            Rectangle {
                anchors.left: sensorsControlCard.right; anchors.leftMargin: 16
                anchors.right: parent.right; anchors.top: parent.top; anchors.bottom: parent.bottom
                radius: 16; color: tok.card; border.width: 1; border.color: tok.border
                Column {
                    anchors.fill: parent; anchors.margins: 24; spacing: 10
                    Text { text: "TOPICS"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                    Text { text: "Click a topic to see what it is and what depends on it."; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12 }
                    ListView {
                        width: parent.width
                        height: parent.height - 60
                        clip: true; spacing: 2
                        model: ListModel {
                            ListElement { topic: "/camera/camera/color/image_raw"; purpose: "RealSense color frames. Feeds the Camera panel, YOLO detector, and the Position Log's annotated crops."; rate: "started by: Start sensors · no fixed Hz threshold, liveness-only" }
                            ListElement { topic: "/camera/camera/depth/image_rect_raw"; purpose: "RealSense depth. Used to project 2D detections into 3D for the position log."; rate: "started by: Start sensors · no fixed Hz threshold, liveness-only" }
                            ListElement { topic: "/a200_1060/scan"; purpose: "Hokuyo lidar scan. Primary input to RTAB-Map's scan matching and Nav2's costmap."; rate: "started by: Start sensors · threshold: >= 8 Hz" }
                            ListElement { topic: "/a200_1060/platform/odom/filtered"; purpose: "Wheel odometry. RTAB-Map's motion prior between scans. Falls back to /odom if this isn't in the graph."; rate: "started by: Start sensors · threshold: >= 45 Hz" }
                            ListElement { topic: "/tf, /tf_static"; purpose: "Transform tree. map→a200_1060/base_link is what rtabmap publishes and what every pose lookup in this app reads."; rate: "started by: Start sensors + rtabmap pipeline" }
                            ListElement { topic: "/map"; purpose: "Occupancy grid published by RTAB-Map. Drawn in the Mapping/K1/K2 session panels, not here."; rate: "started by: Mapping / K1 / K2's pipeline button" }
                            ListElement { topic: "/rosout"; purpose: "Aggregated ROS2 log from every node. Feeds the Log panel on every screen."; rate: "always on once any node is running" }
                        }
                        delegate: Rectangle {
                            width: ListView.view.width
                            height: expanded ? 92 : 44
                            radius: 10
                            color: topicMouse.containsMouse ? tok.chipBg : "transparent"
                            property bool expanded: false
                            Behavior on height { NumberAnimation { duration: 160 } }
                            Column {
                                anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top; anchors.margins: 12
                                spacing: 6
                                Text { text: topic; font.family: tok.fontMono; font.pixelSize: 14; color: tok.textPrimary }
                                Text { visible: expanded; width: parent.width; text: purpose; color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 12; wrapMode: Text.WordWrap }
                                Text { visible: expanded; width: parent.width; text: rate; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 11 }
                            }
                            MouseArea { id: topicMouse; hoverEnabled: true; anchors.fill: parent; onClicked: expanded = !expanded }
                        }
                    }
                }
            }
        }

        // ───────────────────────── Mapping panel ─────────────────────────
        Item {
            id: mappingPanel
            visible: currentPanel === "mapping"
            anchors.top: headerRow.bottom; anchors.topMargin: 16
            anchors.bottom: parent.bottom
            anchors.left: parent.left; anchors.right: parent.right

            Rectangle {
                id: mappingControlCard
                anchors.left: parent.left; anchors.top: parent.top
                width: parent.width * 0.26
                // Was `parent.height * 0.59` — same bug as mainRow1
                // (fraction unrelated to actual content height, content
                // overflowed the card at some window sizes). 24+24 =
                // top+bottom margins, 20 = gap before the button row,
                // 44 = button height.
                height: mappingTopBlock.height + 24 + 20 + 44 + 24
                radius: 16; color: tok.card; border.width: 1; border.color: tok.border
                Column {
                    id: mappingTopBlock
                    anchors.top: parent.top; anchors.left: parent.left; anchors.right: parent.right
                    anchors.margins: 24; spacing: 20
                    Column {
                        width: parent.width; spacing: 10
                        Text { text: "MAPPING"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                        Text { width: parent.width; text: "Start mapping builds a fresh map from scratch and overwrites the DB. Load map localizes against the existing map only, nothing written."; color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 14; wrapMode: Text.WordWrap }
                    }
                    Column {
                        width: parent.width; spacing: 14
                        Row {
                            spacing: 14
                            Loader { sourceComponent: checkMarker; onLoaded: item.done = Qt.binding(function () { return sensorsReady(); }) }
                            Column {
                                Text { text: "Sensors ready"; font.family: tok.fontUi; font.pixelSize: 14; font.weight: Font.Medium; color: tok.textPrimary }
                                Text {
                                    text: {
                                        var scan = badgeByPrefix("Scan"); var odom = badgeByPrefix("Odom"); var cam = badgeDetail("Camera color");
                                        return "Scan " + (scan ? scan.detail : "—") + " · Odom " + (odom ? odom.detail : "—") + " · Camera " + (cam || "—");
                                    }
                                    font.family: tok.fontMono; font.pixelSize: 13; color: tok.textSecondary
                                }
                            }
                        }
                        Row {
                            spacing: 14
                            Loader { sourceComponent: checkMarker; onLoaded: item.done = Qt.binding(function () { return pipelineReady(); }) }
                            Column {
                                Text { text: "Pipeline running"; font.family: tok.fontUi; font.pixelSize: 14; font.weight: Font.Medium; color: tok.textPrimary }
                                Text { text: "rtabmap " + (badgeDetail("rtabmap") || "not started"); font.family: tok.fontMono; font.pixelSize: 13; color: tok.textSecondary }
                            }
                        }
                    }
                }
                Row {
                    anchors.top: mappingTopBlock.bottom; anchors.topMargin: 20
                    anchors.left: parent.left; anchors.leftMargin: 24
                    spacing: 12
                    Loader { visible: !pipelineReady(); sourceComponent: primaryButton; onLoaded: { item.label = "Start mapping"; item.onClick = function () { bridge.startPipeline("map_only"); }; } }
                    Loader { visible: !pipelineReady(); sourceComponent: primaryButton; onLoaded: { item.label = "Load map"; item.onClick = function () { bridge.startPipeline("localize"); }; } }
                    Loader { visible: pipelineReady(); sourceComponent: stopButton; onLoaded: item.stopLabel = "mapping" }
                }
            }
            Rectangle {
                anchors.left: mappingControlCard.right; anchors.leftMargin: 16
                anchors.right: parent.right; anchors.top: parent.top
                height: mappingControlCard.height
                radius: 16; color: "#c9cdd2"; border.width: 1; border.color: tok.border
                clip: true
                CvImageItem { anchors.fill: parent; anchors.margins: 16; image: bridge.mapImage }
                Text { anchors.centerIn: parent; visible: !bridge.mapImageValid; text: "waiting for /map ..."; color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 14 }
                Row {
                    anchors.top: parent.top; anchors.left: parent.left; anchors.margins: 16
                    spacing: 8
                    Text { text: "MAP"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                }
            }
            Rectangle {
                anchors.top: mappingControlCard.bottom; anchors.topMargin: 16
                anchors.bottom: parent.bottom; anchors.left: parent.left; anchors.right: parent.right
                radius: 16; color: "#14171c"
                Column {
                    anchors.fill: parent; anchors.margins: 20; spacing: 8
                    Item {
                        // Was a Row with a `width: parent.width - 236`
                        // spacer-budget for the 5-pill legend — overflowed
                        // the card at some widths ("legend di log masih
                        // over"). anchors.right pins it to the real edge.
                        width: parent.width; height: 16
                        Text { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; text: "LOG"; color: "#9aa3ad"; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                        Loader { anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter; sourceComponent: logLevelPills }
                    }
                    ListView {
                        width: parent.width
                        height: parent.height - 46
                        clip: true
                        model: combinedLog
                        delegate: Row {
                            spacing: 10
                            Text { width: 44; text: modelData.lvl; color: modelData.color; font.family: tok.fontMono; font.pixelSize: 13 }
                            Text { text: modelData.msg; color: modelData.color; font.family: tok.fontMono; font.pixelSize: 13 }
                        }
                    }
                }
            }
        }

        // ───────────────────────── K1 panel ─────────────────────────
        Item {
            id: k1Panel
            visible: currentPanel === "k1"
            anchors.top: headerRow.bottom; anchors.topMargin: 16
            anchors.bottom: parent.bottom
            anchors.left: parent.left; anchors.right: parent.right

            Rectangle {
                id: k1ControlCard
                anchors.left: parent.left; anchors.top: parent.top
                width: parent.width * 0.26
                height: k1TopBlock.height + 24 + 20 + 44 + 24
                radius: 16; color: tok.card; border.width: 1; border.color: tok.border
                Column {
                    id: k1TopBlock
                    anchors.top: parent.top; anchors.left: parent.left; anchors.right: parent.right
                    anchors.margins: 24; spacing: 20
                    Column {
                        width: parent.width; spacing: 10
                        Text { text: "K1 SESSION"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                        Text { width: parent.width; text: "Place the robot on the floor marker, start K1, then drive. The pipeline starts together; you do not set a pose."; color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 14; wrapMode: Text.WordWrap }
                    }
                    Column {
                        width: parent.width; spacing: 14
                        Row {
                            spacing: 14
                            Loader { sourceComponent: checkMarker; onLoaded: item.done = Qt.binding(function () { return sensorsReady(); }) }
                            Column {
                                Text { text: "Sensors ready"; font.family: tok.fontUi; font.pixelSize: 14; font.weight: Font.Medium; color: tok.textPrimary }
                                Text {
                                    text: {
                                        var scan = badgeByPrefix("Scan"); var odom = badgeByPrefix("Odom"); var cam = badgeDetail("Camera color");
                                        return "Scan " + (scan ? scan.detail : "—") + " · Odom " + (odom ? odom.detail : "—") + " · Camera " + (cam || "—");
                                    }
                                    font.family: tok.fontMono; font.pixelSize: 13; color: tok.textSecondary
                                }
                            }
                        }
                        Row {
                            spacing: 14
                            Loader { sourceComponent: checkMarker; onLoaded: item.done = Qt.binding(function () { return pipelineReady(); }) }
                            Column {
                                Text { text: "Pipeline running"; font.family: tok.fontUi; font.pixelSize: 14; font.weight: Font.Medium; color: tok.textPrimary }
                                Text { text: "rtabmap · TF " + (badgeDetail("TF map->base") || "—"); font.family: tok.fontMono; font.pixelSize: 13; color: tok.textSecondary }
                            }
                        }
                        Row {
                            spacing: 14
                            Loader { sourceComponent: checkMarker; onLoaded: item.done = Qt.binding(function () { return badgeDetail("relocalization_ok") === "true"; }) }
                            Column {
                                Text { text: "Relocalized"; font.family: tok.fontUi; font.pixelSize: 14; font.weight: Font.Medium; color: tok.textPrimary }
                                // Known limitation (project memory): relocalization_ok
                                // belongs to semantic_map_exporter, which doesn't run
                                // under husky_rtabmap — this will read "n/a" in K1
                                // until that's addressed, not a bug in this panel.
                                Text { text: "relocalization_ok = " + (badgeDetail("relocalization_ok") || "n/a"); font.family: tok.fontMono; font.pixelSize: 13; color: tok.textSecondary }
                            }
                        }
                        Row {
                            spacing: 14
                            Loader { sourceComponent: checkMarker; onLoaded: item.done = false }
                            Column {
                                Text { text: "Drive"; font.family: tok.fontUi; font.pixelSize: 14; font.weight: Font.Medium; color: tok.textPrimary }
                                Text { text: bridge.positionLogActive ? "Position log is filling" : "waiting for position log to start"; font.family: tok.fontUi; font.pixelSize: 13; color: tok.textSecondary }
                            }
                        }
                    }
                    Rectangle {
                        width: parent.width; radius: 12; color: "#eef6ff"
                        height: calloutText.height + 28
                        Row {
                            anchors.fill: parent; anchors.margins: 14; spacing: 12
                            Rectangle { width: 10; height: 10; radius: 5; color: tok.accent; anchors.top: parent.top; anchors.topMargin: 5 }
                            Text {
                                id: calloutText
                                width: parent.width - 30
                                text: "K1 has no initial-pose control by design. The robot relocalizes on its own, so the measured shift is its own."
                                color: tok.textPrimary; font.family: tok.fontUi; font.pixelSize: 13; wrapMode: Text.WordWrap
                            }
                        }
                    }
                }
                Row {
                    anchors.top: k1TopBlock.bottom; anchors.topMargin: 20
                    anchors.left: parent.left; anchors.leftMargin: 24
                    spacing: 12
                    Loader { visible: !pipelineReady(); sourceComponent: primaryButton; onLoaded: { item.label = "Start K1"; item.onClick = function () { bridge.startPipeline("husky_rtabmap", "LOCALIZATION=true"); }; } }
                    Loader { visible: pipelineReady(); sourceComponent: stopButton; onLoaded: item.stopLabel = "K1" }
                }
            }

            Rectangle {
                id: k1MapPane
                anchors.left: k1ControlCard.right; anchors.leftMargin: 16
                anchors.right: parent.right; anchors.top: parent.top
                height: k1ControlCard.height
                radius: 16; color: tok.card; border.width: 1; border.color: tok.border
                Column {
                    anchors.fill: parent; anchors.margins: 16; spacing: 12
                    Item {
                        width: parent.width; height: 16
                        Text { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; text: "MAP"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                        Row {
                            anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                            spacing: 8
                            Loader { sourceComponent: robotTriangle; onLoaded: { item.width = 14; item.height = 14; } }
                            Text { text: "Robot"; color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 13; anchors.verticalCenter: parent.verticalCenter }
                        }
                    }
                    Rectangle {
                        width: parent.width; height: parent.height - 44
                        radius: 12; color: "#c9cdd2"; clip: true
                        CvImageItem { anchors.fill: parent; anchors.margins: 16; image: bridge.mapImage }
                        Text { anchors.centerIn: parent; visible: !bridge.mapImageValid; text: "waiting for /map ..."; color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 14 }
                    }
                }
            }

            Rectangle {
                id: k1PosLogCard
                anchors.top: k1ControlCard.bottom; anchors.topMargin: 16
                anchors.bottom: parent.bottom; anchors.left: parent.left
                width: (parent.width - 16) / 2
                radius: 16; color: tok.card; border.width: 1; border.color: tok.border
                Column {
                    anchors.fill: parent; anchors.margins: 20; spacing: 10
                    Item {
                        width: parent.width; height: 16
                        Text { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; text: "POSITION LOG"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                        Row {
                            anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                            spacing: 8
                            Rectangle { width: 8; height: 8; radius: 4; color: bridge.positionLogActive ? tok.good : tok.idleDot; anchors.verticalCenter: parent.verticalCenter }
                            Text { text: bridge.positionLogActive ? "Recording" : "Idle"; color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 13; anchors.verticalCenter: parent.verticalCenter }
                        }
                    }
                    Row {
                        width: parent.width; height: 28; visible: bridge.positionLogActive
                        Text { width: 90; text: "FRAME"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium }
                        Text { width: 170; text: "CLASS"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium }
                        Text { width: 120; text: "TRACKER"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium }
                        Text { width: 130; horizontalAlignment: Text.AlignRight; text: "X (m)"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium }
                        Text { width: 130; horizontalAlignment: Text.AlignRight; text: "Y (m)"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium }
                    }
                    Rectangle { width: parent.width; height: 1; color: tok.border; visible: bridge.positionLogActive }
                    Text { visible: !bridge.positionLogActive; text: "no position-log file found under ~/.ros/position_logs/"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 13 }
                    ListView {
                        visible: bridge.positionLogActive
                        width: parent.width
                        height: parent.height - 90
                        clip: true
                        model: bridge.positionLogRows
                        delegate: Row {
                            height: 28
                            Text { width: 90; text: modelData.frameSeq; color: tok.textSecondary; font.family: tok.fontMono; font.pixelSize: 13 }
                            Text { width: 170; text: modelData.className; color: tok.textPrimary; font.family: tok.fontUi; font.pixelSize: 14 }
                            Text { width: 120; text: modelData.trackerId; color: tok.textSecondary; font.family: tok.fontMono; font.pixelSize: 13 }
                            Text { width: 130; horizontalAlignment: Text.AlignRight; text: modelData.x; color: tok.textPrimary; font.family: tok.fontMono; font.pixelSize: 13 }
                            Text { width: 130; horizontalAlignment: Text.AlignRight; text: modelData.y; color: tok.textPrimary; font.family: tok.fontMono; font.pixelSize: 13 }
                        }
                    }
                }
            }
            Rectangle {
                anchors.top: k1ControlCard.bottom; anchors.topMargin: 16
                anchors.bottom: parent.bottom; anchors.right: parent.right
                anchors.left: k1PosLogCard.right; anchors.leftMargin: 16
                radius: 16; color: "#14171c"
                Column {
                    anchors.fill: parent; anchors.margins: 20; spacing: 8
                    Item {
                        // Was a Row with a `width: parent.width - 236`
                        // spacer-budget for the 5-pill legend — overflowed
                        // the card at some widths ("legend di log masih
                        // over"). anchors.right pins it to the real edge.
                        width: parent.width; height: 16
                        Text { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; text: "LOG"; color: "#9aa3ad"; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                        Loader { anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter; sourceComponent: logLevelPills }
                    }
                    ListView {
                        width: parent.width
                        height: parent.height - 46
                        clip: true
                        model: combinedLog
                        delegate: Row {
                            spacing: 10
                            Text { width: 44; text: modelData.lvl; color: modelData.color; font.family: tok.fontMono; font.pixelSize: 13 }
                            Text { text: modelData.msg; color: modelData.color; font.family: tok.fontMono; font.pixelSize: 13 }
                        }
                    }
                }
            }
        }

        // ───────────────────────── K2 panel ─────────────────────────
        Item {
            id: k2Panel
            visible: currentPanel === "k2"
            anchors.top: headerRow.bottom; anchors.topMargin: 16
            anchors.bottom: parent.bottom
            anchors.left: parent.left; anchors.right: parent.right

            Rectangle {
                id: k2ControlCard
                anchors.left: parent.left; anchors.top: parent.top
                width: parent.width * 0.26
                height: k2TopBlock.height + 24 + 20 + 44 + 24
                radius: 16; color: tok.card; border.width: 1; border.color: tok.border
                Column {
                    id: k2TopBlock
                    anchors.top: parent.top; anchors.left: parent.left; anchors.right: parent.right
                    anchors.margins: 24; spacing: 20
                    Column {
                        width: parent.width; spacing: 10
                        Text { text: "K2 SESSION"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                        Text { width: parent.width; text: "Localizes against the existing map and compares the estimate to ground truth. One button starts every pipeline this mode needs."; color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 14; wrapMode: Text.WordWrap }
                    }
                    Column {
                        width: parent.width; spacing: 14
                        Row {
                            spacing: 14
                            Loader { sourceComponent: checkMarker; onLoaded: item.done = Qt.binding(function () { return sensorsReady(); }) }
                            Column {
                                Text { text: "Sensors ready"; font.family: tok.fontUi; font.pixelSize: 14; font.weight: Font.Medium; color: tok.textPrimary }
                                Text {
                                    text: {
                                        var scan = badgeByPrefix("Scan"); var odom = badgeByPrefix("Odom"); var cam = badgeDetail("Camera color");
                                        return "Scan " + (scan ? scan.detail : "—") + " · Odom " + (odom ? odom.detail : "—") + " · Camera " + (cam || "—");
                                    }
                                    font.family: tok.fontMono; font.pixelSize: 13; color: tok.textSecondary
                                }
                            }
                        }
                        Row {
                            spacing: 14
                            Loader { sourceComponent: checkMarker; onLoaded: item.done = Qt.binding(function () { return pipelineReady(); }) }
                            Column {
                                Text { text: "Pipeline running"; font.family: tok.fontUi; font.pixelSize: 14; font.weight: Font.Medium; color: tok.textPrimary }
                                Text { text: "rtabmap · TF " + (badgeDetail("TF map->base") || "—"); font.family: tok.fontMono; font.pixelSize: 13; color: tok.textSecondary }
                            }
                        }
                        Row {
                            spacing: 14
                            Loader { sourceComponent: checkMarker; onLoaded: item.done = false }
                            Column {
                                Text { text: "Drive"; font.family: tok.fontUi; font.pixelSize: 14; font.weight: Font.Medium; color: tok.textPrimary }
                                Text { text: bridge.positionLogActive ? "Position log is filling" : "waiting for position log to start"; font.family: tok.fontUi; font.pixelSize: 13; color: tok.textSecondary }
                            }
                        }
                    }
                }
                Row {
                    anchors.top: k2TopBlock.bottom; anchors.topMargin: 20
                    anchors.left: parent.left; anchors.leftMargin: 24
                    spacing: 12
                    Loader { visible: !pipelineReady(); sourceComponent: primaryButton; onLoaded: { item.label = "Start K2"; item.onClick = function () { bridge.startPipeline("husky_rtabmap", "LOCALIZATION=true"); }; } }
                    Loader { visible: pipelineReady(); sourceComponent: stopButton; onLoaded: item.stopLabel = "K2" }
                }
            }

            Rectangle {
                id: k2MapPane
                anchors.left: k2ControlCard.right; anchors.leftMargin: 16
                anchors.right: parent.right; anchors.top: parent.top
                height: k2ControlCard.height
                radius: 16; color: tok.card; border.width: 1; border.color: tok.border
                Column {
                    anchors.fill: parent; anchors.margins: 16; spacing: 12
                    Item {
                        width: parent.width; height: 16
                        Text { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; text: "MAP"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                        Row {
                            anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                            spacing: 8
                            Loader { sourceComponent: robotTriangle; onLoaded: { item.width = 14; item.height = 14; } }
                            Text { text: "Robot"; color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 13; anchors.verticalCenter: parent.verticalCenter }
                        }
                    }
                    Rectangle {
                        width: parent.width; height: parent.height - 44
                        radius: 12; color: "#c9cdd2"; clip: true
                        CvImageItem { anchors.fill: parent; anchors.margins: 16; image: bridge.mapImage }
                        Text { anchors.centerIn: parent; visible: !bridge.mapImageValid; text: "waiting for /map ..."; color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 14 }
                    }
                }
            }

            Rectangle {
                id: k2PosLogCard
                anchors.top: k2ControlCard.bottom; anchors.topMargin: 16
                anchors.bottom: parent.bottom; anchors.left: parent.left
                width: (parent.width - 16) / 2
                radius: 16; color: tok.card; border.width: 1; border.color: tok.border
                Column {
                    anchors.fill: parent; anchors.margins: 20; spacing: 10
                    Item {
                        width: parent.width; height: 16
                        Text { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; text: "POSITION LOG"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                        Row {
                            anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                            spacing: 8
                            Rectangle { width: 8; height: 8; radius: 4; color: bridge.positionLogActive ? tok.good : tok.idleDot; anchors.verticalCenter: parent.verticalCenter }
                            Text { text: bridge.positionLogActive ? "Recording" : "Idle"; color: tok.textSecondary; font.family: tok.fontUi; font.pixelSize: 13; anchors.verticalCenter: parent.verticalCenter }
                        }
                    }
                    Text { visible: !bridge.positionLogActive; text: "no position-log file found under ~/.ros/position_logs/"; color: tok.textMuted; font.family: tok.fontUi; font.pixelSize: 13 }
                    ListView {
                        visible: bridge.positionLogActive
                        width: parent.width
                        height: parent.height - 50
                        clip: true
                        model: bridge.positionLogRows
                        delegate: Row {
                            height: 28
                            Text { width: 90; text: modelData.frameSeq; color: tok.textSecondary; font.family: tok.fontMono; font.pixelSize: 13 }
                            Text { width: 170; text: modelData.className; color: tok.textPrimary; font.family: tok.fontUi; font.pixelSize: 14 }
                            Text { width: 120; text: modelData.trackerId; color: tok.textSecondary; font.family: tok.fontMono; font.pixelSize: 13 }
                            Text { width: 130; horizontalAlignment: Text.AlignRight; text: modelData.x; color: tok.textPrimary; font.family: tok.fontMono; font.pixelSize: 13 }
                            Text { width: 130; horizontalAlignment: Text.AlignRight; text: modelData.y; color: tok.textPrimary; font.family: tok.fontMono; font.pixelSize: 13 }
                        }
                    }
                }
            }
            Rectangle {
                anchors.top: k2ControlCard.bottom; anchors.topMargin: 16
                anchors.bottom: parent.bottom; anchors.right: parent.right
                anchors.left: k2PosLogCard.right; anchors.leftMargin: 16
                radius: 16; color: "#14171c"
                Column {
                    anchors.fill: parent; anchors.margins: 20; spacing: 8
                    Item {
                        // Was a Row with a `width: parent.width - 236`
                        // spacer-budget for the 5-pill legend — overflowed
                        // the card at some widths ("legend di log masih
                        // over"). anchors.right pins it to the real edge.
                        width: parent.width; height: 16
                        Text { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; text: "LOG"; color: "#9aa3ad"; font.family: tok.fontUi; font.pixelSize: 12; font.weight: Font.Medium; font.letterSpacing: 1 }
                        Loader { anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter; sourceComponent: logLevelPills }
                    }
                    ListView {
                        width: parent.width
                        height: parent.height - 46
                        clip: true
                        model: combinedLog
                        delegate: Row {
                            spacing: 10
                            Text { width: 44; text: modelData.lvl; color: modelData.color; font.family: tok.fontMono; font.pixelSize: 13 }
                            Text { text: modelData.msg; color: modelData.color; font.family: tok.fontMono; font.pixelSize: 13 }
                        }
                    }
                }
            }
        }
    }
}
