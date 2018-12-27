/**************************************************************************
 * Copyright (c) 2018 Chimney Xu. All Rights Reserve.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **************************************************************************/
/* *************************************************************************
   * File Name     : EditablePolygonController.cpp
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2018-12-17 10:03:47
   * Last Modified : smallchimney
   * Modified Time : 2018-12-26 20:46:30
************************************************************************* */
#include <editable_polygon/EditablePolygonController.h>
#include <polygon_painter/PolygonMarker.h>
#include <rviz/validate_floats.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace chimney_rviz_plugins {
namespace editable_polygon {

static EditablePolygonController* l_pController = nullptr;
static const std::string MENU("menu"); // NOLINT(cert-err58-cpp)

/* ********************************************************************************
 *                              SERVICE CALLBACKS                                 *
 ******************************************************************************** */

/**
 * @brief  clean all stamped polygon marker, clean the Ogre scenes' objects
 * @author smallchimney
 */
void EditablePolygonController::reset() {
    MFDClass::reset();
    m_pPolygonPainter -> reset();
    while(!m_mPolygons.empty()) {
        auto head = m_mPolygons.begin();
        head -> second.reset();
        m_mPolygons.erase(head);
    }
    m_pServer -> clear();
    m_pServer -> applyChanges();
}

/**
 * @brief  record vertex move events, update both the polygon data and the visual markers
 * @author smallchimney
 * @param  _Feedback  RViz event feedback message
 */
void reportMove(const EditablePolygonController::Feedback::ConstPtr& _Feedback) {
    using interactive::EditablePoints;
    assert(l_pController != nullptr);

    if(_Feedback -> control_name == MENU) {
        l_pController -> m_pFixedMenuHandler -> apply(
                *(l_pController -> m_pServer), _Feedback -> marker_name);
        return;
    }

    auto polygonIdx = EditablePoints::getPolygonIdx(_Feedback -> marker_name);
    auto pointIdx = EditablePoints::getVertexIdx(_Feedback -> control_name);
    auto& point = l_pController -> m_mPolygons[polygonIdx] -> polygon.points[pointIdx];

    visualization_msgs::InteractiveMarker int_marker;
    l_pController -> m_pServer -> get(_Feedback -> marker_name, int_marker);

    int_marker.controls[pointIdx].markers[0].pose.position.x += _Feedback -> pose.position.x;
    int_marker.controls[pointIdx].markers[0].pose.position.y += _Feedback -> pose.position.y;
    int_marker.controls[pointIdx].markers[0].pose.position.z += _Feedback -> pose.position.z;
    point.x = static_cast<float>(int_marker.controls[pointIdx].markers[0].pose.position.x);
    point.y = static_cast<float>(int_marker.controls[pointIdx].markers[0].pose.position.y);
    point.z = static_cast<float>(int_marker.controls[pointIdx].markers[0].pose.position.z);

    static geometry_msgs::Pose origin;
    l_pController -> m_pServer -> setPose(_Feedback -> marker_name, origin);
    l_pController -> m_pServer -> applyChanges();

    using geometry_msgs::Polygon;
    Polygon::ConstPtr polygon(new Polygon(l_pController -> m_mPolygons[polygonIdx] -> polygon));
    try {
        if(!l_pController -> m_pPolygonPainter -> paint(polygonIdx, _Feedback -> header, polygon)) {
            l_pController -> setStatus(rviz::StatusProperty::Error,
                                       "Interactive", "Error when try to update visual polygon");
        } else {
            l_pController -> setStatus(rviz::StatusProperty::Ok,
                                       "Interactive", "Update visual successful");
        }
    } catch(std::runtime_error& ex) {
        l_pController -> setStatus(rviz::StatusProperty::Warn,
                  "Paint",
                  ex.what());
        ROS_WARN_STREAM(ex.what());
    }

}

/**
 * @brief  record mouse key up events, update the visual markers
 * @author smallchimney
 * @param  _Feedback  RViz event feedback message
 */
void reportRelease(const EditablePolygonController::Feedback::ConstPtr& _Feedback) {
    using interactive::EditablePoints;
    assert(l_pController != nullptr);

    if(_Feedback -> control_name == MENU) {
        l_pController -> m_pFixedMenuHandler -> apply(
                *(l_pController -> m_pServer), _Feedback -> marker_name);
        return;
    }

    auto polygonIdx = EditablePoints::getPolygonIdx(_Feedback -> marker_name);
    auto& polygon = l_pController -> m_mPolygons[polygonIdx];

    auto marker = l_pController -> m_pInteractivePolygon -> getEditablePoints(
            polygonIdx, l_pController -> m_pVertexSize -> getFloat(), polygon -> header,
            polygon -> polygon.points, l_pController -> getVertexColor());
    if(marker == nullptr) {
        l_pController -> setStatus(rviz::StatusProperty::Warn,
                                   "Topic",
                                   "Message contained invalid floating point values"
                                   "(nans or infs)");
        ROS_WARN("something error during generate interactive polygon object");
        return;
    }
    l_pController -> m_pServer -> insert(*marker);
    l_pController -> m_pCurrentMenuHandler -> apply(
            *(l_pController -> m_pServer), marker -> name);
    l_pController -> m_pServer -> applyChanges();
}

/**
 * @brief  record menu select events, publish the polygon
 *          to "/chimney_rviz_plugins/editable_polygon/result"
 * @author smallchimney
 * @param  _Feedback  RViz event feedback message
 */
void publishPolygon(const EditablePolygonController::Feedback::ConstPtr& _Feedback) {
    assert(l_pController != nullptr);

    auto idx = interactive::EditablePoints::getPolygonIdx(_Feedback -> marker_name);

    assert(l_pController -> m_mPolygons.find(idx) != l_pController -> m_mPolygons.end());
    const auto& polygon = l_pController -> m_mPolygons[idx];
    l_pController -> m_cPolygonPub.publish(polygon);
}

/**
 * @brief  record menu select events, fix the polygon current operating, update
 *         the menu for it, publish the polygon data and wait for the next polygon
 * @author smallchimney
 * @param  _Feedback  RViz event feedback message
 */
void fixPolygon(const EditablePolygonController::Feedback::ConstPtr& _Feedback) {
    using interactive::EditablePoints;
    using geometry_msgs::Polygon;
    typedef visualization_msgs::InteractiveMarker Marker;
    typedef visualization_msgs::InteractiveMarkerControl Control;

    publishPolygon(_Feedback);

    const auto& polygonIdx = interactive::EditablePoints::getPolygonIdx(_Feedback -> marker_name);
    const auto& header = l_pController -> m_mPolygons[polygonIdx] -> header;
    Polygon::ConstPtr polygon(new Polygon(l_pController -> m_mPolygons[polygonIdx] -> polygon));
    const auto& currentPolygonName = EditablePoints::getInteractiveMarkerName(polygonIdx);
    const auto& server = l_pController -> m_pServer;
    const auto& menuHandler = l_pController -> m_pFixedMenuHandler;
    std::size_t hash = 0;
    boost::hash_combine(hash, polygonIdx);
    boost::hash_combine(hash, "menu");

    server -> erase(currentPolygonName);

    Marker marker;
    marker.name = currentPolygonName;
    marker.scale = 1.f;
    marker.header = header;
    // set 0 for position is not needed

    Control control;
    control.name = MENU;
    control.interaction_mode = Control::MENU;
    control.always_visible = static_cast<unsigned char>(true);
    control.markers.emplace_back(visualize::PolygonMarker::getBorder(static_cast<int>(hash), header,
            polygon, l_pController -> getVertexColor(), l_pController -> m_pVertexSize -> getFloat()));
    marker.controls.emplace_back(control);
    server -> insert(marker);
    menuHandler -> apply(*server, currentPolygonName);
    server -> applyChanges();
    l_pController -> m_ulLatestPolygon++;
}

/**
 * @brief  delete selected polygon from memory and screen
 * @author smallchimney
 * @param  _Feedback  RViz event feedback message
 */
void deletePolygon(const EditablePolygonController::Feedback::ConstPtr& _Feedback) {
    assert(l_pController != nullptr);

    l_pController -> m_pServer -> erase(_Feedback -> marker_name);
    auto idx = interactive::EditablePoints::getPolygonIdx(_Feedback -> marker_name);
    l_pController -> m_pPolygonPainter -> erase(idx);
    l_pController -> m_pServer -> applyChanges();

    if(l_pController -> m_mPolygons.find(idx) != l_pController -> m_mPolygons.end()) {
        auto& polygon = l_pController -> m_mPolygons[idx];
        if(polygon)polygon.reset();
        l_pController -> m_mPolygons.erase(l_pController -> m_mPolygons.find(idx));
    }
}

/* ********************************************************************************
 *                                GUI CALLBACKS                                   *
 ******************************************************************************** */

/**
 * @brief  listen for initial polygon, change this to interactive polygon and
 *          wait for edit from RViz. the edited result will output to another topic
 * @author smallchimney
 */
void EditablePolygonController::processMessage(const geometry_msgs::PolygonStampedConstPtr& _Msg) {
    using interactive::EditablePoints;
    using geometry_msgs::Polygon;
    using geometry_msgs::PolygonStamped;

//    size_t idx = _Msg -> header.seq;
    // since the sequence cannot fixed, use menu for fix polygon
    ROS_INFO_STREAM("update for polygon " << m_ulLatestPolygon);

    if(!rviz::validateFloats(_Msg -> polygon.points)) {
        setStatus(rviz::StatusProperty::Error,
                  "Topic",
                  "Message contained invalid floating point values"
                  "(nans or infs)");
        return;
    }
    setStatus(rviz::StatusProperty::Ok, "Topic", "ok");

    if(m_mPolygons.find(m_ulLatestPolygon) != m_mPolygons.end()) {
        m_pServer -> erase(EditablePoints::getInteractiveMarkerName(m_ulLatestPolygon));
        m_mPolygons[m_ulLatestPolygon].reset(new PolygonStamped(*_Msg));
    } else {
        m_mPolygons.emplace(std::make_pair(m_ulLatestPolygon, PolygonStamped::Ptr(new PolygonStamped(*_Msg))));
    }

    auto marker = m_pInteractivePolygon -> getEditablePoints(
            m_ulLatestPolygon, m_pVertexSize -> getFloat(),
            _Msg -> header, _Msg -> polygon.points, getVertexColor());

    if(marker == nullptr) {
        setStatus(rviz::StatusProperty::Warn,
                  "Topic",
                  "Message contained invalid floating point values"
                  "(nans or infs)");
        ROS_WARN("something error during generate interactive polygon object");
        return;
    }
    m_pServer -> insert(*marker);
    m_pServer -> setCallback(marker -> name, &reportMove, Feedback::POSE_UPDATE);
    m_pServer -> setCallback(marker -> name, &reportRelease, Feedback::MOUSE_UP);
    m_pCurrentMenuHandler -> apply(*m_pServer, marker -> name);
    m_pServer -> applyChanges();

    Polygon::ConstPtr polygon(new Polygon(_Msg -> polygon));
    auto color = m_pPolygonColor -> getOgreColor();
    color.a = m_pPolygonAlpha -> getFloat();
    try {
        m_pPolygonPainter -> paint(m_ulLatestPolygon, color, _Msg -> header, polygon);
    } catch(std::runtime_error& ex) {
        setStatus(rviz::StatusProperty::Warn,
                  "Paint",
                  ex.what());
        ROS_WARN_STREAM(ex.what());
    }
}

void EditablePolygonController::updateAllPolygonColor() {
    auto color = m_pPolygonColor -> getOgreColor();
    color.a = m_pPolygonAlpha -> getFloat();
    for(const auto& pair : m_mPolygons) {
        m_pPolygonPainter -> setColor(pair.first, color);
        try {
            m_pPolygonPainter -> paint(pair.first);
        } catch(std::runtime_error& ex) {
            setStatus(rviz::StatusProperty::Warn,
                      "Paint",
                      ex.what());
            ROS_WARN_STREAM(ex.what());
        }
    }
}

void EditablePolygonController::updateVertexColor() {
    using interactive::EditablePoints;
    auto color = getVertexColor();
    visualization_msgs::InteractiveMarker marker;
    for(auto& pair : m_mPolygons) {
        m_pServer -> get(EditablePoints::getInteractiveMarkerName(pair.first), marker);
        for(auto& control : marker.controls)control.markers[0].color = color;
    }
    m_pServer -> insert(marker);
    m_pServer -> applyChanges();
}

void EditablePolygonController::updateVertexSize() {
    using interactive::EditablePoints;
    using geometry_msgs::PolygonStamped;
    auto size = m_pVertexSize -> getFloat();
    visualization_msgs::InteractiveMarker marker;
    for(auto& pair : m_mPolygons) {
        m_pServer -> get(EditablePoints::getInteractiveMarkerName(pair.first), marker);
        for(auto& control : marker.controls) {
            if(control.name == MENU) {
                control.markers[0].scale.x = size;
                continue;
            }
            switch(control.markers.size()) {
                case 2:
                    control.markers[1].scale.z = size / 3.f;
                case 1:
                default:
                    control.markers[0].scale = tf2::toMsg(tf2::Vector3(size, size, 0.01));
                case 0:
                    break;
            }
        }
    }
    m_pServer -> insert(marker);
    m_pServer -> applyChanges();
}

void EditablePolygonController::updateEnableLighting() {
    m_pPolygonPainter -> setLightingEnable(m_pEnableLighting -> getBool());
    m_pPolygonPainter -> refresh();
}

void EditablePolygonController::updateDoubleFace() {
    m_pPolygonPainter -> setDoubleFace(m_pDoubleFace -> getBool());
    m_pPolygonPainter -> refresh();
}


/* ********************************************************************************
 *                        CONSTRUCTION && INITIALIZATION                          *
 ******************************************************************************** */

/**
 * @brief  pluginlib::ClassLoader creates instances
 *          by calling the default constructor
 * @author smallchimney
 */
EditablePolygonController::EditablePolygonController()
        : m_cNh("~"), m_pPolygonPainter(nullptr), m_pInteractivePolygon(nullptr), m_pServer(nullptr),
          m_pCurrentMenuHandler(nullptr), m_pFixedMenuHandler(nullptr), m_pVertexColor(nullptr),
          m_pPolygonColor(nullptr), m_pPolygonAlpha(nullptr), m_pVertexSize(nullptr), m_pEnableLighting(nullptr) {
    m_pPolygonColor.reset(new rviz::ColorProperty(
            "Color", QColor(25, 255, 0),
            "Color to draw the polygons.",
            this, SLOT(updateAllPolygonColor())
    ));
    m_pPolygonAlpha.reset(new rviz::FloatProperty(
            "Alpha", 1.f,
            "Amount of transparency to apply to the polygon.",
            this, SLOT(updateAllPolygonColor())
    ));
    m_pVertexColor.reset(new rviz::ColorProperty(
            "Vertex Color", QColor(255, 255, 0),
            "Color to draw the vertices.",
            this, SLOT(updateVertexColor())
    ));
    // alpha of vertex is fixed to .6f
    m_pVertexSize.reset(new rviz::FloatProperty(
            "Vertex Size", .3f,
            "Radius of the vertex controller.",
            this, SLOT(updateVertexSize())
    ));
    m_pEnableLighting.reset(new rviz::BoolProperty(
            "Enable Lighting", true,
            "Material enable lighting label.",
            this, SLOT(updateEnableLighting())
    ));
    m_pDoubleFace.reset(new rviz::BoolProperty(
            "Double face painting", true,
            "Whether paint the background of the polygon.",
            this, SLOT(updateDoubleFace())
    ));
    m_pPolygonAlpha -> setMax(1.f);
    m_pPolygonAlpha -> setMin(0.f);
    m_pVertexSize -> setMax(1.f);
    m_pVertexSize -> setMin(.1f);

    l_pController = this;
    m_ulLatestPolygon = 0;

    m_cPolygonPub = m_cNh.advertise<geometry_msgs::PolygonStamped>(
            "/chimney_rviz_plugins/editable_polygon/result", 10);
}

/**
 * @brief  free the local point of controller
 * @author smallchimney
 */
EditablePolygonController::~EditablePolygonController() {
    l_pController = nullptr;

    m_pVertexColor.reset();
    m_pPolygonColor.reset();
    m_pPolygonAlpha.reset();
    m_pEnableLighting.reset();

    while(!m_mPolygons.empty()) {
        auto head = m_mPolygons.begin();
        if(head -> second)head -> second.reset();
        m_mPolygons.erase(head);
    }
    m_pServer -> clear();
    m_pServer -> applyChanges();
    m_pServer.reset();
    m_pCurrentMenuHandler.reset();
    m_pFixedMenuHandler.reset();

    m_pPolygonPainter.reset();
    m_pInteractivePolygon.reset();
}

/**
 * @brief  initialize the local fields except the RViz bond setting
 * @author smallchimney
 */
void EditablePolygonController::onInitialize() {
    MessageFilterDisplay::onInitialize();

    m_pPolygonPainter.reset(new visualize::PolygonPainter(
            scene_manager_, scene_node_, context_ -> getFrameManager()));
    m_pInteractivePolygon.reset(new interactive::EditablePoints);
    m_pServer.reset(new Server("chimney_rviz_plugins/editable_polygon"));
    m_pCurrentMenuHandler.reset(new MenuHandler);
    m_pCurrentMenuHandler -> insert("publish", &publishPolygon);
    m_pCurrentMenuHandler -> insert("fix", &fixPolygon);
    m_pFixedMenuHandler.reset(new MenuHandler);
    m_pFixedMenuHandler -> insert("publish", &publishPolygon);
    m_pFixedMenuHandler -> insert("delete", &deletePolygon);

    updateAllPolygonColor();
    updateEnableLighting();
    updateDoubleFace();
    updateVertexColor();
    updateVertexSize();
}

/* ********************************************************************************
 *                               TIMER CALLBACKS                                  *
 ******************************************************************************** */

/* ********************************************************************************
 *                              FUNCTIONAL METHODS                                *
 ******************************************************************************** */

std_msgs::ColorRGBA EditablePolygonController::getVertexColor() {
    std_msgs::ColorRGBA colorMsg;
    auto color = m_pVertexColor -> getOgreColor();
    colorMsg.r = color.r;
    colorMsg.g = color.g;
    colorMsg.b = color.b;
    colorMsg.a = .6f;
    return colorMsg;
}

} // namespace editable_polygon
} // namespace chimney_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(chimney_rviz_plugins::editable_polygon::EditablePolygonController, rviz::Display) // NOLINT(cert-err58-cpp)
