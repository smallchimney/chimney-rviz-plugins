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
   * File Name     : PolygonPainter.cpp
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2018-12-17 13:42:30
   * Last Modified : smallchimney
   * Modified Time : 2018-12-26 20:46:13
************************************************************************* */
#include <polygon_painter/PolygonPainter.h>

#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace chimney_rviz_plugins {
namespace visualize {

const Ogre::ColourValue PolygonPainter::DEFAULT_COLOR(0.f, 1.f, 0.f, .4f); // NOLINT(cert-err58-cpp)

/* ********************************************************************************
 *                        CONSTRUCTION && INITIALIZATION                          *
 ******************************************************************************** */

/**
 * @brief  initialize the visualize utility
 * @author smallchimney
 * @param  _SceneManager  scene manager to create and destroy objects
 * @param  _ParentNode    only base on parent scene, we can draw objects in RViz
 * @param  _FrameManager  frame manager to get transform from target frame to fixed frame
 */
PolygonPainter::PolygonPainter(Ogre::SceneManager* _SceneManager,
        Ogre::SceneNode* _ParentNode, rviz::FrameManager* _FrameManager)
        : m_pFrameManager(_FrameManager), m_pSceneManager(_SceneManager), m_pBaseScene(_ParentNode) {}

/**
 * @brief  reset all resource points, destroy all objects in the scene, free all malloc memory
 * @author smallchimney
 */
PolygonPainter::~PolygonPainter() {
    while(!m_mBorders.empty()) {
        auto head = m_mBorders.begin();
        if(head -> second)head -> second.reset();
        m_mBorders.erase(head);
    }
    while(!m_mMaterials.empty()) {
        auto head = m_mMaterials.begin();
        head -> second -> unload();
        Ogre::MaterialManager::getSingleton()
                .remove(head -> second -> getName());
        head -> second.setNull();
        m_mMaterials.erase(head);
    }
    while(!m_mManualObjects.empty()) {
        auto object = m_mManualObjects.begin();
        auto scene  = m_mSceneNodes.find(object -> first);
        if(object -> second) {
            m_pSceneManager -> destroyManualObject(&*(object -> second));
            object -> second.reset();
        }
        if(scene  -> second) {
            m_pSceneManager -> destroySceneNode(&*(scene -> second));
            scene  -> second.reset();
        }
        m_mManualObjects.erase(object);
        m_mSceneNodes.erase(scene);
    }
    while(!m_mColors.empty()) {
        auto head = m_mColors.begin();
        if(head -> second)head -> second.reset();
        m_mColors.erase(head);
    }
    while(!m_mPolygons.empty()) {
        auto head = m_mPolygons.begin();
        if(head -> second) {
            head -> second -> clear();
            head -> second.reset();
        }
        m_mPolygons.erase(head);
    }
    while(!m_mHeaders.empty()) {
        auto head = m_mHeaders.begin();
        head -> second.reset();
        m_mHeaders.erase(head);
    }
    m_mChanged.clear();
}

/* ********************************************************************************
 *                               SERVICE FUNCTION                                 *
 ******************************************************************************** */

bool PolygonPainter::paint(const size_t _Idx, const Ogre::ColourValue& _Color,
                           const std_msgs::Header& _Header,
                           const geometry_msgs::PolygonConstPtr& _Polygon) noexcept(false) {
    if(!setColor(_Idx, _Color))return false;
    return paint(_Idx, _Header, _Polygon);
}

bool PolygonPainter::paint(const size_t _Idx,
                           const std_msgs::Header& _Header,
                           const geometry_msgs::PolygonConstPtr& _Polygon) noexcept(false) {
    // if(!rviz::validateFloats(_Polygon -> polygon.points))return false;
    // please do polygon point validate before the method called
    prepare(_Idx);
    updatePolygon(_Idx, _Header, _Polygon);
    return paint(_Idx);
}

/**
 * @brief  actual painting method implement
 * @author smallchimney
 * @param  _Idx   the polygon index, global unique in this topic
 * @return        if and only if the polygon is repaint successfully,
 *                this method will return {@code true}
 */
bool PolygonPainter::paint(const size_t _Idx) noexcept(false) {
    if(m_mChanged.find(_Idx) == m_mChanged.end() || !m_mChanged[_Idx])return false;
    updateMaterial(_Idx);
    m_mChanged[_Idx] = false;
    Ogre::Vector3 position{};
    Ogre::Quaternion orientation{};
    if(!getTransform(*m_mHeaders[_Idx], position, orientation))return false;

    auto manualObj = m_mManualObjects[_Idx];
    auto sceneNode = m_mSceneNodes[_Idx];
    auto polygon = m_mPolygons[_Idx];
    auto color = *m_mColors[_Idx];

    sceneNode -> setPosition(position);
    sceneNode -> setOrientation(orientation);
    manualObj -> clear();
    manualObj -> setVisible(true);
    manualObj -> estimateVertexCount(polygon -> size() * (m_bDoubleFace ? 6 : 3));
    manualObj -> begin(m_mMaterials[_Idx] -> getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    for(const auto& triangle : *polygon) {
        for(int i = 0; i < 3; i++) {
            auto vertex = triangle[i];
            manualObj -> position(vertex.x, vertex.y, vertex.z);
            manualObj -> colour(color);
        }
        if(m_bDoubleFace) {
            for (int i = 2; i >= 0; i--) {
                auto vertex = triangle[i];
                manualObj -> position(vertex.x, vertex.y, vertex.z);
                manualObj -> colour(color);
            }
        }
    }
    manualObj -> end();
    return true;
}


/**
 * @brief  repaint all polygon that has benn changed after last paint
 * @author smallchimney
 * @return if there is anything wrong during the setting
 *          updating, this method will return {@code false}
 */
bool PolygonPainter::refresh() {
    bool ret = true;
    for(const auto& pair : m_mManualObjects) {
        if(!paint(pair.first))ret = false;
    }
    return ret;
}

bool PolygonPainter::setColor(size_t _Idx, const Ogre::ColourValue& _Color) {
    typedef Ogre::ColourValue Color;
    if(m_mColors.find(_Idx) != m_mColors.end()) {
        // each color setting should be individual, so make a new color instance
        m_mColors.at(_Idx) = Type<Color>::Ptr(new Color(_Color));
    } else {
        m_mColors.emplace(std::make_pair(_Idx, Type<Color>::Ptr(new Color(_Color))));
    }
    changed(_Idx);
    return true;
}

/**
 * @brief  set the lighting enable label for OGRE
 * @author smallchimney
 * @param  _EnableLighting  enable lighting
 * @return                  if there is anything wrong during the setting
 *                           updating, this method will return {@code false}
 */
bool PolygonPainter::setLightingEnable(const bool _EnableLighting) {
    if(m_bEnableLighting == _EnableLighting)return true;
    m_bEnableLighting = _EnableLighting;
    for(auto& pair : m_mPolygons) {
        m_mMaterials[pair.first] -> getTechnique(0)->setLightingEnabled(m_bEnableLighting);
        changed(pair.first);
    }
    return true;
}

/**
 * @brief  set whether to paint backend of the polygon
 * @author smallchimney
 * @param  _DoubleFacePainting  whether to paint backend of the polygon
 * @return                      if there is anything wrong during the setting
 *                               updating, this method will return {@code false}
 */
bool PolygonPainter::setDoubleFace(bool _DoubleFacePainting) {
    if(m_bDoubleFace == _DoubleFacePainting)return true;
    m_bDoubleFace = _DoubleFacePainting;
    for(auto& pair : m_mPolygons) {
        changed(pair.first);
    }
    return true;
}

/**
 * @brief  erase the polygon with specific index
 * @author smallchimney
 * @param  _Idx  the polygon index, global unique in this topic
 */
void PolygonPainter::erase(size_t _Idx) {
    if(m_mManualObjects.find(_Idx) != m_mManualObjects.end())
        m_mManualObjects[_Idx] -> clear();
    if(m_mPolygons.find(_Idx) != m_mPolygons.end()) {
        auto& polygon = m_mPolygons[_Idx];
        if(polygon) {
            polygon -> clear();
            polygon.reset();
        }
        m_mPolygons.erase(m_mPolygons.find(_Idx));
    }
    if(m_mHeaders.find(_Idx) != m_mHeaders.end()) {
        auto& header = m_mHeaders[_Idx];
        if(header)header.reset();
        m_mHeaders.erase(m_mHeaders.find(_Idx));
    }
    m_mChanged.erase(m_mChanged.find(_Idx));
}

/**
 * @brief  clear all visible objects, reset all polygons stamped
 *         and changed labels.</br>
 *         Note that the manual objects and scenes in Ogre will
 *         not be destroyed 'cause decrease compute cost.
 * @author smallchimney
 */
void PolygonPainter::reset() {
    for(auto& pair : m_mManualObjects) {
        pair.second -> clear();
    }
    while(!m_mPolygons.empty()) {
        auto head = m_mPolygons.begin();
        if(head -> second) {
            head -> second -> clear();
            head -> second.reset();
        }
        m_mPolygons.erase(head);
    }
    while(!m_mHeaders.empty()) {
        auto head = m_mHeaders.begin();
        if(head -> second)head -> second.reset();
        m_mHeaders.erase(head);
    }
    m_mChanged.clear();
}

/* ********************************************************************************
 *                              FUNCTIONAL METHODS                                *
 ******************************************************************************** */

/**
 * @brief  decompose polygon into triangles array, update the polygons map
 * @author smallchimney
 * @param  _Idx      the polygon index, global unique in this topic
 * @param  _Header   the fixed frame and timestamp
 * @param  _Polygon  the latest polygon data
 */
void PolygonPainter::updatePolygon(const size_t _Idx,
        const std_msgs::Header& _Header, const geometry_msgs::PolygonConstPtr& _Polygon) {
    using std_msgs::Header;
    using util::PolygonUtils;
    if(m_mPolygons.find(_Idx) != m_mPolygons.end()) {
        m_mPolygons.at(_Idx) = PolygonUtils::decomposeToTriangles(*_Polygon);
        m_mHeaders.at(_Idx) = Type<Header>::Ptr(new Header(_Header));
    } else {
        m_mPolygons.emplace(std::make_pair(_Idx, PolygonUtils::decomposeToTriangles(*_Polygon)));
        m_mHeaders.emplace(std::make_pair(_Idx, Type<Header>::Ptr(new Header(_Header))));
    }
    changed(_Idx);
}

/**
 * @brief  'cause the material should update with color, and prepare() should not
 *          be called when only set color. Since there may be NO material for a new object,
 *          so update material before each time paint.
 * @author smallchimney
 * @param _Idx  the polygon index, global unique in this topic
 */
void PolygonPainter::updateMaterial(const size_t _Idx) {
    const auto& color = *m_mColors[_Idx];
    auto technique = m_mMaterials[_Idx] -> getTechnique(0);
    technique -> setAmbient(color * 0.5);
    technique -> setDiffuse(color);
    if (color.a < 0.9998) {
        technique -> setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        technique -> setDepthWriteEnabled(false);
    }
    else {
        technique -> setSceneBlending(Ogre::SBT_REPLACE);
        technique -> setDepthWriteEnabled(true);
    }
}

/**
 * @brief  prepare for new polygon, will do nothing
 *         if the polygon has been initialized before
 * @author smallchimney
 * @param _Idx  the polygon index, global unique in this topic
 */
void PolygonPainter::prepare(const size_t _Idx) {
    typedef Type<Ogre::SceneNode>::Ptr SceneNodePtr;
    typedef Type<Ogre::ManualObject>::Ptr ManualObjectPtr;
    typedef Type<rviz::BillboardLine>::Ptr BillboardLinePtr;

    if(m_mManualObjects.find(_Idx) != m_mManualObjects.end()) {
        if(m_mColors.find(_Idx) == m_mColors.end())setColor(_Idx, DEFAULT_COLOR);
        return;
    }

    // method "[]" will automatically initialize a SceneNodeConstPtr, so don't use that
    m_mSceneNodes.emplace(std::make_pair(_Idx, SceneNodePtr(m_pBaseScene -> createChildSceneNode())));

    std::stringstream name;
    name << "PolygonMaterial" << _Idx;
    auto materialPtr = Ogre::MaterialManager::
            getSingleton().create(name.str(), "rviz");
    materialPtr -> setReceiveShadows(false);
    materialPtr -> getTechnique(0) -> setLightingEnabled(m_bEnableLighting);
    materialPtr -> getTechnique(0) -> setAmbient(0.5, 0.5, 0.5);
    m_mMaterials.emplace(std::make_pair(_Idx, materialPtr));

    m_mBorders.emplace(std::make_pair(_Idx, BillboardLinePtr(
            new rviz::BillboardLine(&*m_pSceneManager, &*m_mSceneNodes[_Idx]))));

    ManualObjectPtr manualObjectPtr(m_pSceneManager -> createManualObject());
    manualObjectPtr -> setDynamic(true);
    m_mSceneNodes[_Idx] -> attachObject(&*manualObjectPtr);
    m_mManualObjects.emplace(std::make_pair(_Idx, manualObjectPtr));
    if(m_mColors.find(_Idx) == m_mColors.end())setColor(_Idx, DEFAULT_COLOR);
}

/**
 * @brief  set the position and orientation for Ogre
 * @author smallchimney
 * @param  _Header       the header stand for the ros frame and position timestamp
 * @param  _Position     Ogre position to be set
 * @param  _Orientation  Ogre orientation to be set
 * @return               whether transform is valid
 */
bool PolygonPainter::getTransform(const std_msgs::Header& _Header,
        Ogre::Vector3& _Position, Ogre::Quaternion& _Orientation) noexcept(false) {
    bool ok =  m_pFrameManager ->
            getTransform(_Header, _Position, _Orientation);
    if(!ok) {
        std::ostringstream oss;
        oss << "Error transforming from frame '";
        oss << _Header.frame_id << "' to frame '";
        oss << m_pFrameManager -> getFixedFrame() << "'";
        ROS_DEBUG_STREAM(oss.str());
        throw std::runtime_error(oss.str());
    }
    return ok;
}

/* ********************************************************************************
 *                                LOCAL METHODS                                   *
 ******************************************************************************** */

} // namespace visualize
} // namespace chimney_rviz_plugins
