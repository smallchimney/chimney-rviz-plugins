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
   * File Name     : PolygonPainter.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2018-12-17 13:42:53
   * Last Modified : smallchimney
   * Modified Time : 2018-12-26 19:05:13
************************************************************************* */
#ifndef __CHIMNEY_RVIZ_PLUGINS_POLYGON_PAINTER_H__
#define __CHIMNEY_RVIZ_PLUGINS_POLYGON_PAINTER_H__

#include <traits.h>
#include <utils/polygon/PolygonUtils.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>

namespace chimney_rviz_plugins {
namespace visualize {

    class PolygonPainter {
    public:
        /**
         * @brief  initialize the visualize utility
         * @author smallchimney
         * @param  _SceneManager  scene manager to create and destroy objects
         * @param  _ParentNode    only base on parent scene, we can draw objects in RViz
         * @param  _FrameManager  frame manager to get transform from target frame to fixed frame
         */
        explicit PolygonPainter(Ogre::SceneManager* _SceneManager,
                Ogre::SceneNode* _ParentNode, rviz::FrameManager* _FrameManager);

        /**
         * @brief  reset all resource, clear all objects in the scene, free all malloc space
         * @author smallchimney
         */
        virtual ~PolygonPainter();

        PolygonPainter() = delete;

    public:
        /**
         * @brief  actual painting method implement
         * @author smallchimney
         * @param  _Idx   the polygon index, global unique in this topic
         * @return        if there is anything wrong during the setting
         *                 updating, this method will return {@code false}
         */
        bool paint(size_t _Idx) noexcept(false);

        bool paint(size_t _Idx, const std_msgs::Header& _Header,
                   const geometry_msgs::PolygonConstPtr& _Polygon) noexcept(false);

        bool paint(size_t _Idx, const Ogre::ColourValue& _Color,
                   const std_msgs::Header& _Header,
                   const geometry_msgs::PolygonConstPtr& _Polygon) noexcept(false);

        /**
         * @brief  repaint all polygon that has benn changed after last paint
         * @author smallchimney
         * @return if there is anything wrong during the setting
         *          updating, this method will return {@code false}
         */
        bool refresh();

        const static Ogre::ColourValue DEFAULT_COLOR;

        bool setColor(size_t _Idx, const Ogre::ColourValue& _Color);

        /**
         * @brief  set the lighting enable label for ogre
         * @author smallchimney
         * @param  _EnableLighting  enable lighting
         * @return                  if there is anything wrong during the setting
         *                           updating, this method will return {@code false}
         */
        bool setLightingEnable(bool _EnableLighting);

        /**
         * @brief  set whether to paint backend of the polygon
         * @author smallchimney
         * @param  _DoubleFacePainting  whether to paint backend of the polygon
         * @return                      if there is anything wrong during the setting
         *                               updating, this method will return {@code false}
         */
        bool setDoubleFace(bool _DoubleFacePainting);

        /**
         * @brief  erase the polygon with specific index
         * @author smallchimney
         * @param  _Idx  the polygon index, global unique in this topic
         */
        void erase(size_t _Idx);

        /**
         * @brief  clear all visible objects, reset all polygons stamped and changed labels
         * @author smallchimney
         */
        void reset();

    protected:
        typedef Type<Ogre::SceneManager>::Ptr       SceneManagerPtr;
        typedef Type<rviz::FrameManager>::Ptr       FrameManagerPtr;
        typedef util::PolygonUtils::TriangleList    TriangleList;

    private:

        const FrameManagerPtr m_pFrameManager;
        const SceneManagerPtr m_pSceneManager;
        const Type<Ogre::SceneNode>::Ptr m_pBaseScene;

        bool m_bEnableLighting, m_bDoubleFace;

        Type<Ogre::ManualObject>::PtrMap  m_mManualObjects{};
        Type<Ogre::SceneNode>::PtrMap     m_mSceneNodes{};
        Type<Ogre::MaterialPtr>::Map      m_mMaterials{};
        Type<rviz::BillboardLine>::PtrMap m_mBorders{};
        Type<Ogre::ColourValue>::PtrMap   m_mColors{};
        Type<TriangleList>::PtrMap        m_mPolygons{};
        Type<std_msgs::Header>::PtrMap    m_mHeaders{};
        Type<bool>::Map                   m_mChanged{};

        /**
         * @brief  decompose polygon into triangles array, update the polygons map
         * @author smallchimney
         * @param  _Idx      the polygon index, global unique in this topic
         * @param  _Header   the fixed frame and timestamp
         * @param  _Polygon  the latest polygon data
         */
        void updatePolygon(size_t _Idx,
                           const std_msgs::Header& _Header,
                           const geometry_msgs::PolygonConstPtr& _Polygon);

        /**
         * @brief  prepare for new polygon, will do nothing
         *         if the polygon has been initialized before
         * @author smallchimney
         * @param _Idx  the polygon index, global unique in this topic
         */
        void prepare(size_t _Idx);

        /**
         * @brief  'cause the material should update with color, and prepare() should not
         *          be called when only set color. so update material before each time paint.
         * @author smallchimney
         * @param _Idx  the polygon index, global unique in this topic
         */
        void updateMaterial(size_t _Idx);

        /**
         * @brief  set the position and orientation for Ogre
         * @author smallchimney
         * @param  _Header       the header stand for the ros frame and position timestamp
         * @param  _Position     Ogre position to be set
         * @param  _Orientation  Ogre orientation to be set
         * @return               whether transform is valid
         */
        bool getTransform(const std_msgs::Header& _Header,
                Ogre::Vector3& _Position, Ogre::Quaternion& _Orientation) noexcept(false);

        /**
         * @brief  simply set changed label for the polygon with specific ID
         * @author smallchimney
         * @param  _Idx  the polygon index, global unique in this topic
         */
        void changed(size_t _Idx) {
            m_mChanged[_Idx] = true;
        }

    };

} // namespace visualize
} // namespace chimney_rviz_plugins

#endif //__CHIMNEY_RVIZ_PLUGINS_POLYGON_PAINTER_H__
