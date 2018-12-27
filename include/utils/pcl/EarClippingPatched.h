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
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
/* *************************************************************************
   * File Name     : EarClippingPatched.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2018-12-21 18:50:53
   * Last Modified : smallchimney
   * Modified Time : 2018-12-24 17:20:00
************************************************************************* */
#ifndef __CHIMNEY_RVIZ_PLUGINS_EAR_CLIPPING_PATCHED_H__
#define __CHIMNEY_RVIZ_PLUGINS_EAR_CLIPPING_PATCHED_H__

#include <pcl/surface/ear_clipping.h>

namespace pcl {

    /**
     * @brief  this class is total a patch for original EarClipping,
     *         almost all behaviors are same with original.
     * @author smallchimney
     */
    class EarClippingPatched : public EarClipping {

    protected:

        /**
         * @brief  override the pcl::EarClipping::performProcessing()
         *          override this just for call pcl::EarClippingPatched::triangulate()
         * @author smallchimney
         * @param  _Output  output the output polygonal mesh
         */
        void performProcessing(pcl::PolygonMesh& _Output) override;

        /**
         * @brief  override the pcl::EarClipping::triangulate()
         *          override this just for call pcl::EarClippingPatched::isEar()
         * @author smallchimney
         * @param  _Vertices   all the vertices indexes of the polygon in clockwise
         * @param  _Output     the triangles list
         */
        void triangulate(const Vertices& _Vertices, PolygonMesh& _Output);

        /**
         * @brief  override the pcl::EarClipping::isEar()
         * @author smallchimney
         * @param  u             the first vertex of triangle
         * @param  v             the middle vertex of triangle
         * @param  w             the last vertex of triangle
         * @param  vertices      remain vertices to be checked
         * @return               whether the triangle u-v-w is a ear, note that
         *                       the vertex v is the main point to check for
         */
        bool isEar(int u, int v, int w, const std::vector<uint32_t>& vertices);
    };

}

#endif //__CHIMNEY_RVIZ_PLUGINS_EAR_CLIPPING_PATCHED_H__
