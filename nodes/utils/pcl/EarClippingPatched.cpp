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
   * File Name     : EarClippingPatched.cpp
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2018-12-21 18:59:16
   * Last Modified : smallchimney
   * Modified Time : 2018-12-27 16:38:20
************************************************************************* */
#include <utils/pcl/EarClippingPatched.h>

#define __DEBUG__ false

/**
 * @brief  override the pcl::EarClipping::performProcessing()
 *          The actual surface reconstruction method.
 * @author smallchimney
 * @param  _Output  output the output polygonal mesh
 */
void pcl::EarClippingPatched::performProcessing(pcl::PolygonMesh& _Output) {
    _Output.polygons.clear();
    _Output.cloud = input_mesh_ -> cloud;
    for(const auto& polygon : input_mesh_ -> polygons)
        triangulate(polygon, _Output);
#if __DEBUG__
    std::cerr << "===============================================\n"
                 "=========== triangulate finish ================\n"
                 "===============================================\n\n";
#endif
}

void pcl::EarClippingPatched::triangulate(const Vertices& _Vertices, PolygonMesh& _Output) {
    const size_t n_vertices = _Vertices.vertices.size ();

    if(n_vertices < 3)
        return;
    else if(n_vertices == 3)
    {
        _Output.polygons.push_back(_Vertices);
        return;
    }

    std::vector<uint32_t> remaining_vertices(n_vertices);

    // put the clockwise check in the future to fix complex situation's decompose bug
    remaining_vertices = _Vertices.vertices;

    // Avoid closed loops.
    if(remaining_vertices.front () == remaining_vertices.back ())
        remaining_vertices.erase (remaining_vertices.end () - 1);

#if __DEBUG__
    std::cerr << "begin points: (" << remaining_vertices[0] + 1;
    for(size_t idx = 1; idx < remaining_vertices.size(); idx++)
        std::cerr << ", " << remaining_vertices[idx] + 1;
    std::cerr << ")\n-----\n";
#endif

    // null_iterations avoids infinite loops if the polygon is not simple.
    for(int u = static_cast<int>(remaining_vertices.size()) - 1, null_iterations = 0;
         remaining_vertices.size() > 2 && null_iterations < static_cast<int>(remaining_vertices.size() * 2);
         ++null_iterations, u = (u+1) % static_cast<int>(remaining_vertices.size())) {
        int v = (u + 1) % static_cast<int>(remaining_vertices.size());
        int w = (u + 2) % static_cast<int>(remaining_vertices.size());

        if(isEar(u, v, w, remaining_vertices)) {
            Vertices triangle;
            triangle.vertices.resize (3);
            triangle.vertices[0] = remaining_vertices[u];
            triangle.vertices[1] = remaining_vertices[v];
            triangle.vertices[2] = remaining_vertices[w];
            _Output.polygons.push_back(triangle);
            remaining_vertices.erase(remaining_vertices.begin() + v);
            null_iterations = 0;
#if __DEBUG__
            std::cerr << "remain points: (" << remaining_vertices[0] + 1;
            for(size_t idx = 1; idx < remaining_vertices.size(); idx++)
                std::cerr << ", " << remaining_vertices[idx] + 1;
            std::cerr << ")\n-----\n";
#endif
        }
    }
}

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
bool pcl::EarClippingPatched::isEar(int u, int v, int w, const std::vector<uint32_t>& _Vertices) {
    bool ret;
    Eigen::Vector3f p_u, p_v, p_w;
    p_u = points_ -> points[_Vertices[u]].getVector3fMap();
    p_v = points_ -> points[_Vertices[v]].getVector3fMap();
    p_w = points_ -> points[_Vertices[w]].getVector3fMap();

    static const float eps = 1e-15f;
    Eigen::Vector3f p_vu, p_vw;
    p_vu = p_u - p_v;
    p_vw = p_w - p_v;

#if __DEBUG__
    std::cerr << "from " << _Vertices[v] + 1 << "-" << _Vertices[u] + 1 << " to "
            << _Vertices[v] + 1 << "-" << _Vertices[w] + 1 << " is anti-clockwise? "
            << ((p_vu[0] * p_vw[1] - p_vu[1] * p_vw[0] > 0) ? "true" : "false") << std::endl;
    std::cerr << _Vertices[v] + 1 << "-" << _Vertices[u] + 1 << " crossing "
            << _Vertices[v] + 1 << "-" << _Vertices[w] + 1
            << " is " << (p_vu.cross(p_vw)).norm() << std::endl;
#endif

    Eigen::Vector3f cross = p_vu.cross(p_vw);
    // Avoid flat triangles.
    if((cross[2] > 0) != ((p_vu.cross(p_vw)).norm() < eps)) {
        ret = false;
    } else {
        ret = true;
        Eigen::Vector3f p;
        // Check if any other vertex is inside the triangle.
        for(int k = 0; k < static_cast<int>(_Vertices.size()); k++) {
            if ((k == u) || (k == v) || (k == w))continue;
            p = points_->points[_Vertices[k]].getVector3fMap();

            if (isInsideTriangle(p_u, p_v, p_w, p)) {
                if (u + 1 == 513) {
                    std::cerr << _Vertices[k] + 1 << " is in the triangle "
                              << _Vertices[u] + 1 << "-" << _Vertices[v] + 1 << "-"
                              << _Vertices[w] + 1 << std::endl;
                }
                ret = false;
                break;
            }
        }
    }

#if __DEBUG__
    std::cerr << "check for triangle (" <<
        _Vertices[u] + 1 << ", " << _Vertices[v] + 1 << ", " << _Vertices[w] + 1 <<
            "): " << (ret ? "true" : "false") << std::endl;
#endif
    return ret;
}
