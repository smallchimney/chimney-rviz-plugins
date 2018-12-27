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
   * File Name     : PolygonUtils.cpp
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2018-12-25 14:30:37
   * Last Modified : smallchimney
   * Modified Time : 2018-12-25 14:40:53
************************************************************************* */
#include <utils/polygon/PolygonUtils.h>

#include <utils/pcl/EarClippingPatched.h>

namespace chimney_rviz_plugins {
namespace util {

/**
  * @brief  decompose polygon into triangles array for visualize
  * @author smallchimney
  * @param  _Polygon  the original polygon in 3D
  * @return           the array of triangles decomposed from polygon
  */
PolygonUtils::TriangleListPtr PolygonUtils::decomposeToTriangles(
        const geometry_msgs::Polygon& _Polygon) noexcept(false) {
    TriangleListPtr ret(new TriangleList);
    const auto& vertices = _Polygon.points;
    if(vertices.size() == 3) {
        ret -> emplace_back(Triangle{
                vertices[0],
                vertices[1],
                vertices[2]
        });
        return ret;
    }
    if(vertices.size() < 3)throw std::runtime_error("vertices count is less than 3");

    pcl::PolygonMesh::Ptr input_mesh(new pcl::PolygonMesh);
    pcl::PointCloud<pcl::PointXYZ> mesh_pcl_cloud;
    mesh_pcl_cloud.points.resize(vertices.size());
    for (size_t i = 0; i < vertices.size(); i++) {
        const auto& v = vertices[i];
        mesh_pcl_cloud.points[i] = {v.x, v.y, v.z};
    }
    mesh_pcl_cloud.height = 1;
    mesh_pcl_cloud.width = static_cast<uint32_t>(mesh_pcl_cloud.points.size());
    std::vector<pcl::Vertices> mesh_vertices(1);
    for (size_t i = 0; i < vertices.size(); i++) {
        mesh_vertices[0].vertices.emplace_back(i);
    }

    pcl::PCLPointCloud2 mesh_cloud;
    pcl::toPCLPointCloud2<pcl::PointXYZ>(mesh_pcl_cloud, mesh_cloud);
    input_mesh -> polygons = mesh_vertices;
    input_mesh -> cloud = mesh_cloud;

    pcl::EarClippingPatched clip;
    clip.setInputMesh(input_mesh);
    pcl::PolygonMesh output;
    clip.process(output);
    if(output.polygons.empty())
        throw std::runtime_error("decompose to triangles failed");
    // convert to triangle list instances
    for(const auto &polygon : output.polygons) {
        const auto& p1 = mesh_pcl_cloud.points[polygon.vertices[0]];
        const auto& p2 = mesh_pcl_cloud.points[polygon.vertices[1]];
        const auto& p3 = mesh_pcl_cloud.points[polygon.vertices[2]];
        ret -> emplace_back(Triangle{
                transform(p1),
                transform(p2),
                transform(p3)
        });
    }
    size_t  t = 1;
    ROS_DEBUG_STREAM("====================================================" );
    for(const auto& triangle : *ret) {
        size_t i = 1;
        ROS_DEBUG_STREAM("---------------------- TRIANGLE " << t++ << " ----------------------" );
        for (const auto &point : triangle)
            ROS_DEBUG_STREAM("P" << i++ << ": (" << point.x << ", " << point.y << ", " << point.z << ")");
    }
    return ret;
}

} // namespace util
} // namespace chimney_rviz_plugins
