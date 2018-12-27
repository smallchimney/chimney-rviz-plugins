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
   * File Name     : traits.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2018-12-20 14:40:25
   * Last Modified : smallchimney
   * Modified Time : 2018-12-20 14:41:08
************************************************************************* */
#ifndef __CHIMNEY_RVIZ_PLUGINS_TRAITS_H__
#define __CHIMNEY_RVIZ_PLUGINS_TRAITS_H__

#include <boost/unordered/unordered_map.hpp>

namespace chimney_rviz_plugins {

    template<typename ValueT>
    struct Type {
        typedef boost::shared_ptr<ValueT> Ptr;
        typedef boost::shared_ptr<ValueT const> ConstPtr;
        typedef boost::unordered_map<size_t, Ptr> PtrMap;
        typedef boost::unordered_map<size_t, ValueT> Map;
    };

}

#endif //__CHIMNEY_RVIZ_PLUGINS_TRAITS_H__
