## polygon_painter

This node is used for visualize polygon on the screen.

#### Polygon Painter
It is used for paint polygon on the Ogre's scene.
* Ogre context  
It must be initialized within a Ogre context, such as rviz::MessageFilterDisplay.
* Polygon ID  
The each polygon it paint will bind with a unique ID(type of `size_t`),
if the new polygon come with the same ID, the old one will be discard and
cleaned from the scene.
* Polygon color  
Each polygon can have unique color, which can be set in either paint() or setColor().
The following tips is what I think you should know before using this node:  
  1. The polygon that paint before setting color will automatically paint with the
default color(green with alpha value: 0.4).
  2. The setColor() will NOT refresh the scene, maybe call paint() or refresh() after that.
* Material  
The following attributes can be adjust in current version:  
  * lighting_enable
  * double_face_painting

  Also, the setter methods will NOT paint refresh the scene, maybe call
paint() or refresh() after that.  
* Reset  
The reset method is obviously build for RViz's GUI button, haha.
All the scenes will be reset and all polygons will be cleaned.
But the scenes and it's objects will not be destroyed 'cause the computing cost.

#### PolygonMarker
It is used for generate polygon marker to visualize in native RViz way.
The following marker can be generated in current version:  
* Polygon border
* Polygon area
