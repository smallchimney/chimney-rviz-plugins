## editable_polygon

#### Usage
This is the main node for plugin chimney_rviz_plugins/editable_polygon.
It can easily visualize the standard `geometry_msgs::Polygon` and it's vertices as
interactive marker which can be moved in RViz. This means, you input a origin polygon, and
you can edit the polygon whatever you want. The effect of the plugin can be seen as this:  
![image](../../.readme/editable-polygon.gif)  
-----
The following tips is what I think you should know before you use this plugin:  

1. If you want not only visualize the polygons, but also edit it as you want.
You should subscribe the topic "**/chimney_rviz_plugins/editable_polygon/update**"
in RViz after you launch this plugin.
2. If you need to edit a polygon, the first thing you have to do is check whether
the polygon input has been stopped. It's very important because when new polygon come,
all your editing will be discard unless you have fix it before.
3. The current version is not allowed to add vertices number during the editing. This
means if you want a five-pointed star finally, don't input a square as origin.
4. All the vertices can moved only in the X-Y plane, so if want to put a polygon above
something, input it with enough Z-Axis value.

#### Panel Setting
This plugin contain a panel setting, that is mean you can alter the input topic name, the
color of the polygons, vertices' color and size. The vertices will be paint
as a disc, so the size is mean diameter of it, in meters.  
The current version is only support single color for all polygons, I'm planing to add a
rainbow color mode, which will automatically set a unique beautiful color for each polygon.

#### Menu Operate
* publish  
Only edit the polygon is obviously not enough for the plugin need. In most situations,
we edit the polygon because we need the altered polygon's position data. So here we can
right click any one of the vertices for the menu, and select publish to send it to
the topic "**/chimney_rviz_plugins/editable_polygon/result**". I know the menu can only
called from vertices is stupid, but the current version can only support this, maybe
update later.
* fix  
The sequence number of message can not be controlled. It will be terrible when
you want to just refresh one already existed polygon, because you never update, always
add new polygon into the screen. In current version, the count of polygon which can
be edit will never bigger than 1. So, if you want to go on the next polygon, fix the
current one before that.
* delete
There always be some mistakes in out work, if you want delete the wrong polygon and try a
new one, just click it. Maybe publish it before delete? Note that, there is no any undo
function in the current version.
