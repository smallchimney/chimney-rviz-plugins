## utils

this node contain all base operate of the object,
maybe these patch will be merged into PCL future release. 

#### pcl
This is the patch for the original pcl(version 1.9 and before),
contains the following patch:  
* EarClippingPatched([#2743](https://github.com/PointCloudLibrary/pcl/pull/2743))  
the patch for the original ear-clipping method, since the old one has bad bug that
will mark the concave triangle as ear sometimes.

#### polygon
Provide polygon generic operate such as decompose it to triangles list for visualize
