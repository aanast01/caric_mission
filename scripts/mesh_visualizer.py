#! /usr/bin/env python3
import os
import rospy
import rospkg
from visualization_msgs.msg import MarkerArray, Marker

rospy.init_node("mesh_visualizer")
rate = rospy.Rate(1)

rospy.loginfo('Initializing object visualizer')
rp = rospkg.RosPack()

visualizer_path = rospy.get_param("/model_path")
print("model_path ", visualizer_path)

markerArray = MarkerArray()

publisher = rospy.Publisher('visualization_marker', MarkerArray, queue_size=1)

current_file_count = 0
while not rospy.is_shutdown():

    # find all meshes in the 'meshes' folder
    files = [ file for file in os.listdir(visualizer_path) \
                   if file.endswith(('.dae', '.stl', '.mesh'))]
    # if the number of valid meshed in the 'meshes' folder has changed
    if len(files) != current_file_count:
        # if some markers are removed from the 'meshes' folder, delete them in RViz
        if len(files) < current_file_count:
            marker = Marker()
            marker.header.frame_id = 'world'
            # send the DELETEALL marker to delete all marker in RViz
            marker.action = marker.DELETEALL
            markerArray.markers.append(marker)
            publisher.publish(markerArray)

        current_file_count = len(files)
        for marker_id, file in enumerate(files):
            rospy.loginfo('Loading file: %s', file)
            marker = Marker()
            marker.id = marker_id
            marker.mesh_resource = 'package://caric_mission/models/mbs/' + file
            marker.mesh_use_embedded_materials = True  # Need this to use textures for mesh
            marker.type = marker.MESH_RESOURCE
            marker.header.frame_id = "world"
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.pose.orientation.w = 1.0
            markerArray.markers.append(marker)

    # rospy.loginfo('Published %d objects. ', len(markerArray.markers))
    publisher.publish(markerArray)
    rate.sleep()
