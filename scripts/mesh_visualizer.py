#! /usr/bin/env python
import os
import numpy as np
import rospy
import rospkg
from visualization_msgs.msg import MarkerArray, Marker

if __name__ == '__main__':

    rospy.init_node("mesh_visualizer")
    rate = rospy.Rate(1)

    print('Initializing object visualizer')
    rp = rospkg.RosPack()

    model_path = rospy.get_param("/model_path")
    print("model_path ", model_path)
    model_dir = model_path.split('caric_mission')[-1]

    modelMarkerArray = MarkerArray()
    modelMarkerPub = rospy.Publisher('model_viz', MarkerArray, queue_size=1)

    bbox_path = rospy.get_param("/bounding_box_path")
    print("bounding_box_path ", bbox_path)
    bboxMarkerArray = MarkerArray()
    bboxMarkerPub = rospy.Publisher('bbox_viz', MarkerArray, queue_size=1)

    current_model_count = 0
    current_bbox_count = 0
    while not rospy.is_shutdown():

        # find all model in the model
        files = [ file for file in os.listdir(model_path) if file.endswith(('.dae', '.stl', '.mesh'))]

        # if the number of valid meshed in the 'meshes' folder has changed
        if len(files) != current_model_count:
            
            # if some markers are removed from the 'meshes' folder, delete them in RViz
            if len(files) < current_model_count:
                marker = Marker()
                marker.header.frame_id = 'world'
                # send the DELETEALL marker to delete all marker in RViz
                marker.action = marker.DELETEALL
                modelMarkerArray.markers.append(marker)
                modelMarkerPub.publish(modelMarkerArray)

            current_model_count = len(files)
            for marker_id, file in enumerate(files):
                print('Loading file: %s', file)
                marker = Marker()
                marker.id = marker_id
                marker.mesh_resource = 'package://caric_mission' + model_dir + '/' + file
                marker.mesh_use_embedded_materials = True  # Need this to use textures for mesh
                marker.type = marker.MESH_RESOURCE
                marker.header.frame_id = "world"
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.pose.orientation.w = 1.0
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0            
                modelMarkerArray.markers.append(marker)
        
        modelMarkerPub.publish(modelMarkerArray)

        # find all bounding box in the bounding box path
        files = [ file for file in os.listdir(bbox_path) if file.endswith(('.dae', '.stl', '.mesh'))]
        
        # if the number of valid meshed in the 'meshes' folder has changed
        if len(files) != current_bbox_count:
            
            # if some markers are removed from the 'meshes' folder, delete them in RViz
            if len(files) < current_bbox_count:
                marker = Marker()
                marker.header.frame_id = 'world'
                # send the DELETEALL marker to delete all marker in RViz
                marker.action = marker.DELETEALL
                bboxMarkerArray.markers.append(marker)
                bboxMarkerPub.publish(bboxMarkerArray)

            current_bbox_count = len(files)
            for marker_id, file in enumerate(files):
                print('Loading file: %s', file)
                marker = Marker()
                marker.id = marker_id
                marker.mesh_resource = 'package://caric_mission' + model_dir + '/bounding_boxes/' + file
                marker.mesh_use_embedded_materials = True  # Need this to use textures for mesh
                marker.type = marker.MESH_RESOURCE
                marker.header.frame_id = "world"
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.pose.orientation.w = 1.0
                marker.color.a = 0.3
                marker.color.r = 1.0
                marker.color.g = 0.9
                marker.color.b = 0.0            
                bboxMarkerArray.markers.append(marker)

        bboxMarkerPub.publish(bboxMarkerArray)
        
        rate.sleep()
