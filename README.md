# Pointcloud-Data-from-Azure-Kinect-DK-to-Blender
Pointcloud Data from Azure Kinect DK to Blender

# Requirements
	# Python ( + Editor)
		# os
		# numpy
		# open3d
		# cv2
	# Blender
	# bash
	# ffmpeg
	# AzureKinect SDK + Viewer und Recorder
 
 # Creating frames

	# 1. Azure Kinect has to record with the Azure-Kinect-Recorder (k4arecorder.exe) 
	#	to get an matroska.file (the depht video must be coded to b16g)

	# 2. Export the rgb_video and the depth_video from matroska.file via MVKToolNix

	# 3. Convert each video to frames with bash and ffmpeg (depth frames need to be in grayscale b16g)
	
	
		# 3.1 Extract RGB Frames in Bash
	
		ffmpeg -i rgb_video.mp4 -vsync 0 rgb_frames/frame_%04d.png
	
		# 3.2 Extract depth Frames in Bash
	
		ffmpeg -i depth_video.mp4 -vsync 0 depth_frames/frame_%04d.png
	
	 # Note: the video-files can also be .mkv 
	
# Creating pointcloud	
	
	# 4. Run this script in Python to create a .ply for every frame
	
		import os
		import numpy as np
		import open3d as o3d
		import cv2
		
		# Camera intrinsic parameters
		fx = 525.0  # Focal length in x axis
		fy = 525.0  # Focal length in y axis
		cx = 319.5  # Optical center in x axis
		cy = 239.5  # Optical center in y axis
		depth_scale = 1000.0  # Scaling factor for depth values
		
		def create_point_cloud_from_rgb_depth(rgb_image, depth_image):
		    # Get image dimensions
		    h, w = depth_image.shape
		    
		    # Create meshgrid for pixel coordinates
		    u = np.arange(w)
		    v = np.arange(h)
		    u, v = np.meshgrid(u, v)
		    
		    # Flatten arrays
		    u = u.flatten()
		    v = v.flatten()
		    depth = depth_image.flatten() / depth_scale
		    
		    # Filter out invalid depth values
		    valid = depth > 0
		    u = u[valid]
		    v = v[valid]
		    depth = depth[valid]
		    
		    # Project to 3D space
		    z = depth
		    x = (u - cx) * z / fx
		    y = (v - cy) * z / fy
		    
		    # Stack into an Nx3 array
		    points = np.stack((x, y, z), axis=1)
		    
		    # Get corresponding RGB values
		    colors = rgb_image[v, u, :] / 255.0  # Normalize to [0, 1]
		    
		    return points, colors
		
		def save_point_cloud(points, colors, output_path):
		    point_cloud = o3d.geometry.PointCloud()
		    point_cloud.points = o3d.utility.Vector3dVector(points)
		    point_cloud.colors = o3d.utility.Vector3dVector(colors)
		    o3d.io.write_point_cloud(output_path, point_cloud)
		
		def process_frames(rgb_folder, depth_folder, output_folder):
		    # Ensure output folder exists
		    os.makedirs(output_folder, exist_ok=True)
		    
		    rgb_files = sorted([f for f in os.listdir(rgb_folder) if f.endswith('.png') or f.endswith('.jpg')])
		    depth_files = sorted([f for f in os.listdir(depth_folder) if f.endswith('.png') or f.endswith('.jpg')])
		    
		    for rgb_file, depth_file in zip(rgb_files, depth_files):
		        # Load images
		        rgb_image = cv2.imread(os.path.join(rgb_folder, rgb_file))
		        depth_image = cv2.imread(os.path.join(depth_folder, depth_file), cv2.IMREAD_UNCHANGED)
		        
		        # Create point cloud
		        points, colors = create_point_cloud_from_rgb_depth(rgb_image, depth_image)
		        
		        # Save point cloud
		        output_path = os.path.join(output_folder, os.path.splitext(rgb_file)[0] + '.ply')
		        save_point_cloud(points, colors, output_path)
		        print(f'Saved point cloud: {output_path}')
		
		# Example usage
		rgb_folder = '/path/to/rgb_frames'
		depth_folder = '/path/to/depth_frames'
		output_folder = '/path/to/output_pointclouds'
		
		process_frames(rgb_folder, depth_folder, output_folder)
	 
		# Note: you only have to define the paths under Example usage

 # to Blender
	# 5. Create a new Collection in Blender and import every .ply into it
	
	# 6. Run this script in Blender to create a key-frame-animation that enables every .ply-frame 
	#	at the corresponding Blender-Timeline-frame (for Viewport and Render)
		
		import bpy
		
		# Replace "CollectionName" with the name of the collection containing your point cloud objects
		collection_name = "CollectionName"
		
		# List of point cloud objects
		point_cloud_objects = bpy.data.collections.get(collection_name)
		
		if point_cloud_objects:
		    # Set up animation
		    frame_num = 1
		    for obj in point_cloud_objects.objects:
		        # Ensure the object is initially hidden
		        obj.hide_viewport = True
		        obj.hide_render = True
		        obj.keyframe_insert(data_path="hide_viewport", frame=frame_num - 1)
		        obj.keyframe_insert(data_path="hide_render", frame=frame_num - 1)
		        
		        # Make the object visible for its corresponding frame
		        obj.hide_viewport = False
		        obj.hide_render = False
		        obj.keyframe_insert(data_path="hide_viewport", frame=frame_num)
		        obj.keyframe_insert(data_path="hide_render", frame=frame_num)
		        
		        # Hide the object again in the next frame
		        obj.hide_viewport = True
		        obj.hide_render = True
		        obj.keyframe_insert(data_path="hide_viewport", frame=frame_num + 1)
		        obj.keyframe_insert(data_path="hide_render", frame=frame_num + 1)
		        
		        frame_num += 1
		
		    # Set end frame
		    bpy.context.scene.frame_end = len(point_cloud_objects.objects)
		else:
		    print("Collection '{}' not found.".format(collection_name))
	
	# 7. done
	
# Notes
	# 	I tried to export this as an alembic file, which works but still treats 
	#		the whole file as one Collection (at least in Blender)
 
 	#	Make sure that you have the same amount of Depth- and RGB-frames, else the point-color will be off
  
	#       If you want to asign a material to a whole collection you first need to create it for every single .ply 
	#		in the collection (same for Geo-node material)
 
	#       Maybe there is a fix with a plugin I don't know yet, so pls hit me up If you have any ideas :)
 
	#       I created this workflow with ChatGPT, because I don't have any coding knowledge- There deffinetly is a 
	#		better way of doing this, so If you have one pls share it
