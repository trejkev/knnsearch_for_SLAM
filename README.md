# knnsearch_for_SLAM
Script to get the point cloud for both, the ground truth and the slam generated map, and use the knnsearch function to compare them. The script takes as inputs both images, the SLAM generated image has to be named as 'SLAM_Gen_Map.png', and the ground truth image has to be named as 'Ground_Truth.png'.

Constraints:
  1. Both images need to have the same DPI and have the same dimensions.
  2. Both images need to be B/W.
  3. For ideal results, the user has to modify the SLAM_Gen_Map image, so that it is aligned and centered to match with the Ground_Truth image.
