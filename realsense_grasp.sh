CUDA_VISIBLE_DEVICES=0 python execute_grasp.py \
--checkpoint_path ./checkpoint/checkpoint-rs.tar \
--num_point 80000 \
--source realsense \
--move_robot
