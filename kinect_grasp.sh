CUDA_VISIBLE_DEVICES=0 python execute_grasp.py \
--checkpoint_path ./checkpoint/checkpoint-rs.tar \
--num_point 100000 \
--source kinect \
--move_robot
