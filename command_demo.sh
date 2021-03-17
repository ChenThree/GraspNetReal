CUDA_VISIBLE_DEVICES=0 python execute_grasp.py \
--checkpoint_path GraspNetToolBox/checkpoint/checkpoint-rs.tar \
--num_point 1000000 \
--source camera \
--move_robot
