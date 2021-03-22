CUDA_VISIBLE_DEVICES=0 python execute_grasp.py \
--checkpoint_path GraspNetToolBox/checkpoint/checkpoint-kn.tar \
--num_point 100000 \
--source kinect \
--move_robot