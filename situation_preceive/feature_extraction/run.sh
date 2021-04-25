
#rosrun ORB_SLAM3 Mono_Inertial /home/xhe/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/xhe/ORB_SLAM3/Examples/Monocular-Inertial/EuRoC.yaml 
pathDatasetEuroc='/home/xhe/ORB_SLAM3/Datasets' #Example, it is necesary to change it by the dataset path


./build/feature_extraction  "$pathDatasetEuroc"/MH01 ./data/MH01.txt 

