This repository contains the code for the **IEEE INFOCOM 2023** paper: *"[AdaptSLAM: Edge-Assisted Adaptive SLAM with Resource Constraints via Uncertainty Minimization](https://arxiv.org/abs/2301.04620)"* by [Ying Chen](https://sites.duke.edu/marialabyingchen/), [Hazer Inaltekin](https://scholar.google.com.tr/citations?user=yBRPzisAAAAJ&hl=en), and [Maria Gorlatova](https://maria.gorlatova.com/). 

The AdaptSLAM implementation is based on [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) and [Edge-SLAM](https://github.com/droneslab/edgeslam).

## Outline
* [I. Prerequisites](#1)
* [II. Our Testing Setup](#2)
* [III. Running AdaptSLAM in Simulated Settings](#3)
* [IV. Running AdaptSLAM on Client and Server Devices](#4)


#### <span id="1">I. Prerequisites
Setup the [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/README.md) prerequisites.

#### <span id="2">II. Our Testing Setup
  * Dell XPS 8930 desktop with Intel (R) Core (TM) i7-9700K CPU@3.6GHz and NVIDIA GTX 1080 GPU, and a Lenovo Legion 5 laptop (with an AMD Ryzen 7 4800H CPU and an NVIDIA
GTX 1660 Ti GPU) using a virtual machine with 4-core CPUs and 8GB of RAM.
  * Ubuntu 18.04LTS.
  * OpenCV 3.4.2.
  * Eigen3 3.2.10.
 
#### <span id="3">III. Running AdaptSLAM in Simulated Settings
##### 1. Building AdaptSLAM in Simulated Settings
After cloning the repository, build the Thirdparty libraries and AdaptSLAM.
 ```
 cd AdaptSLAM/Sim AdaptSLAM
 chmod +x build.sh
 ./build.sh
 ```
##### 2. Running Examples

a. Download the [EuRoC V102 sequence](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room2/V2_02_medium/V2_02_medium.zip).

b. Execute the following script:
```
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml V102FileDirectory ./Examples/Monocular/EuRoC_TimeStamps/V102.txt dataset-V102_mono
```
Change ```V102FileDirectory``` to the directory where the dataset has been uncompressed. 

#### <span id="4">IV. Running AdaptSLAM on Client and Server Devices
##### 1. Building AdaptSLAM on Client and Server Devices
a. In  ```src/LocalMapping.cc```, modify ```#Input your IP address#```  to your IP address. You can also modify ```#Input your port number#```  to the port number you selected. However, the default port numbers should also work.

b. On both client and server devices, build the Thirdparty libraries and AdaptSLAM.
 ```
 cd AdaptSLAM/Edge-assisted AdaptSLAM
 chmod +x build.sh
 ./build.sh
 ```
 ##### 2. Running Examples

For example, for EuRoC V102 sequence, please execute the following script on both server and client devices
```
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml V102FileDirectory ./Examples/Monocular/EuRoC_TimeStamps/V102.txt dataset-V102_mono
```
Then on the client device, please enter ```client``` when the message in the terminal window ask to ```Enter the run type (client or server)```. On the server device, please enter ```server```.

After this, two TCP connections between the client and server devices will be established.
 # Citation

Please cite the following paper in your publications if the dataset or code helps your research. 

     @inproceedings{Chen23AdaptSLAM,
      title={{AdaptSLAM}: Edge-Assisted Adaptive SLAM with Resource Constraints via Uncertainty Minimization},
      author={Chen, Ying and Inaltekin, Hazer and Gorlatova, Maria},
      booktitle={Proc. IEEE INFOCOM},
      year={2023}
    }
    
# Acknowledgments
The contributors of the code are [Ying Chen](https://sites.duke.edu/marialabyingchen/) and [Maria Gorlatova](https://maria.gorlatova.com/). For questions on this repository or the related paper, please contact Ying Chen at yc383 [AT] duke [DOT] edu.

This work was supported in part by NSF grants CSR1903136, CNS-1908051, and CNS-2112562, NSF CAREER Award IIS-2046072, by an IBM Faculty Award, and by the Australian Research Council under Grant DP200101627.
