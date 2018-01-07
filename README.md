# VAMR-project
visual odometry pipeline by Samuel Nyffenegger and Sebastian Ratz
Some of the code is curtesy of the VAMR lecture at ETH, 2018.

####################
DEPENDENCIES
####################
Matlab R2017b 9.3.0
Computer Vision System Toolbox
Optimization Toolbox

###################
HWO TO RUN
###################
1) Place the data and this repository in a folder corresponding to this structure and the following folder names:

$BASE_DIRECTORY
                /[this repository]
                /data
                      /kitti
                      /malaga
                      /parking

Thus the data folder (with this exact name, containing the dataset folders) needs to be on the same level as the folder that contains this readme file.

2) open general/param.m
3) first line, select the desired dataset by changing the variable "ds". (See comments for more)
4) open main.m
5) hit run.


###################
HOW TO PLAY AROUND
###################
- feel free to change any of the parameters in general/param.m
- It is also possible to switch on and off different features of the VO pipeline

###################
SPECIFICATION OF THE MACHINE
###################
- Proceccor: 	2.2 GHz Intel Core i7
- Memory: 	16 GB 1600 MHz DDR3
- no parallel computing (threads = 1)
- no GPU computing
- average computation time per frame = 2 seconds

