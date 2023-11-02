# CUDA-CenterPoint
This repository contains sources and model for [CenterPoint](https://arxiv.org/abs/2006.11275) inference using CUDA & TensorRT.
![title](/assets/centerpoint.png)


### Compile && Run

Run detection process on test data.

#### step 1:

```shell
$ cd CUDA-CenterPoint
$ catkin_make ..
$ source devel/setup.bash
$ cd build
$ make -j12
```
#### step 2:

```shell
$ rosrun nus_centerpoint nus_centerpoint 
```
