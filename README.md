## 安装依赖
### 1、Eigen
            1、通过源码编译安装
            源码下载地址：https://eigen.tuxfamily.org/index.php?title=Main_Page
            推荐选择3.4一下的版本，最好是3.3.X，eigen 3.4以上版本运行时有可能会出现bug。
            然后编译，sudo make install即可，安装的主要库文件有：
            (1)、/usr/local/include/eigen3：库头文件
            (2)、/usr/local/share/eigen3/cmake/Eigen3Config.cmake：cmake的配置文件，用于find_package
            (3)、/usr/local/share/pkgconfig/eigen3.pc：pkg-config的配置文件。
              
            2、操作系统包管理器安装(推荐法一，可以灵活的选择版本，有的库如G2O对Eigen的版本有要求)
            sudo apt install libeigen3-dev
            eigen库被安装在/usr/include/eigen3 中。
            cmake配置文件安装在/usr/lib/cmake/eigen3/Eigen3Config.cmake，这样就可以通过
            find_package(Eigen3 REQUIRED) 去找到eigen3库。

            若想要查看可安装的版本信息，输入命令：apt-cache policy libeigen3-dev
            我的电脑上是3.3.4。
            安装完毕后，通过dpkg -s libeigen3-dev | grep 'Version' 查询安装的版本。
            
### 2、pcl
     sudo apt install libpcl-dev 
### 3、ceres
    安装依赖：
    sudo apt-get install cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
    安装ceres最新的版本， 推荐选择2.0.0版本
    https://github.com/ceres-solver/ceres-solver/tags
    cd ceres-solver-2.0.0
    mkdir build
    cd build
    cmake ..
    make -j4
    make test
    sudo make install
### 3、安装G2O
    sudo apt-get update
    sudo apt-get install libatlas-base-dev libsuitesparse-dev libgoogle-glog-dev libgflags-dev libeigen3-dev
    sudo apt-get install libboost-all-dev
    sudo apt-get install libopencv-dev
    下载地址： https://github.com/RainerKuemmerle/g2o/releases 
    (推荐选择20230223_git及以上版本)
    cd g2o-20230223_git/
    mkdir build
    cd build/
    cmake ..
    make -j8
    sudo make install


