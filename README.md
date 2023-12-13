# toragi_map_gnss_visualization

## Introduction

- 3次元地図上にGNSS測位結果を表示するパッケージです．
- ROSの機能を使ったシミュレーションが実行可能です．


## Functions
### las_viewer

- シミュレーションに使用する地図データを読み込み，rvizに表示する．

### gnss_viewer

- rosbagに保存されているGNSSの測位データを地図座標に座標変換し，位置結果を出力する．

### simulation_launcher

- 3次元地図上にGNSSの測位結果を表示するシミュレーションを起動する．

## How to Install

### 環境構築

1. toragi_map_gnss_visualizationのクローン

        git clone https://github.com/MapIV/toragi_map_gnss_visualization.git` 

2. 必要なパッケージのインストール（libLAS）


        git clone https://github.com/MapIV/libLAS.git
        cd libLAS
        mkdir -p build && cd build
        cmake -DCMAKE_BUILD_TYPE=Release ..
        make
        sudo make install

    以下の行の実行は任意(クローンしたlibLASフォルダの削除) \
    
        cd ../../ && rm -rf libLAS

### ビルド
    cd toragi_map_gnss_visualization/ros/
    rosdep install --from-paths src --ignore-src -r -y 
    catkin_make -DCMAKE_BUILD_TYPE=Release

## Start Simulation

1. ターミナルを開き roscore を起動する

        roscore

2. 別ターミナルでシミュレーターを起動する

        cd toragi_map_gnss_visualization/ros/
        source devel/setup.bash
        roslaunch simulation_launcher simulation.launch

3. 別ターミナルでROSGABを起動する

        rosbag play simulation_smaple.bag