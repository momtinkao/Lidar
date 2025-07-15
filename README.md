# How to use the Project
>[!NOTE]
> **PCL VERSION == 1.13.1**    
> **Ubuntu VERSION == 22.04**


### 目錄

- [🛠️ 環境準備與相依套件](#️-環境準備與相依套件)
  - [1. 安裝基礎編譯工具](#1-安裝基礎編譯工具)
  - [2. 安裝PCL相依套件](#2-安裝PCL相依套件)
- [⚙️ 編譯與安裝流程](#️-編譯與安裝流程)
- [✅ 程式使用流程](#-程式使用流程)

---

### 🛠️ 環境準備與相依套件

使用前需要先編譯好PCL Library

#### 1. 安裝基礎編譯工具

更新套件列表，並安裝 `git`、`build-essential` (包含 g++ 編譯器) 和 `cmake`。

```bash
sudo apt update
sudo apt install -y git build-essential cmake
```
#### 2. 安裝PCL相依套件
編譯PCL Library需要先安裝相依套件
```bash
sudo apt install libboost-all-dev libeigen3-dev libflann-dev
sudo apt install libvtk-dev libqhull-dev libopenni2-dev qtbase5-dev libqt5svg5-dev
```
### ⚙️編譯與安裝流程
[按此下載](https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.13.1/source.tar.gz)PCL 1.13.1版本    
將檔案解壓縮
```bash
tar xvf pcl-pcl-1.13.1.tar.gz
```
Enter to Folder and run the cmake and install the library
```bash
cd pcl-pcl-1.13.1 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2
sudo make -j2 install
```

### 程式使用流程
