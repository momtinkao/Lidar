# How to use the Project
>[!NOTE]
> **PCL VERSION == 1.3.1**    
> **Ubuntu VERSION == 22.04**


### 目錄

- [🛠️ 環境準備與相依套件](#️-環境準備與相依套件)
  - [1. 安裝基礎編譯工具](#1-安裝基礎編譯工具)
  - [2. 安裝PCL相依套件](#2-安裝PCL相依套件)
  - [3. (建議) 安裝 PCL 可選相依套件](#3-建議-安裝-pcl-可選相依套件)
- [⚙️ 編譯與安裝流程](#️-編譯與安裝流程)
- [✅ 驗證安裝](#-驗證安裝)
- [🤔 常見問題](#-常見問題)
- [📄 授權](#-授權)

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
```
