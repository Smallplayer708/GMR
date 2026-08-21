# GMR: 通用运动重定向（General Motion Retargeting）

  <a href="https://arxiv.org/abs/2505.02833">
    <img src="https://img.shields.io/badge/paper-arXiv%3A2505.02833-b31b1b.svg" alt="arXiv Paper"/>
  </a> <a href="https://arxiv.org/abs/2510.02252">
    <img src="https://img.shields.io/badge/paper-arXiv%3A2510.02252-b31b1b.svg" alt="arXiv Paper"/>
  </a> <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT"/>
  </a> <a href="https://github.com/YanjieZe/GMR/releases">
    <img src="https://img.shields.io/badge/version-0.2.0-blue.svg" alt="Version"/>
  </a> <a href="https://x.com/ZeYanjie/status/1952446745696469334">
    <img src="https://img.shields.io/badge/twitter-ZeYanjie-blue.svg" alt="Twitter"/>
  </a> <a href="https://yanjieze.github.io/humanoid-foundation/#GMR">
    <img src="https://img.shields.io/badge/blog-GMR-blue.svg" alt="Blog"/>
  </a> <a href="https://www.bilibili.com/video/BV1p1nazeEzC/?share_source=copy_web&vd_source=c76e3ab14ac3f7219a9006b96b4b0f76">
    <img src="https://img.shields.io/badge/tutorial-BILIBILI-blue.svg" alt="Blog"/>
  </a>

![Banner for GMR](./assets/GMR.png)

![GMR](./assets/GMR_pipeline.png)

#### GMR 的主要特点：
- 实时高质量重定向，释放实时全身遥操作的潜力，即 [TWIST](https://github.com/YanjieZe/TWIST)。
- 针对 RL 跟踪策略的良好性能进行了精细调优。
- 支持多种人形机器人及多种人体运动数据格式（详见下表）。

> [!NOTE]
> 如果您希望本仓库支持新的机器人或新的人体运动数据格式，请将机器人文件（`.xml`、`.urdf` 及网格文件）/人体运动数据发送至 <a href="mailto:lastyanjieze@gmail.com">Yanjie Ze</a>，或创建一个 issue，我们将尽快支持。同时请确保您发送的机器人文件可以在本仓库中开源。

本仓库基于 [MIT 许可证](LICENSE) 授权。


# 新闻与更新
- **2026-01-21：** GMR 现支持 [Xsens](https://www.xsens.com/) BVH 离线数据。
- **2026-01-12：** GMR 现支持 [Fourier GR3](https://www.fftai.com/)，这是本仓库中的第 17 款人形机器人。
- **2025-12-02：** GMR 现支持 [TWIST2](https://yanjieze.com/TWIST2)，其利用了 [XRoboToolkit SDK](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)。
- **2025-11-17：** 如需加入我们的社区进行讨论，可添加我的微信联系方式 [二维码](https://yanjieze.com/TWIST2/images/my_wechat.jpg)，备注信息格式如"[GMR] [您的姓名] [您的单位]"。
- **2025-11-08：** Jason Peng 的 [MimicKit] 现已支持 GMR 格式。请查看[此处](https://github.com/xbpeng/MimicKit/tree/main/tools/gmr_to_mimickit)。
- **2025-10-15：** 现已支持 [PAL Robotics 的 Talos](https://pal-robotics.com/robot/talos/)，这是第 15 款人形机器人。
- **2025-10-14：** GMR 现支持 [Nokov](https://www.nokov.com/) BVH 数据。
- **2025-10-14：** 新增关于 ik 配置的文档。参见 [DOC.md](DOC.md)
- **2025-10-09：** 查看 [TWIST](https://github.com/YanjieZe/TWIST) 开源代码以了解 RL 运动跟踪。
- **2025-10-02：** GMR 技术报告现已发布在 [arXiv](https://arxiv.org/abs/2510.02252) 上。
- **2025-10-01：** GMR 现支持将 GMR pickle 文件转换为 CSV（用于 beyondmimic），请查看 `scripts/batch_gmr_pkl_to_csv.py`。
- **2025-09-25：** GMR 的介绍视频已在 [Bilibili](https://www.bilibili.com/video/BV1p1nazeEzC/?share_source=copy_web&vd_source=c76e3ab14ac3f7219a9006b96b4b0f76) 上线。
- **2025-09-16：** GMR 现支持使用 [GVHMR](https://github.com/zju3dv/GVHMR) 从**单目视频**中提取人体姿态并将其重定向到机器人。
- **2025-09-12：** GMR 现支持 [Tienkung](https://github.com/Open-X-Humanoid/TienKung-Lab)，这是本仓库中的第 14 款人形机器人。
- **2025-08-30：** GMR 现支持 [Unitree H1 2](https://www.unitree.com/cn/h1) 和 [PND Adam Lite](https://pndbotics.com/)，分别是本仓库中的第 12 和第 13 款人形机器人。
- **2025-08-28：** GMR 现支持 [Booster T1](https://www.boosterobotics.com/) 的 23dof 和 29dof 两个版本。
- **2025-08-28：** GMR 现支持使用从 [OptiTrack](https://www.optitrack.com/) 导出的离线 FBX 运动数据。
- **2025-08-27：** GMR 现支持 [Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Assets)，这是本仓库中的第 11 款人形机器人。
- **2025-08-24：** GMR 现支持 [Unitree H1](https://www.unitree.com/h1/)，这是本仓库中的第 10 款人形机器人。
- **2025-08-24：** GMR 现支持机器人电机速度限制，`GeneralMotionRetargeting` 类中默认 `use_velocity_limit=True`（默认速度限制为 3*pi）；同时新增了机器人 DoF/Body/Motor 名称及其 ID 的打印（默认开启），您可以通过 `robot_dof_names`、`robot_body_names` 和 `robot_motor_names` 属性访问它们。
- **2025-08-10：** GMR 现支持 [Booster K1](https://www.boosterobotics.com/)，这是本仓库中的第 9 款机器人。
- **2025-08-09：** GMR 现支持 *带有 Dex31 灵巧手的 Unitree G1*。
- **2025-08-07：** GMR 现支持 [Galexea R1 Pro](https://galaxea-dynamics.com/)（这是一款轮式人形机器人！）和 [KUAVO](https://www.kuavo.ai/)，分别是本仓库中的第 7 和第 8 款人形机器人。
- **2025-08-06：** GMR 现支持 [HighTorque Hi](https://www.hightorquerobotics.com/hi/)，这是本仓库中的第 6 款人形机器人。
- **2025-08-04：** GMR 初始版本发布。查看我们的[推特帖子](https://x.com/ZeYanjie/status/1952446745696469334)。

## 演示

<table>
  <tr>
    <td align="center" width="20%">
      <b>演示 1</b><br>
      将 LAFAN1 舞蹈动作重定向到 5 款机器人。<br>
      <video src="https://github.com/user-attachments/assets/23566fa5-6335-46b9-957b-4b26aed11b9e" width="200" controls></video>
    </td>
    <td align="center" width="20%">
      <b>演示 2</b><br>
      Galexea R1 Pro 机器人（视角 1）。<br>
      <video src="https://github.com/user-attachments/assets/903ed0b0-0ac5-4226-8f82-5a88631e9b7c" width="200" controls></video>
    </td>
    <td align="center" width="20%">
      <b>演示 3</b><br>
      Galexea R1 Pro 机器人（视角 2）。<br>
      <video src="https://github.com/user-attachments/assets/deea0e64-f1c6-41bc-8661-351682006d5d" width="200" controls></video>
    </td>
    <td align="center" width="20%">
      <b>演示 4</b><br>
      只需更改一个参数即可切换机器人。<br>
      <video src="https://github.com/user-attachments/assets/03f10902-c541-40b1-8104-715a5759fd5e" width="200" controls></video>
    </td>
    <td align="center" width="20%">
      <b>演示 5</b><br>
      HighTorque 机器人做扭动舞蹈。<br>
      <video src="https://github.com/user-attachments/assets/1d3e663b-f29e-41b1-8e15-5c0deb6a4a5c" width="200" controls></video>
    </td>
  </tr>

  <tr>
    <td align="center">
      <b>演示 6</b><br>
      Kuavo 机器人捡起一个箱子。<br>
      <video src="https://github.com/user-attachments/assets/02fc8f41-c363-484b-a329-4f4e83ed5b80" width="200" controls></video>
    </td>
    <td align="center">
      <b>演示 7</b><br>
      Unitree H1 机器人跳恰恰舞。<br>
      <video src="https://github.com/user-attachments/assets/28ee6f0f-be30-42bb-8543-cf1152d97724" width="200" controls></video>
    </td>
    <td align="center">
      <b>演示 8</b><br>
      Booster T1 机器人跳跃（视角 1）。<br>
      <video src="https://github.com/user-attachments/assets/2c75a146-e28f-4327-930f-5281bfc2ca9c" width="200" controls></video>
    </td>
    <td align="center">
      <b>演示 9</b><br>
      Booster T1 机器人跳跃（视角 2）。<br>
      <video src="https://github.com/user-attachments/assets/ff10c7ef-4357-4789-9219-23c6db8dba6d" width="200" controls></video>
    </td>
    <td align="center">
      <b>演示 10</b><br>
      Unitree H1-2 机器人跳跃。<br>
      <video src="https://github.com/user-attachments/assets/2382d8ce-7902-432f-ab45-348a11eeb312" width="200" controls></video>
    </td>
  </tr>

  <tr>
    <td align="center">
      <b>演示 11</b><br>
      PND Adam Lite 机器人。<br>
      <video src="https://github.com/user-attachments/assets/a8ef1409-88f1-4393-9cd0-d2b14216d2a4" width="200" controls></video>
    </td>
    <td align="center">
      <b>演示 12</b><br>
      Tienkung 机器人行走。<br>
      <video src="https://github.com/user-attachments/assets/7a775ecc-4254-450c-a3eb-49e843b8e331" width="200" controls></video>
    </td>
    <td align="center">
      <b>演示 13</b><br>
      提取人体姿态（GVHMR + GMR）。<br>
      <a href="https://www.bilibili.com/video/BV1Tnpmz9EaE">▶ 在 Bilibili 上观看</a>
    </td>
    <td align="center">
      <b>演示 14</b><br>
      PAL Robotics 的 Talos 机器人对打。<br>
      <video src="https://github.com/user-attachments/assets/3ec0bf80-80c1-4181-a623-dc2b072c2ca2" width="200" controls></video>
    </td>
    <td align="center">
      <b>演示 15</b><br>
      （如果您以后添加新的演示，可在此放置占位内容！）<br>
      <i>敬请期待...</i>
    </td>
  </tr>
</table>


## 支持的机器人与数据格式



| 分配 ID | 机器人/数据格式 | 机器人自由度 | SMPLX（[AMASS](https://amass.is.tue.mpg.de/)、[OMOMO](https://github.com/lijiaman/omomo_release)） | BVH [LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset) | FBX（[OptiTrack](https://www.optitrack.com/)） | BVH [Nokov](https://www.nokov.com/) | PICO（[XRoboToolkit](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)） | 更多格式即将推出 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | Unitree G1 `unitree_g1` | 腿（2\*6）+ 腰（3）+ 臂（2\*7）= 29 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 1 | 带灵巧手的 Unitree G1 `unitree_g1_with_hands` | 腿（2\*6）+ 腰（3）+ 臂（2\*7）+ 手（2\*7）= 43 | ✅ | ✅ | ✅ | 待定 | 待定 |
| 2 | Unitree H1 `unitree_h1` | 腿（2\*5）+ 腰（1）+ 臂（2\*4）= 19 | ✅ | 待定 | 待定 | 待定 | 待定 |
| 3 | Unitree H1 2 `unitree_h1_2` | 腿（2\*6）+ 腰（1）+ 臂（2\*7）= 27 | ✅ | 待定 | 待定 | 待定 | 待定 |
| 4 | Booster T1 `booster_t1` | 待定 | ✅ | 待定 | 待定 | 待定 | 待定 |
| 5 | Booster T1 29自由度 `booster_t1_29dof` | 待定 | ✅ | ✅ | 待定 | 待定 | 待定 |
| 6 | Booster K1 `booster_k1` | 颈（2）+ 臂（2\*4）+ 腿（2\*6）= 22 | ✅ | 待定 | 待定 | 待定 | 待定 |
| 7 | Stanford ToddlerBot `stanford_toddy` | 待定 | ✅ | ✅ | 待定 | 待定 | 待定 |
| 8 | Fourier N1 `fourier_n1` | 待定 | ✅ | ✅ | 待定 | 待定 | 待定 |
| 9 | ENGINEAI PM01 `engineai_pm01` | 待定 | ✅ | ✅ | 待定 | 待定 | 待定 |
| 10 | HighTorque Hi `hightorque_hi` | 头（2）+ 臂（2\*5）+ 腰（1）+ 腿（2\*6）= 25 | ✅ | 待定 | 待定 | 待定 | 待定 |
| 11 | Galaxea R1 Pro `galaxea_r1pro`（这是一款轮式机器人！） | 基座（6）+ 躯干（4）+ 臂（2*7）= 24 | ✅ | 待定 | 待定 | 待定 | 待定 |
| 12 | Kuavo `kuavo_s45` | 头（2）+ 臂（2\*7）+ 腿（2\*6）= 28 | ✅ | 待定 | 待定 | 待定 | 待定 |
| 13 | Berkeley Humanoid Lite `berkeley_humanoid_lite`（需要进一步调优） | 腿（2\*6）+ 臂（2\*5）= 22 | ✅ | 待定 | 待定 | 待定 | 待定 |
| 14 | PND Adam Lite `pnd_adam_lite` | 腿（2\*6）+ 腰（3）+ 臂（2\*5）= 25 | ✅ | 待定 | 待定 | 待定 | 待定 |
| 15 | Tienkung `tienkung` | 腿（2\*6）+ 臂（2\*4）= 20 | ✅ | 待定 | 待定 | 待定 | 待定 |
| 16 | PAL Robotics 的 Talos `pal_talos` | 头（2）+ 臂（2\*7）+ 腰（2）+ 腿（2\*6）= 30 | ✅ | 待定 | 待定 | 待定 | 待定 |
| 17 | Fourier GR3 `fourier_gr3` | 头（2）+ 臂（2\*7）+ 腰（3）+ 腿（2\*6）= 31 | ✅ | 待定 | 待定 | 待定 | 待定 |
| 更多机器人即将推出！ |
| 18 | AgiBot A2 `agibot_a2` | 待定 | 待定 | 待定 | 待定 | 待定 |
| 19 | OpenLoong `openloong` | 待定 | 待定 | 待定 | 待定 | 待定 |




## 安装

> [!NOTE]
> 该代码已在 Ubuntu 22.04/20.04 上测试通过。

首先创建您的 conda 环境：

```bash
conda create -n gmr python=3.10 -y
conda activate gmr
```

然后，安装 GMR：

```bash
pip install -e .
```

安装 SMPLX 后，如果您使用的是 SMPL-X pkl 文件，请将 `smplx/body_models.py` 中的 `ext` 从 `npz` 改为 `pkl`。

此外，为解决一些可能的渲染问题：

```bash
conda install -c conda-forge libstdcxx-ng -y
```

## 数据准备

[[SMPLX](https://github.com/vchoutas/smplx) 身体模型] 从 [SMPL-X](https://smpl-x.is.tue.mpg.de/) 下载 SMPL-X 身体模型到 `assets/body_models`，并按如下结构组织：
```bash
- assets/body_models/smplx/
-- SMPLX_NEUTRAL.pkl
-- SMPLX_FEMALE.pkl
-- SMPLX_MALE.pkl
```

[[AMASS](https://amass.is.tue.mpg.de/) 运动数据] 从 [AMASS](https://amass.is.tue.mpg.de/) 下载原始 SMPL-X 数据到任意文件夹。注意：请勿下载 SMPL+H 数据。

[[OMOMO](https://github.com/lijiaman/omomo_release) 运动数据] 从[此谷歌网盘文件](https://drive.google.com/file/d/1tZVqLB7II0whI-Qjz-z-AU3ponSEyAmm/view?usp=sharing)下载原始 OMOMO 数据到任意文件夹。然后使用 `scripts/convert_omomo_to_smplx.py` 将数据处理为 SMPL-X 格式。

[[LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset) 运动数据] 从[官方仓库](https://github.com/ubisoft/ubisoft-laforge-animation-dataset)下载原始 LAFAN1 bvh 文件，即 [lafan1.zip](https://github.com/ubisoft/ubisoft-laforge-animation-dataset/blob/master/lafan1/lafan1.zip)。


## 人体/机器人运动数据格式

为了更好地使用本库，您可以先了解我们所使用的人体运动数据以及我们获得的机器人运动数据。

**人体运动数据**的每一帧都是一个 dict，格式为（human_body_name，3D 全局平移 + 全局旋转）。旋转通常用四元数表示（默认使用 wxyz 顺序，以与 mujoco 对齐）。

**机器人运动数据**的每一帧可以理解为一个元组（robot_base_translation，robot_base_rotation，robot_joint_positions）。

## 使用方法

### [新增] PICO 流式传输到机器人（TWIST2）

安装 PICO SDK：
1. 在您的 PICO 上安装 PICO SDK：参见[此处](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/)。
2. 在您自己的电脑上，
    - 下载 [ubuntu 22.04 的 deb 包](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb)，或从[仓库源码](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)构建。
    - 安装时使用命令
        ```bash
        sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
        ```
        然后您应该能在应用列表中看到 `xrobotoolkit-pc-service`。在进行遥操作之前，请记得启动该应用。
    - 构建用于 PICO 流式传输的 PICO PC Service SDK 和 Python SDK：
        ```bash
        conda activate gmr

        git clone https://github.com/YanjieZe/XRoboToolkit-PC-Service-Pybind.git
        cd XRoboToolkit-PC-Service-Pybind

        mkdir -p tmp
        cd tmp
        git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git
        cd XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK 
        bash build.sh
        cd ../../../..
        

        mkdir -p lib
        mkdir -p include
        cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h include/
        cp -r tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann include/nlohmann/
        cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so lib/
        # rm -rf tmp

        # 构建项目
        conda install -c conda-forge pybind11
        pip uninstall -y xrobotoolkit_sdk
        python setup.py install
        ```

到这里您就全部设置好了！

要试用它，请查看 [TWIST2 中的此脚本](https://github.com/amazon-far/TWIST2/blob/master/teleop.sh)：
```bash
bash teleop.sh
```
您应该能够在 mujoco 窗口中看到重定向后的机器人运动。

### 从 SMPL-X（AMASS、OMOMO）重定向到机器人

> [!NOTE]
> 注意：安装 SMPL-X 后，如果您使用的是 SMPL-X pkl 文件，请将 `smplx/body_models.py` 中的 `ext` 从 `npz` 改为 `pkl`。

重定向单个动作：

```bash
python scripts/smplx_to_robot.py --smplx_file <smplx数据路径> --robot <机器人数据路径> --save_path <机器人数据保存路径.pkl> --rate_limit
```

默认情况下，您应该在 mujoco 窗口中看到重定向后机器人运动的可视化。
如果您想录制视频，请添加 `--record_video` 和 `--video_path <您的视频路径,mp4>`。

- `--rate_limit` 用于限制重定向后机器人运动的速度，使其与人体运动保持一致。如果您希望尽可能快，请移除 `--rate_limit`。

重定向文件夹中的一系列动作：

```bash
python scripts/smplx_to_robot_dataset.py --src_folder <smplx数据目录路径> --tgt_folder <机器人数据保存目录路径> --robot <机器人名称>
```

默认情况下，批量重定向没有可视化。

### 从 GVHMR 重定向到机器人

首先，按照[官方说明](https://github.com/zju3dv/GVHMR/blob/main/docs/INSTALL.md)安装 GVHMR。

然后运行他们的演示，从单目视频中提取人体姿态：

```bash
cd path/to/GVHMR
python tools/demo/demo.py --video=docs/example_video/tennis.mp4 -s
```

然后您应该会获得保存在 `GVHMR/outputs/demo/tennis/hmr4d_results.pt` 中的人体姿态数据。

接着，运行以下命令将提取的人体姿态数据重定向到您的机器人：

```bash
python scripts/gvhmr_to_robot.py --gvhmr_pred_file <hmr4d_results.pt路径> --robot unitree_g1 --record_video
```



## 从 BVH（LAFAN1、Nokov）重定向到机器人

重定向单个动作：

```bash
# 单个动作
python scripts/bvh_to_robot.py --bvh_file <bvh数据路径> --robot <机器人数据路径> --save_path <机器人数据保存路径.pkl> --rate_limit --format <格式>
```

默认情况下，您应该在 mujoco 窗口中看到重定向后机器人运动的可视化。
- `--rate_limit` 用于限制重定向后机器人运动的速度，使其与人体运动保持一致。如果您希望尽可能快，请移除 `--rate_limit`。
- `--format` 用于指定 BVH 数据的格式。支持的格式为 `lafan1` 和 `nokov`。


重定向文件夹中的一系列动作：

```bash
python scripts/bvh_to_robot_dataset.py --src_folder <bvh数据目录路径> --tgt_folder <机器人数据保存目录路径> --robot <机器人名称>
```

默认情况下，批量重定向没有可视化。



## 从 Xsens 重定向到机器人

### 离线：Xsens BVH 到机器人

#### 使用 MuJoCo 可视化 Xsens BVH 数据

安装 PyQt6：
```bash
pip install PyQt6 PyQt6-Qt6 PyQt6-sip
```


```bash
python general_motion_retargeting/utils/xsens_vendor/mujoco_xsens_bvh_view.py \
  --bvh_file <bvh数据目录路径> \
  --scale <位移缩放大小> \
  --reset_to_zero
```
例如
```bash
python general_motion_retargeting/utils/xsens_vendor/mujoco_xsens_bvh_view.py \
  --scale 0.01 \
  --bvh_file assets/xsens_bvh_test/251021_04_boxing_120Hz_cm_3DsMax.bvh \
  --reset_to_zero
```

- `--start` 用于指定起始处理帧。如果不输入，默认从第一帧开始处理。

- `--end` 用于指定结束处理帧。如果不输入，默认处理到最后一帧。

- `--reset_to_zero` 用于将位移和 Z 轴旋转重置为零。该功能与 `--start` 配合使用时，可以很好地让数据回到初始的零位置。因为有时某些数据集的前一两帧与后续数据差异过大，这些数据需要被丢弃。

- `--scale` 用于设置位移的缩放值，其取值取决于数据集中位移所使用的单位与米之间的换算关系。

- ##### 使用之前，您必须安装 PyQt6。`pip install PyQt6`
- ##### 执行此命令后，将启动一个 UI 界面，使您能够调整每个关节在 x、y、z 方向上的各个通道的角度值。完成调整后，点击 `"Apply and Preview"` 按钮，将生成本地 `offset.json` 文件并执行 BVH 文件的 MuJoCo 可视化回放。运行 `xsens_bvh_to_robot.py` 时，它会读取此 JSON 文件中的数据。因此，您需要在运动重定向之前先执行 `mujoco_xsens_bvh_view.py`，以确保本地存在 `offset.json` 文件。

#### 重定向单个动作：
```bash
# 单个动作
python scripts/xsens_bvh_to_robot.py \
  --bvh_file <bvh数据路径> \
  --robot <机器人数据路径> \
  --save_path <机器人数据保存路径.pkl> \
  --rate_limit \
  --start <起始帧编号> \
  --scale <位移缩放大小> \
  --reset_to_zero \
  --bvh_format <导出的bvh格式>
```
例如
```bash
python scripts/xsens_bvh_to_robot.py  \
  --robot unitree_h1_2 \
  --scale 0.01 \
  --reset_to_zero \
  --bvh_format 3DSM \
  --bvh_file assets/xsens_bvh_test/251021_04_boxing_120Hz_cm_3DsMax.bvh \
  --save_path retargeting_data/h1/251021_04_boxing_120Hz_cm_3DsMax.pkl
```
##### 默认情况下，您应该在 mujoco 窗口中看到重定向后机器人运动的可视化。
- `--rate_limit` 用于限制重定向后机器人运动的速度，使其与人体运动保持一致。如果您希望尽可能快，请移除 `--rate_limit`。

- `--start` 用于指定起始处理帧。如果不输入，默认从第一帧开始处理。

- `--end` 用于指定结束处理帧。如果不输入，默认处理到最后一帧。

- `--reset_to_zero` 用于将位移和 Z 轴旋转重置为零。该功能与 `--start` 配合使用时，可以很好地让数据回到初始的零位置。因为有时某些数据集的前一两帧与后续数据差异过大，这些数据需要被丢弃。

- `--scale` 用于设置位移的缩放值，其取值取决于数据集中位移所使用的单位与米之间的换算关系。

##### ！！！！！！！！！！！！！！！！！！ 注意 ！！！！！！！！！！！！！！！！！！！！
- `--bvh_format` 用于设置要解析的 bvh 的格式。在 Xsens MVN 软件中，可以导出三种格式的 BVH 文件。不同格式的 BVH 文件之间会存在一些差异。这里我建议使用 3D Studio Max 格式。（事实上，我还没有完成其他格式数据的解析。）

- 导出的 pkl 文件将以 `wxyz` 格式表示四元数。^ _ ^

---

### 在线流式传输（Xsens MVN）

将 **Xsens MVN 软件**的实时运动数据直接流式传输到 GMR，用于实时机器人重定向。

#### 1. 安装 Xsens MVN UDP 数据解析器

`xsens_mvn_robot_python` 库将 Xsens MVN 网络数据报（位置 + 四元数格式的方向）解析为 Python 可访问的数据结构。请为您的 Python 版本安装正确的 `.whl` 文件。

```bash
# 克隆解析器仓库
git clone https://github.com/jiminghe/xsens_mvn_robot_python.git
cd xsens_mvn_robot_python

# 安装与您的 Python 版本匹配的 wheel 包
# Python 3.10 示例：
pip install xsens_mvn_robot_python-*-cp310-*.whl
```

> 请选择文件名中包含您 Python 版本标签的 `.whl` 文件（例如 Python 3.10 对应 `cp310`，Python 3.8 对应 `cp38`）。该库会自动处理 UDP socket 绑定和数据报解包。

#### 2. 配置 Xsens MVN 网络流

在 Windows 或 Linux 上启动 **Xsens MVN 软件**。您可以在穿着 Xsens Link / Awinda 服装的情况下从实时录制会话进行流式传输，也可以回放先前录制的 `.mvn` 文件。

| 步骤 | 操作 |
|---|---|
| 1 | 点击 **选项 → 网络流（Network Streamer）** |
| 2 | 在弹出的窗口中，点击 **添加（Add）** 以创建新的流目标 |
| 3 | 设置 **主机地址（Host Address）**（见下表） |
| 4 | 在网络流选项中，仅勾选 **位置 + 方向（四元数）（Position + Orientation (Quaternion)）** |
| 5 | GMR 重定向不需要其他数据源 |
| 6 | 点击 **确定（OK）** —— 确认流在绿色状态下 |

**主机地址参考：**

| 场景 | 主机地址设置 |
|---|---|
| MVN 位于同一台 Linux 机器上（MVN Linux） | `127.0.0.1`（本地回环） |
| MVN 位于 Windows 上 → 流式传输到 Ubuntu（同一局域网） | Ubuntu 的 IP 地址，例如 `192.168.1.10` |

> **重要：** 从 Windows PC 向 Ubuntu 计算机流式传输时，请确保两台机器在同一局域网内。为 MVN 应用程序禁用 Windows 防火墙，或在 MVN 默认端口（`9763`）上创建入站 UDP 规则。

#### 3. 运行 GMR 实时流式传输脚本

在 Xsens MVN 网络流处于活动状态且 conda 环境已加载的情况下，运行实时流式传输重定向脚本。将打开一个 MuJoCo 窗口，显示重定向后的 Unitree G1 机器人实时镜像您的动作。

```bash
# 激活 GMR 环境
conda activate gmr

# 运行 Xsens 实时流式传输重定向脚本
python scripts/xsens_live_streaming.py
```

### 从 FBX（OptiTrack）重定向到机器人

#### 离线 FBX 文件

重定向单个动作：

1. 按照[这些说明](https://github.com/nv-tlabs/ASE/tree/main/ase/poselib#importing-from-fbx)和[这些说明](https://github.com/nv-tlabs/ASE/issues/61#issuecomment-2670315114)安装 `fbx_sdk`。您可能为此需要新建一个 conda 环境。

2. 激活您安装了 `fbx_sdk` 的 conda 环境。
使用以下命令从您的 `.fbx` 文件中提取运动数据：

```bash
cd third_party
python poselib/fbx_importer.py --input <fbx文件路径.fbx> --output <运动数据保存路径.pkl> --root-joint <根关节名称> --fps <帧率>
```

3. 然后，运行以下命令将提取的运动数据重定向到您的机器人：

```bash
conda activate gmr
# 单个动作
python scripts/fbx_offline_to_robot.py --motion_file <已保存运动数据路径.pkl> --robot <机器人数据路径> --save_path <机器人数据保存路径.pkl> --rate_limit
```

默认情况下，您应该在 mujoco 窗口中看到重定向后机器人运动的可视化。

- `--rate_limit` 用于限制重定向后机器人运动的速度，使其与人体运动保持一致。如果您希望尽可能快，请移除 `--rate_limit`。

#### 在线流式传输

我们提供了使用 OptiTrack MoCap 数据进行实时流式传输和重定向的脚本。

通常您会有两台计算机，一台是安装了 Motive（OptiTrack 的桌面应用程序）的服务器，另一台是安装了 GMR 的客户端。

找到服务器 IP（安装了 Motive 的计算机）和客户端 IP（您的计算机）。按如下方式设置流式传输：

![OptiTrack 流式传输](./assets/optitrack.png)

然后运行：

```bash
python scripts/optitrack_to_robot.py --server_ip <服务器ip> --client_ip <客户端ip> --use_multicast False --robot unitree_g1
```

您应该能看到重定向后机器人运动在 mujoco 窗口中的可视化。

### 可视化已保存的机器人运动

可视化单个动作：

```bash
python scripts/vis_robot_motion.py --robot <机器人名称> --robot_motion_path <已保存机器人数据路径.pkl>
```

如果您想录制视频，请添加 `--record_video` 和 `--video_path <您的视频路径,mp4>`。

可视化文件夹中的一系列动作：

```bash
python scripts/vis_robot_motion_dataset.py --robot <机器人名称> --robot_motion_folder <已保存机器人数据文件夹路径>
```

启动 MuJoCo 可视化窗口并点击它后，您可以使用以下键盘控制：
* `[`：播放上一个动作
* `]`：播放下一个动作
* `space`：切换播放/暂停

## 速度基准

| CPU | 重定向速度 |
| --- | --- |
| AMD Ryzen Threadripper 7960X 24 核 | 60~70 FPS |
| 第 13 代 Intel Core i9-13900K 24 核 | 35~45 FPS |
| 待定 | 待定 |

## 引用

如果您觉得我们的代码有用，请考虑引用我们的相关论文：

```bibtex
@article{joao2025gmr,
  title={Retargeting Matters: General Motion Retargeting for Humanoid Motion Tracking},
  author= {Joao Pedro Araujo and Yanjie Ze and Pei Xu and Jiajun Wu and C. Karen Liu},
  year= {2025},
  journal= {arXiv preprint arXiv:2510.02252}
}
```

```bibtex
@article{ze2025twist,
  title={TWIST: Teleoperated Whole-Body Imitation System},
  author= {Yanjie Ze and Zixuan Chen and João Pedro Araújo and Zi-ang Cao and Xue Bin Peng and Jiajun Wu and C. Karen Liu},
  year= {2025},
  journal= {arXiv preprint arXiv:2505.02833}
}
```

以及这个 GitHub 仓库：

```bibtex
@software{ze2025gmr,
  title={GMR: General Motion Retargeting},
  author= {Yanjie Ze and João Pedro Araújo and Jiajun Wu and C. Karen Liu},
  year= {2025},
  url= {https://github.com/YanjieZe/GMR},
  note= {GitHub repository}
}
```

## 已知问题

为所有不同的人设计单一配置并非易事。我们观察到某些动作可能产生不佳的重定向结果。如果您观察到一些不好的结果，请告诉我们！我们现在在 [TEST_MOTIONS.md](TEST_MOTIONS.md) 中收集了此类动作。

## 致谢

我们的 IK 求解器基于 [mink](https://github.com/kevinzakka/mink) 和 [mujoco](https://github.com/google-deepmind/mujoco) 构建。我们的可视化基于 [mujoco](https://github.com/google-deepmind/mujoco)。我们尝试的人体运动数据包括 [AMASS](https://amass.is.tue.mpg.de/)、[OMOMO](https://github.com/lijiaman/omomo_release) 和 [LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset)。

原始机器人模型可以在以下位置找到：

* [Berkley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Assets)：CC-BY-SA-4.0 许可证
* [Booster K1](https://www.boosterobotics.com/)
* [Booster T1](https://booster.feishu.cn/wiki/UvowwBes1iNvvUkoeeVc3p5wnUg)（[英文版](https://booster.feishu.cn/wiki/DtFgwVXYxiBT8BksUPjcOwG4n4f)）
* [EngineAI PM01](https://github.com/engineai-robotics/engineai_ros2_workspace)：[文件链接](https://github.com/engineai-robotics/engineai_ros2_workspace/blob/community/src/simulation/mujoco/assets/resource)
* [Fourier N1](https://github.com/FFTAI/Wiki-GRx-Gym)：[文件链接](https://github.com/FFTAI/Wiki-GRx-Gym/tree/FourierN1/legged_gym/resources/robots/N1)
* [Galaxea R1 Pro](https://galaxea-dynamics.com/)：MIT 许可证
* [HighToqure Hi](https://www.hightorquerobotics.com/hi/)
* [LEJU Kuavo S45](https://gitee.com/leju-robot/kuavo-ros-opensource/blob/master/LICENSE)：MIT 许可证
* [PAL Robotics 的 Talos](https://github.com/google-deepmind/mujoco_menagerie)：[文件链接](https://github.com/google-deepmind/mujoco_menagerie/tree/main/pal_talos)
* [Toddlerbot](https://github.com/hshi74/toddlerbot)：[文件链接](https://github.com/hshi74/toddlerbot/tree/main/toddlerbot/descriptions/toddlerbot_active)
* [Unitree G1](https://github.com/unitreerobotics/unitree_ros)：[文件链接](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description)
