# ArisBit

<!-- 在这里可以添加一些徽章，例如构建状态、许可证等 -->
[![LICENSE](https://img.shields.io/badge/license-MIT-green)](./LICENSE)

> 一款为Yahboom ROSMaster v3下位板对于RCJ比赛场景深度优化的固件。

此固件基于 Yahboom 官方固件进行了大量修改和功能增强，旨在提供更稳定、更强大、更易于扩展的底层控制体验。

<div align="center">
  <img src="arisu.jpg" width="600" alt="Arisu">
  <br>
  <em>图源: <a href="https://www.pixiv.net/artworks/113515308">Pixiv @mrIto</a></em>
</div>

## ✨ 项目特性

- **功能优化**:
  - 重写了PID电机控制部分，提升维护性。
  - 删减不需要的PWM舵机控制，改为IO操作。
- **功能增强**:
  - 新增按钮事件功能


## 🚀 快速上手

### 环境要求

- **硬件**: Yahboom ROSMaster v3
- **IDE**: Keil µVision 5

### 串口绑定（必须）

1. **克隆仓库**

2. **将arisbit.rules复制到树莓派/etc/udev/rules.d/目录下**

3. **在树莓派控制台执行下列命令**

```bash
sudo chmod a+x /etc/udev/rules.d/arisbit.rules
sudo udevadm trigger
sudo service udev reload
sudo service udev restart
```

4. **最后重新插拔主板**


### 烧录教程

> 目前仅在Windows平台FlyMCU软件上测试刷录成功，故展示本方法。其余刷录方式可自行研究。

1. **克隆仓库**

2. **打开FlyMCU.exe，并选择arisbit.hex作为固件文件**

3. **MicroUSB连接主板到电脑，点击搜索串口，并选择对应的串口**

4. **在底部配置选择「DTR的低电平复位，RTS高电平进BootLoader」**

5. **进入主板刷录模式：先按住扩展板上的BOOT0键，再按一下RESET键，最后松开BOOT0键**

6. **点击开始编程，等待编程完成**