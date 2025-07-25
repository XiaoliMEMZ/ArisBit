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
  - 新增按钮事件功能。


## 🚀 快速上手

### 环境要求

- **硬件**: Yahboom ROSMaster v3
- **IDE**: Keil µVision 5

### 烧录步骤

1. **克隆仓库**:
   ```bash
   git clone https://github.com/your-username/ArisBit-FW.git
   cd ArisBit-FW