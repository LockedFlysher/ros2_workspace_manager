# Workspace Manager

一个用于管理 ROS2 工作空间的图形化工具，支持选择性编译、清理和管理 ROS2 包。

## 功能特点

- 📂 图形化选择工作空间目录
- 📦 自动检测并显示工作空间中的所有 ROS2 包
- ✅ 支持多选进行选择性编译
- 🔄 支持 symlink-install 模式
- 🧹 智能清理功能（保留关键文件）
- 💾 自动保存配置和上次选择

## 安装

1. 将本包克隆到你的 ROS2 工作空间的 src 目录：
```bash
cd ~/your_workspace/src
git clone https://gitee.com/LockedFlysher/ros2-workspace-manager.git
```

2. 安装依赖：
```bash
sudo apt update
sudo apt install libgtk-3-dev
pip3 install wxPython
sudo apt install python3-pyqt5  # PyQt5
pip3 install pyyaml            # PyYAML
```

3. 编译工作空间：
```bash
cd ~/your_workspace
colcon build --packages-select workspace_manager
```

4. 设置环境：
```bash
source ~/your_workspace/install/setup.bash
```

## 使用方法

1. 启动工具：
```bash
ros2 run workspace_manager workspace_manager
```

2. 基本操作：
    - 点击 "Select Workspace" 选择 ROS2 工作空间目录
    - 在包列表中选择需要编译的包
    - 根据需要勾选 "Symlink Install" 选项
    - 点击 "Build Selected" 开始编译
    - 使用 "Clean" 按钮清理工作空间（会保留必要文件）

## 配置文件

配置文件位于 `install/workspace_manager/share/workspace_manager/config/config.yaml`，包含：
- 上次选择的工作空间路径
- 上次选择的包列表
- Symlink Install 选项状态

## 注意事项

- 清理功能会保留以下文件：
    - build 目录中的：
        - .cache 目录
        - .idea 目录
        - COLCON_IGNORE
        - compile_commands.json
        - .built_by
    - install 目录中的：
        - COLCON_IGNORE

## 依赖项

- ROS2（已测试于 Humble）
- Python 3.8+
- PyQt5
- PyYAML

## 开发说明

### 项目结构
```
workspace_manager/
├── workspace_manager/
│   ├── __init__.py
│   ├── workspace_manager_node.py
│   ├── gui/
│   │   ├── __init__.py
│   │   └── main_window.py
│   └── config/
│       └── config.yaml
├── setup.py
├── setup.cfg
├── package.xml
└── resource/
```

### 构建新版本

1. 更新 `setup.py` 中的版本号
2. 更新 `package.xml` 中的版本号
3. 重新构建：
```bash
colcon build --packages-select workspace_manager
```

## 贡献

欢迎提交 Issue 和 Pull Request！

## 许可证
没有，但在B站点个关注即可随意分发

## 作者

B站id：晴糖豆

## 更新日志

### v0.0.1
- 初始版本
- 基本的工作空间管理功能
- 包选择和编译功能
- 工作空间清理功能