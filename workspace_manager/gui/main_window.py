from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QMessageBox, QCheckBox,
                             QFileDialog, QScrollArea, QGroupBox, QGridLayout,
                             QSpinBox)
from PyQt5.QtCore import Qt
import os
import subprocess
import yaml
import xml.etree.ElementTree as ET
import shutil
from ament_index_python.packages import get_package_share_directory

class WorkspaceManagerGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.workspace_root = None
        self.package_checkboxes = {}
        self.config_file = os.path.join(
            get_package_share_directory('workspace_manager'),
            'config',
            'config.yaml'
        )
        self.load_config()
        self.setupUI()
        self.package_dependencies = {}  # 存储包的依赖关系
        self.reverse_dependencies = {}  # 存储反向依赖关系
        self.always_on_top = False  # 添加一个标志来跟踪窗口是否置顶

        # 如果配置文件中有工作空间路径，则加载它
        if self.config.get('workspace_path'):
            self.workspace_root = self.config['workspace_path']
            self.workspace_path.setText(self.workspace_root)
            self.refresh_packages()

    def get_package_dependencies(self, package_xml_path):
        """解析package.xml获取依赖关系"""
        try:
            tree = ET.parse(package_xml_path)
            root = tree.getroot()
            deps = set()

            # 检查所有类型的依赖
            for dep_type in ['depend', 'build_depend', 'build_export_depend',
                             'exec_depend', 'test_depend']:
                for dep in root.findall(dep_type):
                    if dep.text:
                        deps.add(dep.text)
            return deps
        except (ET.ParseError, AttributeError):
            return set()


    def load_config(self):
        try:
            with open(self.config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        except (FileNotFoundError, yaml.YAMLError):
            # 默认配置
            self.config = {
                'workspace_path': '',
                'last_selected_packages': [],
                'symlink_install': True,
                'always_on_top': False,  # 添加新的配置项
                'parallel_workers': os.cpu_count() or 8,
            }

    def save_config(self):
        self.config['workspace_path'] = self.workspace_root or ''
        self.config['last_selected_packages'] = [
            pkg for pkg, cb in self.package_checkboxes.items()
            if cb.isChecked()
        ]
        self.config['symlink_install'] = self.symlink_check.isChecked()
        self.config['always_on_top'] = self.always_on_top  # 保存置顶状态
        # 保存并同步并行编译线程数
        try:
            self.config['parallel_workers'] = int(self.workers_spin.value())
        except Exception:
            self.config['parallel_workers'] = os.cpu_count() or 8

        os.makedirs(os.path.dirname(self.config_file), exist_ok=True)
        with open(self.config_file, 'w') as f:
            yaml.dump(self.config, f)

    def setupUI(self):
        self.setWindowTitle('ROS2 Workspace Manager')
        self.setMinimumSize(800, 600)

        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Workspace selection
        workspace_layout = QHBoxLayout()
        self.workspace_label = QLabel('Workspace:')
        self.workspace_path = QLabel('Not selected')
        self.select_workspace_btn = QPushButton('Select Workspace')
        self.select_workspace_btn.clicked.connect(self.select_workspace)
        workspace_layout.addWidget(self.workspace_label)
        workspace_layout.addWidget(self.workspace_path)
        workspace_layout.addWidget(self.select_workspace_btn)
        layout.addLayout(workspace_layout)

        # Package list
        packages_group = QGroupBox("Packages")
        packages_layout = QVBoxLayout()

        # Create scroll area for packages
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        self.packages_grid = QGridLayout(scroll_content)
        scroll.setWidget(scroll_content)
        packages_layout.addWidget(scroll)
        packages_group.setLayout(packages_layout)
        layout.addWidget(packages_group)

        # Build options
        options_layout = QHBoxLayout()
        self.symlink_check = QCheckBox('Symlink Install')
        self.symlink_check.setChecked(self.config.get('symlink_install', True))
        options_layout.addWidget(self.symlink_check)

        # 添加置顶按钮
        self.always_on_top_btn = QPushButton('置顶窗口')
        self.always_on_top_btn.setCheckable(True)  # 使按钮可切换
        self.always_on_top_btn.setChecked(self.config.get('always_on_top', False))
        self.always_on_top_btn.clicked.connect(self.toggle_always_on_top)
        options_layout.addWidget(self.always_on_top_btn)

        # 并行编译线程设置
        options_layout.addWidget(QLabel('Workers'))
        self.workers_spin = QSpinBox()
        self.workers_spin.setMinimum(1)
        max_workers = os.cpu_count() or 32
        self.workers_spin.setMaximum(max_workers)
        self.workers_spin.setValue(int(self.config.get('parallel_workers', max_workers)))
        self.workers_spin.valueChanged.connect(self.save_config)
        options_layout.addWidget(self.workers_spin)

        layout.addLayout(options_layout)

        # 如果配置中已经设置了置顶，则应用置顶状态
        self.always_on_top = self.config.get('always_on_top', False)
        if self.always_on_top:
            self.set_always_on_top(True)

        # Buttons
        button_layout = QHBoxLayout()
        self.select_all_btn = QPushButton('Select All')
        self.deselect_all_btn = QPushButton('Deselect All')
        self.build_btn = QPushButton('Build Selected')
        self.clean_btn = QPushButton('Clean')

        self.select_all_btn.clicked.connect(self.select_all_packages)
        self.deselect_all_btn.clicked.connect(self.deselect_all_packages)
        self.build_btn.clicked.connect(self.build_package)
        self.clean_btn.clicked.connect(self.clean_workspace)

        button_layout.addWidget(self.select_all_btn)
        button_layout.addWidget(self.deselect_all_btn)
        button_layout.addWidget(self.build_btn)
        button_layout.addWidget(self.clean_btn)
        layout.addLayout(button_layout)

    def toggle_always_on_top(self):
        """切换窗口置顶状态"""
        self.always_on_top = self.always_on_top_btn.isChecked()
        self.set_always_on_top(self.always_on_top)
        self.save_config()

    def set_always_on_top(self, on_top):
        """设置窗口置顶状态"""
        flags = self.windowFlags()
        if on_top:
            self.setWindowFlags(flags | Qt.WindowStaysOnTopHint)
            self.always_on_top_btn.setText('取消置顶')
        else:
            self.setWindowFlags(flags & ~Qt.WindowStaysOnTopHint)
            self.always_on_top_btn.setText('置顶窗口')
        self.show()  # 需要重新显示窗口以应用新的标志

        # 使用X11特定API设置窗口置顶
        try:
            if hasattr(self.windowHandle(), 'setProperty'):
                # 设置X11属性
                self.windowHandle().setProperty("_NET_WM_STATE_ABOVE", on_top)
        except Exception as e:
            print(f"无法设置X11窗口属性: {e}")

    def select_all_packages(self):
        for checkbox in self.package_checkboxes.values():
            checkbox.setChecked(True)

    def deselect_all_packages(self):
        for checkbox in self.package_checkboxes.values():
            checkbox.setChecked(False)

    def select_workspace(self):
        dir_path = QFileDialog.getExistingDirectory(self, 'Select Workspace Root')
        if dir_path:
            self.workspace_root = dir_path
            self.workspace_path.setText(dir_path)
            self.refresh_packages()
            self.save_config()

    def get_package_name_from_xml(self, package_xml_path):
        try:
            tree = ET.parse(package_xml_path)
            root = tree.getroot()
            return root.find('name').text
        except (ET.ParseError, AttributeError):
            return None

    def refresh_packages(self):
        if not self.workspace_root:
            QMessageBox.warning(self, 'Error', 'Please select workspace first!')
            return

        src_dir = os.path.join(self.workspace_root, 'src')
        if not os.path.exists(src_dir):
            QMessageBox.warning(self, 'Error', 'src directory not found!')
            return

        # 清除现有的包
        for i in reversed(range(self.packages_grid.count())):
            self.packages_grid.itemAt(i).widget().setParent(None)
        self.package_checkboxes.clear()
        self.package_dependencies.clear()
        self.reverse_dependencies.clear()

        # 第一遍：收集所有包和它们的依赖
        available_packages = {}
        for root, dirs, files in os.walk(src_dir):
            if 'package.xml' in files:
                package_xml_path = os.path.join(root, 'package.xml')
                package_name = self.get_package_name_from_xml(package_xml_path)
                if package_name:
                    available_packages[package_name] = package_xml_path
                    self.package_dependencies[package_name] = set()
                    self.reverse_dependencies[package_name] = set()

        # 第二遍：构建依赖关系
        for package_name, xml_path in available_packages.items():
            deps = self.get_package_dependencies(xml_path)
            # 只保留工作空间内的依赖
            workspace_deps = deps.intersection(available_packages.keys())
            self.package_dependencies[package_name] = workspace_deps
            # 构建反向依赖
            for dep in workspace_deps:
                self.reverse_dependencies[dep].add(package_name)

        # 创建复选框并设置连接
        row = 0
        col = 0
        max_cols = 3

        for package_name in available_packages.keys():
            checkbox = QCheckBox(package_name)
            if package_name in self.config.get('last_selected_packages', []):
                checkbox.setChecked(True)

            # 连接信号到新的处理方法
            checkbox.stateChanged.connect(lambda state, pkg=package_name:
                                          self.on_package_checkbox_changed(pkg, state))

            self.package_checkboxes[package_name] = checkbox
            self.packages_grid.addWidget(checkbox, row, col)

            col += 1
            if col >= max_cols:
                col = 0
                row += 1


    def build_package(self):
        if not self.workspace_root:
            QMessageBox.warning(self, 'Error', 'Please select workspace first!')
            return

        selected_packages = [
            pkg for pkg, cb in self.package_checkboxes.items()
            if cb.isChecked()
        ]

        if not selected_packages:
            QMessageBox.warning(self, 'Error', 'Please select at least one package!')
            return

        cmd = ['colcon', 'build']

        if self.symlink_check.isChecked():
            cmd.append('--symlink-install')

        # 添加parallel-workers参数
        # 使用配置中的并行线程数
        workers = int(self.config.get('parallel_workers', os.cpu_count() or 8))
        cmd.extend(['--parallel-workers', str(workers)])

        cmd.extend(['--packages-select'])
        cmd.extend(selected_packages)

        print("execute command :")
        print(cmd)

        try:
            process = subprocess.Popen(
                cmd,
                cwd=self.workspace_root,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            stdout, stderr = process.communicate()

            if process.returncode == 0:
                QMessageBox.information(self, 'Success', 'Build completed successfully!')
                self.save_config()
            else:
                QMessageBox.critical(self, 'Error', f'Build failed:\n{stderr}')
        except subprocess.CalledProcessError as e:
            QMessageBox.critical(self, 'Error', f'Build failed: {str(e)}')


    def closeEvent(self, event):
        self.save_config()
        super().closeEvent(event)

    def clean_workspace(self):
        if not self.workspace_root:
            QMessageBox.warning(self, 'Error', 'Please select workspace first!')
            return

        reply = QMessageBox.question(self, 'Confirm Clean',
                                     'This will clean both build and install directories while preserving specific files.\n'
                                     'Are you sure you want to continue?',
                                     QMessageBox.Yes | QMessageBox.No)

        if reply == QMessageBox.Yes:
            build_dir = os.path.join(self.workspace_root, 'build')
            install_dir = os.path.join(self.workspace_root, 'install')

            if not (os.path.exists(build_dir) or os.path.exists(install_dir)):
                QMessageBox.warning(self, 'Error', 'Neither build nor install directory found!')
                return

            try:
                import shutil

                def remove_contents(directory):
                    for item in os.listdir(directory):
                        item_path = os.path.join(directory, item)

                        # 跳过特定目录
                        if any(skip in item_path for skip in ['.cache', '.idea']):
                            continue

                        # 如果是文件
                        if os.path.isfile(item_path):
                            # 跳过需要保留的文件
                            if item in ['COLCON_IGNORE', 'compile_commands.json', '.built_by']:
                                continue
                            try:
                                os.remove(item_path)
                            except OSError as e:
                                self.node.get_logger().warning(f"Failed to remove file {item_path}: {e}")

                        # 如果是目录
                        elif os.path.isdir(item_path):
                            try:
                                shutil.rmtree(item_path)
                            except OSError as e:
                                self.node.get_logger().warning(f"Failed to remove directory {item_path}: {e}")

                # 清理 build 目录
                if os.path.exists(build_dir):
                    remove_contents(build_dir)

                # 清理 install 目录
                if os.path.exists(install_dir):
                    remove_contents(install_dir)

                QMessageBox.information(self, 'Success',
                                        'Clean completed successfully!\n'
                                        'Both build and install directories have been cleaned.')

            except Exception as e:
                QMessageBox.critical(self, 'Error', f'Clean failed: {str(e)}')
                self.node.get_logger().error(f'Clean failed: {str(e)}')

    def on_package_checkbox_changed(self, package_name, state):
        """处理包选择状态改变"""
        if self.package_checkboxes[package_name].isChecked():
            # 如果选中了一个包，递归选中其所有依赖
            self.select_dependencies(package_name)
        else:
            # 如果取消选中一个包，递归取消选中依赖它的包
            self.deselect_dependent_packages(package_name)

    def select_dependencies(self, package_name, visited=None):
        """递归选中所有依赖的包"""
        if visited is None:
            visited = set()

        if package_name in visited:
            return
        visited.add(package_name)

        # 选中当前包
        if package_name in self.package_checkboxes:
            self.package_checkboxes[package_name].setChecked(True)

        # 递归选中所有依赖
        for dep in self.package_dependencies.get(package_name, set()):
            self.select_dependencies(dep, visited)

    def deselect_dependent_packages(self, package_name, visited=None):
        """递归取消选中依赖此包的包"""
        if visited is None:
            visited = set()

        if package_name in visited:
            return
        visited.add(package_name)

        # 取消选中依赖此包的所有包
        for dep in self.reverse_dependencies.get(package_name, set()):
            if dep in self.package_checkboxes:
                self.package_checkboxes[dep].setChecked(False)
                self.deselect_dependent_packages(dep, visited)
