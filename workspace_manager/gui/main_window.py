"""PyQt5 UI for the ROS2 Workspace Manager with improved styling and UX."""

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QMessageBox, QCheckBox,
    QFileDialog, QGroupBox,
    QSpinBox, QToolBar, QAction, QLineEdit, QTextEdit,
    QStatusBar, QComboBox, QSplitter, QStyle, QProgressBar,
    QTableWidget, QTableWidgetItem, QHeaderView,
)
from PyQt5.QtCore import Qt, QProcess, QSize
import os
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
        self.build_process = None
        self.theme_name = self.config.get('theme', 'dark')
        self.setupUI()
        # 应用主题
        try:
            self.apply_theme(self.theme_name)
        except Exception:
            pass
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
                'theme': 'dark',
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
        # 保存主题
        try:
            self.config['theme'] = self.theme_combo.currentData() or self.theme_name
        except Exception:
            self.config['theme'] = self.theme_name

        os.makedirs(os.path.dirname(self.config_file), exist_ok=True)
        with open(self.config_file, 'w') as f:
            yaml.dump(self.config, f)

    def setupUI(self):
        self.setWindowTitle('ROS2 Workspace Manager')
        self.setMinimumSize(980, 680)

        # Toolbar
        self.toolbar = QToolBar('工具栏')
        self.toolbar.setIconSize(QSize(18, 18))
        self.addToolBar(self.toolbar)
        self._create_toolbar_actions()

        # Central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Workspace + Search header
        header_layout = QHBoxLayout()
        self.workspace_label = QLabel('工作空间:')
        self.workspace_path = QLabel('未选择')
        header_layout.addWidget(self.workspace_label)
        header_layout.addWidget(self.workspace_path, stretch=1)
        self.search_edit = QLineEdit()
        self.search_edit.setPlaceholderText('搜索包...')
        self.search_edit.textChanged.connect(self._apply_search_filter)
        header_layout.addWidget(self.search_edit)
        layout.addLayout(header_layout)

        # Splitter with packages (left) and logs (right)
        splitter = QSplitter()

        # Packages panel
        packages_group = QGroupBox('包列表')
        packages_layout = QVBoxLayout()
        # 表格形式展示包
        self.packages_table = QTableWidget(0, 2)
        self.packages_table.setHorizontalHeaderLabels(['选择', '包名'])
        header = self.packages_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        self.packages_table.setAlternatingRowColors(True)
        self.packages_table.setShowGrid(True)
        packages_layout.addWidget(self.packages_table)
        packages_group.setLayout(packages_layout)
        splitter.addWidget(packages_group)

        # Log panel
        self.log_group = QGroupBox('构建日志')
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit(readOnly=True)
        self.log_text.setPlaceholderText('编译输出将在此显示...')
        log_layout.addWidget(self.log_text)
        self.log_group.setLayout(log_layout)
        splitter.addWidget(self.log_group)
        # 默认让包列表占据更大的横向空间
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 2)
        try:
            splitter.setSizes([800, 400])
        except Exception:
            pass
        layout.addWidget(splitter)

        # Build options row
        options_layout = QHBoxLayout()
        options_layout.setContentsMargins(0, 6, 0, 6)
        self.symlink_check = QCheckBox('符号链接安装 (symlink)')
        self.symlink_check.setChecked(self.config.get('symlink_install', True))
        options_layout.addWidget(self.symlink_check)

        self.always_on_top_btn = QPushButton('置顶窗口')
        self.always_on_top_btn.setCheckable(True)
        self.always_on_top_btn.setChecked(self.config.get('always_on_top', False))
        self.always_on_top_btn.clicked.connect(self.toggle_always_on_top)
        options_layout.addWidget(self.always_on_top_btn)

        options_layout.addWidget(QLabel('并行线程'))
        self.workers_spin = QSpinBox()
        self.workers_spin.setMinimum(1)
        max_workers = os.cpu_count() or 32
        self.workers_spin.setMaximum(max_workers)
        self.workers_spin.setValue(int(self.config.get('parallel_workers', max_workers)))
        self.workers_spin.valueChanged.connect(self.save_config)
        options_layout.addWidget(self.workers_spin)

        options_layout.addWidget(QLabel('主题'))
        self.theme_combo = QComboBox()
        self.theme_combo.addItem('浅色', 'light')
        self.theme_combo.addItem('深色', 'dark')
        idx = self.theme_combo.findData(self.theme_name)
        if idx >= 0:
            self.theme_combo.setCurrentIndex(idx)
        self.theme_combo.currentIndexChanged.connect(self._on_theme_changed)
        options_layout.addWidget(self.theme_combo)

        # push primary actions to the far right
        options_layout.addStretch(1)

        # Secondary + Primary buttons (same row, aligned height)
        buttons_row = QHBoxLayout()
        buttons_row.setSpacing(8)

        self.clean_secondary_btn = QPushButton('清理')
        self.clean_secondary_btn.setObjectName('cleanSecondaryBtn')
        self.clean_secondary_btn.setFixedHeight(36)
        self.clean_secondary_btn.setMinimumWidth(120)
        self.clean_secondary_btn.clicked.connect(self.clean_workspace)
        buttons_row.addWidget(self.clean_secondary_btn)

        self.build_primary_btn = QPushButton('编译所选')
        self.build_primary_btn.setObjectName('buildPrimaryBtn')
        self.build_primary_btn.setFixedHeight(36)
        self.build_primary_btn.setMinimumWidth(140)
        self.build_primary_btn.clicked.connect(self.build_package)
        buttons_row.addWidget(self.build_primary_btn)

        options_layout.addLayout(buttons_row)

        layout.addLayout(options_layout)

        # Status bar
        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self.progress = QProgressBar()
        self.progress.setMaximum(0)
        self.progress.setVisible(False)
        self.status.addPermanentWidget(self.progress)

        # Apply stored always-on-top
        self.always_on_top = self.config.get('always_on_top', False)
        if self.always_on_top:
            self.set_always_on_top(True)

        # footer removed; build button lives in options row for aligned height

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

    def _create_toolbar_actions(self):
        """创建工具栏动作并绑定。"""
        style = self.style()

        act_select_ws = QAction(style.standardIcon(QStyle.SP_DirOpenIcon), '选择工作空间', self)
        act_select_ws.triggered.connect(self.select_workspace)
        self.toolbar.addAction(act_select_ws)

        act_refresh = QAction(style.standardIcon(QStyle.SP_BrowserReload), '刷新', self)
        act_refresh.triggered.connect(self.refresh_packages)
        self.toolbar.addAction(act_refresh)

        self.toolbar.addSeparator()

        act_select_all = QAction(style.standardIcon(QStyle.SP_DialogYesButton), '全选', self)
        act_select_all.triggered.connect(self.select_all_packages)
        self.toolbar.addAction(act_select_all)

        act_deselect_all = QAction(style.standardIcon(QStyle.SP_DialogNoButton), '全不选', self)
        act_deselect_all.triggered.connect(self.deselect_all_packages)
        self.toolbar.addAction(act_deselect_all)

        self.toolbar.addSeparator()

        self.act_build = QAction(style.standardIcon(QStyle.SP_ArrowRight), '编译所选', self)
        self.act_build.triggered.connect(self.build_package)
        self.toolbar.addAction(self.act_build)

        self.act_stop = QAction(style.standardIcon(QStyle.SP_BrowserStop), '停止编译', self)
        self.act_stop.triggered.connect(self.stop_build)
        self.act_stop.setEnabled(False)
        self.toolbar.addAction(self.act_stop)

        self.toolbar.addSeparator()

        act_clean = QAction(style.standardIcon(QStyle.SP_DialogResetButton), '清理', self)
        act_clean.triggered.connect(self.clean_workspace)
        self.toolbar.addAction(act_clean)

    def _on_theme_changed(self):
        """主题切换处理。"""
        theme = self.theme_combo.currentData()
        self.theme_name = theme
        self.apply_theme(theme)
        self.save_config()

    def apply_theme(self, theme_name: str):
        """应用主题（light/dark）。"""
        try:
            share_dir = get_package_share_directory('workspace_manager')
            qss_name = 'style_dark.qss' if theme_name == 'dark' else 'style_light.qss'
            qss_path = os.path.join(share_dir, 'gui', qss_name)
            if os.path.exists(qss_path):
                with open(qss_path, 'r', encoding='utf-8') as f:
                    self.setStyleSheet(f.read())
            else:
                self.setStyleSheet('')
        except Exception as exc:
            print(f'Failed to apply theme: {exc}')

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

        # 清空现有的表格条目
        try:
            self.packages_table.setRowCount(0)
        except Exception:
            pass
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

        # 填充表格
        for package_name in sorted(available_packages.keys()):
            row = self.packages_table.rowCount()
            self.packages_table.insertRow(row)

            # 选择列：复选框
            checkbox = QCheckBox()
            if package_name in self.config.get('last_selected_packages', []):
                checkbox.setChecked(True)
            checkbox.stateChanged.connect(
                lambda state, pkg=package_name: self.on_package_checkbox_changed(pkg, state)
            )
            self.packages_table.setCellWidget(row, 0, checkbox)

            # 包名列
            item = QTableWidgetItem(package_name)
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            self.packages_table.setItem(row, 1, item)

            self.package_checkboxes[package_name] = checkbox

        # 应用搜索过滤
        try:
            self._apply_search_filter(self.search_edit.text())
        except Exception:
            pass


    def build_package(self):
        """使用 QProcess 启动编译并将日志实时输出到右侧面板。"""
        if not self.workspace_root:
            QMessageBox.warning(self, 'Error', 'Please select workspace first!')
            return

        selected_packages = [
            pkg for pkg, cb in self.package_checkboxes.items() if cb.isChecked()
        ]
        if not selected_packages:
            QMessageBox.warning(self, 'Error', 'Please select at least one package!')
            return

        cmd = ['colcon', 'build']
        if self.symlink_check.isChecked():
            cmd.append('--symlink-install')
        workers = int(self.config.get('parallel_workers', os.cpu_count() or 8))
        cmd.extend(['--parallel-workers', str(workers)])
        cmd.extend(['--packages-select'])
        cmd.extend(selected_packages)

        # 准备 UI
        self.log_text.clear()
        self.status.showMessage('开始编译...')
        self.progress.setVisible(True)
        self._set_building_ui_state(True)

        # 启动进程
        self.build_process = QProcess(self)
        self.build_process.setProgram(cmd[0])
        self.build_process.setArguments(cmd[1:])
        self.build_process.setWorkingDirectory(self.workspace_root)
        self.build_process.readyReadStandardOutput.connect(self._read_build_stdout)
        self.build_process.readyReadStandardError.connect(self._read_build_stderr)
        self.build_process.finished.connect(self._on_build_finished)
        self.build_process.errorOccurred.connect(self._on_build_error)
        self.build_process.start()
        if not self.build_process.waitForStarted(3000):
            self._append_log('[错误] 无法启动构建进程。')
            self.progress.setVisible(False)
            self._set_building_ui_state(False)

    def stop_build(self):
        """停止当前编译。"""
        if self.build_process and self.build_process.state() != QProcess.NotRunning:
            self.build_process.kill()
            self.build_process.waitForFinished(1000)
            self.status.showMessage('已停止编译')
            self.progress.setVisible(False)
            self._set_building_ui_state(False)

    def _append_log(self, text: str):
        self.log_text.append(text.rstrip())

    def _read_build_stdout(self):
        data = bytes(self.build_process.readAllStandardOutput()).decode('utf-8', 'ignore')
        for line in data.splitlines():
            self._append_log(line)

    def _read_build_stderr(self):
        data = bytes(self.build_process.readAllStandardError()).decode('utf-8', 'ignore')
        for line in data.splitlines():
            self._append_log(f"[ERR] {line}")

    def _on_build_finished(self, code: int, _status):
        self.progress.setVisible(False)
        self._set_building_ui_state(False)
        if code == 0:
            self.status.showMessage('编译成功')
            QMessageBox.information(self, 'Success', 'Build completed successfully!')
            self.save_config()
        else:
            self.status.showMessage('编译失败')
            QMessageBox.critical(self, 'Error', 'Build failed. 请查看日志。')

    def _on_build_error(self, _err):
        self.progress.setVisible(False)
        self._set_building_ui_state(False)
        self.status.showMessage('构建进程启动失败')
        QMessageBox.critical(self, 'Error', 'Failed to start build process.')

    def _set_building_ui_state(self, building: bool):
        # 在构建期间禁用部分控件
        if hasattr(self, 'act_build'):
            self.act_build.setEnabled(not building)
        if hasattr(self, 'act_stop'):
            self.act_stop.setEnabled(building)
        if hasattr(self, 'build_primary_btn'):
            self.build_primary_btn.setEnabled(not building)
        if hasattr(self, 'clean_secondary_btn'):
            self.clean_secondary_btn.setEnabled(not building)
        self.symlink_check.setEnabled(not building)
        self.workers_spin.setEnabled(not building)
        if hasattr(self, 'packages_table'):
            self.packages_table.setEnabled(not building)
        for cb in self.package_checkboxes.values():
            cb.setEnabled(not building)


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

    def _apply_search_filter(self, text: str):
        """根据输入文本过滤包列表（不区分大小写）。"""
        text = (text or '').strip().lower()
        if hasattr(self, 'packages_table'):
            for row in range(self.packages_table.rowCount()):
                name_item = self.packages_table.item(row, 1)
                name = name_item.text() if name_item else ''
                visible = (text in name.lower()) if text else True
                self.packages_table.setRowHidden(row, not visible)

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
