import subprocess
import time


class LaunchManager:
    def __init__(self):
        self.roscore_process = None
        self.node_process = None

        self.start_roscore()
        time.sleep(3)
        self.set_use_sim_time()


    def start_roscore(self):
        self.roscore_process = subprocess.Popen(['roscore'])
        print("正在启动 roscore...")

    def set_use_sim_time(self, use=False):
        """是否使用模拟时间

        /use_sim_time 是一个特殊的 ROS 全局参数，用于控制是否使用模拟时间。

        Args:
            use (bool): 是否使用模拟时间。默认值为 False，使用系统时间（真实时间）。设置为True，则ROS会从/clock话题中读取时间（用于仿真环境）。

        Raises:
            subprocess.CalledProcessError: 如果命令执行失败。
            FileNotFoundError: 如果命令 'rosparam' 未找到。
        """
        try:
            # 执行命令，设置 ROS 参数
            subprocess.run(
                ["rosparam", "set", "/use_sim_time", f"{use}"],
                check=True,  # 如果命令失败则抛出异常
                text=True    # 输出自动解码为字符串
            )
            print("ROS parameter '/use_sim_time' set to false successfully.")
        except subprocess.CalledProcessError as e:
            print(f"Failed to set ROS parameter: {e}")
        except FileNotFoundError:
            print("Command 'rosparam' not found. Ensure ROS is installed and sourced.")

    def start_gmapping(self):
        pass

    def start_mobi_con(self):
        pass

    def start_rplidar_s2(self):
        pass

    def start_joystick_controller(self):
        pass


if __name__ == '__main__':
    try:
        # 启动 roscore
        roscore_process = subprocess.Popen(['roscore'])
        print("正在启动 roscore...")
        time.sleep(5)  # 等待 roscore 启动完成

        # 启动单个 ROS 节点并传递参数
        print("正在启动节点...")
        node_process = subprocess.Popen([
            'rosrun', 'example_package', 'example_node',
            '_example_param:=example_value'
        ])

        # 启动 launch 文件
        node_process = subprocess.Popen(
            ['roslaunch', 'package_name', 'launch_file.launch'])

        # 可以通过 stdout 和 stderr 参数捕获节点的标准输出和错误输出
        node_process = subprocess.Popen(
            ['rosrun', 'package_name', 'node_name'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        for line in node_process.stdout:
            print(line.decode(), end='')

        # 等待节点被终止
        node_process.wait()
    except KeyboardInterrupt:
        print("捕获到中断信号，停止所有进程...")
        roscore_process.terminate()
        node_process.terminate()
        roscore_process.wait()
        node_process.wait()
