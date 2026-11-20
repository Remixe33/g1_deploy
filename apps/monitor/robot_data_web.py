"""统一机器人数据可视化与录制服务（joint_monitor 的公开入口）。

仿真由 sim.py 进程内托管（--source sim 由 sim.py 注入）；实机独立运行：
  python3 apps/monitor/robot_data_web.py --source realsense \
      --camera-config cameras/d435_rgbd.json --domain 0 --interface eth0
"""
from joint_monitor import main

if __name__ == "__main__":
    main()
