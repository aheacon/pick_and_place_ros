## 1. Getting started with NVIDIA Isaac Notes
### Installation
- [Isaac Sym min requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html#isaac-sim-app-install-workstation)
  <details closed>
  <summary> My system specifications </summary>

  ```bash
  # GPU GeForce GTX 1060 6GB
  $ sudo lshw -C display
        description: VGA compatible controller
        product: GP106 [GeForce GTX 1060 6GB]
        vendor: NVIDIA Corporation

  # CPU
  $ lscpu # middle (8 cores, 16 ideal)
  Architecture:             x86_64
    CPU op-mode(s):         32-bit, 64-bit
    Address sizes:          43 bits physical, 48 bits virtual
    Byte Order:             Little Endian
  CPU(s):                   16
    On-line CPU(s) list:    0-15
  Vendor ID:                AuthenticAMD
    Model name:             AMD Ryzen 7 2700X Eight-Core Processor

  # RAM (32GB) # minimum
  $ grep MemTotal /proc/meminfo 
  MemTotal:       32765800 kB
  $ free --mega
                total        used        free      shared  buff/cache   available
  Mem:           33552        5336       23477         255        4738       27485

  # Check FPS - For Navigation at least `30fps`
  $ glxgears
  Running synchronized to the vertical refresh.  The framerate should be
  approximately the same as the monitor refresh rate.
  298 frames in 5.0 seconds = 59.571 FPS
  300 frames in 5.0 seconds = 59.951 FPS
  ```
  </details>

- [Tutorial YT](https://www.youtube.com/watch?v=SyrsAd8WbCo&list=PL15gNY53iTm_J9fVmmNQO0tiqibH_bc-g&index=7)
  - `Omniverse` is the platform of tools
    - `Isaac Sim` is one of the tools (robotics, digital twins)
    - `Omniverse launcher`
      - Gives access to all of tools
      - Omniverse Launcher is supported until end of 2024.
      - Gateway to apps, `Connectors`, utilites
      - [Getting started](https://developer.nvidia.com/omniverse#section-getting-started)
      - [Worstatation insallation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html#isaac-sim-app-install-workstation)
      - [Getting started with launcher](https://docs.omniverse.nvidia.com/launcher/latest/installing_launcher.html)
      - [System monitor](https://docs.omniverse.nvidia.com/utilities/latest/system_monitor.html)
        - I cannot see it on Linux `Omniverse system monitor`)
      - Download `launcher` (`omniverse-launcher-linux.AppImage`)
      - `$ sudo chmod +x omniverse-launcher-linux.AppImage` and doubleclike
      - Install [Nucleus](https://docs.omniverse.nvidia.com/nucleus/latest/workstation/installation.html) and [Cache](https://docs.omniverse.nvidia.com/utilities/latest/cache/installation/workstation.html)
      - Library `~/.local/share/ov/pkg`, data `~/.local/share/ov/data`, cache ``~/.cache/ov`,
      - `Exchange` -> `Isaac Sim` -> Install (4.2.0 release) 9GB -> `Launch`
      - `Nucleus` -> `Add Local Nuceleus services` (345MB) `(~/.local/share/ov/data`)-> check http://localhost:34080 (Nucleus), system monitor (http://localhost:3080/)
      - `Exchange`-> `Cache` -> Install (2023.2.5) -> `Launch` -> check http://localhost:3080/
      - Before launching the `Isaac Sim`, there is another APp ` Isaac Sim Compatibility checker` (245MB) and launch it.
        - As expected only GPU 0 is red, other things are green.
      <details closed>
      <summary> Warning when launching</summary>

      Warning
      ```
      An input-output memory management unit (IOMMU) appears to be enabled on this system.
      On bare-metal Linux systems, CUDA and the display driver do not support IOMMU-enabled PCIe peer to peer memory copy.
      If you are on a bare-metal Linux system, please disable the IOMMU. Otherwise you risk image corruption and program instability.
      This typically can be controlled via BIOS settings (Intel Virtualization Technology for Directed I/O (VT-d) or AMD I/O Virtualization Technology (AMD-Vi)) and kernel parameters (iommu, intel_iommu, amd_iommu).
      Note that in virtual machines with GPU pass-through (vGPU) the IOMMU needs to be enabled.
      Since we can not reliably detect whether this system is bare-metal or a virtual machine, we show this warning in any case when an IOMMU appears to be enabled.
      ```
      - The same happens when Isaac Sim is started
      ```bash
      [Warning] [gpu.foundation.plugin] IOMMU is enabled.
      [Warning] [omni.gpu_foundation_factory.plugin] RT-capable GPU not found, switching to compatibility mode
      ```
      - Check commands
      ```bash
      $  sudo dmesg | grep -i virtual
      [    0.063843] Booting paravirtualized kernel on bare hardware
      [    0.538075] AMD-Vi: Virtual APIC enabled
      [    6.483996] systemd[1]: Unnecessary job was removed for /sys/devices/virtual/misc/vmbus!hv_fcopy.
      [    6.484002] systemd[1]: Unnecessary job was removed for /sys/devices/virtual/misc/vmbus!hv_vss.
      [    7.329427] kvm_amd: Nested Virtualization enabled
      [    7.329430] kvm_amd: LBR virtualization supported
      [    7.329456] kvm_amd: Virtual VMLOAD VMSAVE supported
      [    7.329458] kvm_amd: Virtual GIF supported

      $ sudo dmesg | grep -e DMAR -e IOMMU
      [    0.532264] pci 0000:00:00.2: AMD-Vi: IOMMU performance counters supported
      [    0.538177] perf/amd_iommu: Detected AMD IOMMU #0 (2 banks, 4 counters/bank)
      
      $ ls /sys/kernel/iommu_groups/
      $ for d in /sys/kernel/iommu_groups/*/devices/*; do n=${d#*/iommu_groups/*}; n=${n%%/*}; printf 'IOMMU group %s ' "$n"; lspci -nns "${d##*/}"; done
      $ lsmod

      # Try disable SVM mode
      # Update grup
      $ sudo bash -c 'echo GRUB_CMDLINE_LINUX="amd_iommu=off" >> /etc/default/grub' && sudo update-grup && sudo reboot


      ```

      - NVIDIA Commands
      ```bash
      $ nvidia-sim # sysetm managment interface
      ```
      </details>

### Getting started - Isaac Sim Interface
[Tutorial Nvidia Isaac Sim Interface](https://docs.omniverse.nvidia.com/isaacsim/latest/introductory_tutorials/tutorial_intro_interface.html#isaac-sim-app-tutorial-intro-interface)

### How to import robots into Isaac Sym
- [Tutorial YT](https://www.youtube.com/watch?v=pxPFr58gHmQ&list=PL15gNY53iTm_J9fVmmNQO0tiqibH_bc-g&index=5)
- Importers:
- Based on Universal Scene Description (USD) -3D scene description API
1. URDF (Unified Robotics Description Format)
- imported with physics, joints automatically added
- [automatic addison ros2](https://automaticaddison.com/how-to-model-a-robotic-arm-with-a-urdf-file-ros-2/)
  - [How to model a robotic arm with urdf file ros2](https://automaticaddison.com/how-to-model-a-robotic-arm-with-a-urdf-file-ros-2/) TODO
  - [Based on ROS2 tutorial](https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/URDF-Main.html)
  - [mycobot_ros2 addison](https://github.com/automaticaddison/mycobot_ros2)

2. Step - need manually setup physics
3. OnShape - need manually setup physics
4. MJCF (MuJoCo XML Format)

- Start `Isaac Sim` - > 
### 1.1. Literature 📖
[NVIDIA Isaac ]()
[Nvidia Isaac Gym - deprecated](https://www.hackster.io/Elephant-Robotics-Official/mycobot-gripping-task-reinforcement-learning-with-isaac-gym-5621db)
[Nvidia on demand tutorials](https://www.nvidia.com/en-us/on-demand/search/?facet.event_name[]=Omniverse&facet.event_sessionType[]=Tutorial&facet.mimetype[]=event%20session&headerText=Omniverse%20%20Tutorials&layout=list&page=1&q=-&sort=relevance&sortDir=desc)
[Demo schens n nvidia and digital twins (other video)](https://www.youtube.com/watch?v=YEK10l19cAg)

### Other
https://www.hackster.io/Elephant-Robotics-Official/mycobot-gripping-task-reinforcement-learning-with-isaac-gym-5621db
https://www.hackster.io/Elephant-Robotics-Official/exploring-the-power-of-mycobot-280-for-jetson-nano-05b035 
https://www.hackster.io/gary26/hand-gestures-as-the-remote-controlling-mycobot-320-5dd749
https://www.linkedin.com/posts/ahcorde_ros-ros2-gazebo-activity-7249773121124130816-ERKg?utm_source=share&utm_medium=member_desktop
https://github.com/robotology/gym-ignition 
https://www.youtube.com/watch?v=OcqzPEJWnms
https://caiobarrosv.github.io/
https://www.youtube.com/watch?v=aJ39MruDdLo
https://github.com/lar-deeufba/ssggcnn_ur5_grasping
https://www.linkedin.com/pulse/innovative-applications-robotic-arms-integrated-software/
https://larics.fer.hr/
https://github.com/larics
https://github.com/larics/trt_pose
https://github.com/larics/ros_openpose
https://developer.nvidia.com/deep-learning-frameworks
https://github.com/adnan-saood/ros-course-examples/blob/27d114774fb9fac8670a44a2cfd482e2c1178dbd/docs/Examples/E2.markdown