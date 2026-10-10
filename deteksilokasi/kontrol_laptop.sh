#!/bin/bash
# Kontrol keyboard dari LAPTOP saat membuat peta (dipanggil buat_peta_laptop.sh).
# lewat WiFi/LAN yang sama (192.168.0.x, lihat ~/cyclonedds_laptop.xml).
# Di LattePanda jalankan dulu:
#   ~/"Robot magang/deteksilokasi/scripts/buat_peta.sh"

exec env -i \
    HOME="$HOME" \
    USER="$USER" \
    LOGNAME="$LOGNAME" \
    SHELL=/bin/bash \
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" \
    PATH="/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
    AMENT_PREFIX_PATH="/opt/ros/jazzy" \
    PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages" \
    LD_LIBRARY_PATH="/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib" \
    ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="file://$HOME/cyclonedds_laptop.xml" \
    TERM="${TERM:-xterm}" \
    python3 "$(cd "$(dirname "$0")" && pwd)/scripts/kontrol_keyboard.py"
