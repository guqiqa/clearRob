#!/bin/bash
# 只读备份 + 依赖审计（不切换 systemd、不启停服务、不 ip link set、不启动电机）
set +u
TS=$(date +%Y%m%d_%H%M)
BK=/root/backup_systemd_switch_$TS
mkdir -p $BK/{app,v3,scripts,configs,systemd,deps,verification}
log() { echo "$*"; }
log "BK=$BK"

# ---------- 1) v3 app ----------
cp -a /root/chassis_ros2_app_20260921/RemoteControl_WheelPID_V2 $BK/app/ 2>/dev/null || true
cp -a /root/chassis_ros2_app_20260921/main.cpp $BK/app/ 2>/dev/null || true
cp -a /root/chassis_ros2_app_20260921/odometry.cpp /root/chassis_ros2_app_20260921/odometry.h $BK/app/ 2>/dev/null || true
cp -a /root/chassis_ros2_app_20260921/config.xml /root/chassis_ros2_app_20260921/config_emu.xml /root/chassis_ros2_app_20260921/config_fixed.xml $BK/app/ 2>/dev/null || true
cp -a /root/chassis_ros2_app_20260921/config.xml.bak_before_id_dir_fix_20261021 $BK/app/ 2>/dev/null || true
cp -a /root/chassis_ros2_app_20260921/SHA256SUMS /root/chassis_ros2_app_20260921/FREEZE_INFO.txt /root/chassis_ros2_app_20260921/app_web_control.py /root/chassis_ros2_app_20260921/emergency_stop_can.sh $BK/app/ 2>/dev/null || true
cp -a /root/chassis_ros2_app_20260921/package.xml 2>/dev/null $BK/app/ || true

# ---------- 2) v3 patched build + source ----------
cp -a /root/chassis_ros2_nav_v3/build/RemoteControl_WheelPID_V2 $BK/v3/ 2>/dev/null || true
cp -a /root/chassis_ros2_nav_v3/src/* $BK/v3/ 2>/dev/null || true
cp -a /root/chassis_ros2_nav_v3/build/CMakeCache.txt $BK/v3/ 2>/dev/null || true

# ---------- 3) scripts ----------
for f in start_chassis_ros2.sh stop_chassis_ros2.sh start_v4_bench.sh stop_v4_bench.sh run_mission_bench_v4.sh start_nav_hardware_dry_run.sh stop_nav.sh run_mission_bench.sh run_demo_mission.sh sbus_emu.py prep_config.py stability_run.sh audit_v4_interfaces.py v4_rejection_test.py pub_safe_cmd.py sweep_driver.py speed_sweep_v4.sh speed_mission_runs.sh fault_test_v4.sh feedback_inject_test2.sh net_test_v4.sh flat_ground_grid.py record_trace.py ft_parse.py bench_repeat.sh; do
  [ -f /root/ros2_bridge/$f ] && cp -a /root/ros2_bridge/$f $BK/scripts/
done

# ---------- 4) configs / env / launch ----------
cp -a /root/nav_ws/config/*.yaml $BK/configs/ 2>/dev/null || true
cp -a /root/nav_ws/env.sh $BK/configs/ 2>/dev/null || true
cp -a /root/nav_ws/src/config/interface_register.yaml $BK/configs/ 2>/dev/null || true
cp -a /root/nav_ws/install/navigation_ros2/share/navigation_ros2/config/*.yaml $BK/configs/ 2>/dev/null || true
cp -a /root/nav_ws/install/navigation_ros2/share/navigation_ros2/launch/navigation_hardware_dry_run.launch.py $BK/deps/ 2>/dev/null || true
cp -a /root/ros2_bridge/start_v4_bench.sh $BK/deps/ 2>/dev/null || true
cp -a /root/ros2_bridge/start_nav_hardware_dry_run.sh $BK/deps/ 2>/dev/null || true
cp -a /root/ros2_bridge/run_demo_mission.sh $BK/deps/ 2>/dev/null || true
cp -a /root/ros2_bridge/sbus_emu.py $BK/deps/ 2>/dev/null || true
cp -a /root/ros2_bridge/prep_config.py $BK/deps/ 2>/dev/null || true
cp -a /root/nav_ws/docs/*.md $BK/deps/ 2>/dev/null || true

# ---------- 5) systemd evidence (read-only) ----------
{
  echo "### systemctl cat laifei-wheel-controller.service"; systemctl cat laifei-wheel-controller.service 2>&1
  echo; echo "### systemctl is-enabled"; systemctl is-enabled laifei-wheel-controller.service 2>&1
  echo; echo "### systemctl is-active"; systemctl is-active laifei-wheel-controller.service 2>&1
  echo; echo "### unit files matching laifei/wheel/chassis/sbus"; systemctl list-unit-files 2>/dev/null | grep -iE "laifei|wheel|chassis|sbus" || echo NONE
  echo; echo "### loaded units matching"; systemctl list-units --all 2>/dev/null | grep -iE "laifei|wheel|chassis|sbus" || echo NONE
  echo; echo "### find unit files"; find /etc/systemd /lib/systemd /usr/lib/systemd -iname "*laifei*" -o -iname "*chassis*" -o -iname "*wheel*" 2>/dev/null | head -20 || true
  echo; echo "### /etc/systemd/system listing"; ls -la /etc/systemd/system/ 2>/dev/null
  echo; echo "### multi-user.target.wants"; ls -la /etc/systemd/system/multi-user.target.wants/ 2>/dev/null
  echo; echo "### rc.local"; cat /etc/rc.local 2>/dev/null || echo NO_RC_LOCAL
  echo; echo "### crontab"; crontab -l 2>/dev/null || echo NO_CRONTAB
  echo; echo "### greps for controller autostart"; grep -Ril "RemoteControl_WheelPID\|wheel_speed_node\|sbus_emu" /etc/systemd /etc/init.d /etc/rc.local 2>/dev/null | head -10 || echo NONE
} > $BK/systemd/unit_audit.txt 2>&1

# ---------- 6) verification (read-only) ----------
{
  echo "time: $(date -Is)"; echo "host: $(hostname)"; echo "ip: $(hostname -I)"; echo "kernel: $(uname -a)"
  echo; echo "### pgrep residuals"
  pgrep -af "RemoteControl|wheel_speed_node|hardware_adapter|sbus_emu|navigation_node|safety_supervisor|demo_mission_manager" || echo NONE
  echo; echo "### can0 details (read-only)"
  ip -details link show can0 2>&1
  echo; echo "### can0 stats"
  ip -details -statistics link show can0 2>&1
  echo; echo "### eth0 link"
  ip -br link show eth0 2>&1
} > $BK/verification/readonly_state.txt 2>&1

# ---------- 7) hashes ----------
( cd $BK && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum ) > $BK/SHA256SUMS
{
  echo "### v3 original app binary"; sha256sum /root/chassis_ros2_app_20260921/RemoteControl_WheelPID_V2; md5sum /root/chassis_ros2_app_20260921/RemoteControl_WheelPID_V2
  echo "### v3 patched build binary"; sha256sum /root/chassis_ros2_nav_v3/build/RemoteControl_WheelPID_V2; md5sum /root/chassis_ros2_nav_v3/build/RemoteControl_WheelPID_V2
  echo "### v4 binary"; sha256sum /root/chassis_ros2_nav_v4/build/RemoteControl_WheelPID_V2
  echo "### v4 main.cpp"; sha256sum /root/chassis_ros2_nav_v4/src/main.cpp
  echo "### v4 config"; sha256sum /root/chassis_ros2_nav_v4/src/config.xml
  echo "### v4 config_current_mode"; sha256sum /root/chassis_ros2_nav_v4/src/config_current_mode.xml
} > $BK/versions.txt

# ---------- 8) manifest ----------
{
  echo "# v3/systemd 切换备份清单（自动生成）"
  echo "backup_dir: $BK"
  echo "created: $(date -Is)"
  echo "host: $(hostname)  ip: $(hostname -I)"
  echo
  echo "## 备份内容与哈希"
  echo "（完整列表见 $BK/SHA256SUMS）"
  cat $BK/versions.txt
  echo
  echo "## systemd 核对（只读）"
  cat $BK/systemd/unit_audit.txt
  echo
  echo "## 切换前回滚命令（文本记录，未执行）"
  echo "systemctl disable --now laifei-chassis-v4.service  # 若已安装"
  echo "rm -f /etc/systemd/system/laifei-chassis-v4.service"
  echo "systemctl daemon-reload"
  echo "# 当前无 v3 systemd 单元，回滚=移除 v4 单元并恢复手动链路："
  echo "bash /root/ros2_bridge/stop_v4_bench.sh"
  echo "bash /root/ros2_bridge/start_chassis_ros2.sh   # v3 手动启动（sbus_emu + v3 控制器）"
  echo
  echo "## 只读状态快照"
  cat $BK/verification/readonly_state.txt
} > $BK/MANIFEST.txt
cp $BK/MANIFEST.txt /tmp/v3_backup_manifest_20261023.txt
echo "=== backup complete: $BK ==="
echo "--- SHA256SUMS count: $(wc -l < $BK/SHA256SUMS) ---"
echo "--- dirs ---"; find $BK -maxdepth 1 -type d
echo "--- versions ---"; cat $BK/versions.txt
echo "--- unit audit summary ---"; head -30 $BK/systemd/unit_audit.txt
