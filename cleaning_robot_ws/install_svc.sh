#!/bin/bash
# Phase 1 systemd service installer
cat > /etc/systemd/system/phase1.service << 'SVC'
[Unit]
Description=Phase 1 Chassis Control
After=network.target

[Service]
Type=simple
ExecStartPre=/bin/bash -c "source /opt/ros/humble/setup.bash"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /root/cleaning_robot_ws/install/setup.bash && exec ros2 launch cleaning_robot_bringup phase1_chassis.launch.py simulate:=false"
StandardOutput=append:/tmp/phase1_svc.log
StandardError=append:/tmp/phase1_svc.log
Restart=no
WorkingDirectory=/root/cleaning_robot_ws

[Install]
WantedBy=multi-user.target
SVC

systemctl daemon-reload
systemctl stop LawnMower 2>/dev/null
systemctl stop phase1 2>/dev/null
echo "Service installed. Start with: systemctl start phase1"
echo "Logs: journalctl -u phase1 -f"
echo "Status: systemctl status phase1"
