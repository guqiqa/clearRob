import re, sys

def read(path):
    try:
        return open(path, encoding="utf-8", errors="ignore").read()
    except FileNotFoundError:
        return ""

def odom_x(path):
    text = read(path)
    m = re.search(r"position:\s*\n\s*x:\s*([-0-9.eE]+)", text)
    return float(m.group(1)) if m else None

def field(path, name):
    text = read(path)
    m = re.search(r"^" + re.escape(name) + r":\s*(.+)$", text, re.M)
    return m.group(1).strip() if m else "?"

tag = sys.argv[1] if len(sys.argv) > 1 else ""
x0 = odom_x("/tmp/ft_odom0.txt")
x1 = odom_x("/tmp/ft_odom_pre.txt")
x2 = odom_x("/tmp/ft_odom_post1.txt")
x3 = odom_x("/tmp/ft_odom_post2.txt")
print("odom x: start=%s pre_fault=%s post1=%s post2=%s" % (x0, x1, x2, x3))
if x2 is not None and x3 is not None and x1 is not None:
    print("  moved_before_fault=%.4f m, moved_after_fault=%.4f m" % (abs(x1 - x0) if x0 is not None else -1, abs(x3 - x2)))
print("chassis: control_mode=%s motion_allowed=%s estop=%s can_healthy=%s feedback_valid=%s safety_state=%s fault=%s age=%s" % (
    field("/tmp/ft_state.txt", "control_mode"), field("/tmp/ft_state.txt", "motion_allowed"),
    field("/tmp/ft_state.txt", "emergency_stop"), field("/tmp/ft_state.txt", "can_healthy"),
    field("/tmp/ft_state.txt", "feedback_valid"), field("/tmp/ft_state.txt", "safety_state"),
    field("/tmp/ft_state.txt", "fault_reason"), field("/tmp/ft_state.txt", "command_age_s")))
print("safety:", read("/tmp/ft_safety.txt").strip().splitlines()[-1] if read("/tmp/ft_safety.txt").strip() else "?")
print("mission:", read("/tmp/ft_mission_status.txt").strip().splitlines()[-1] if read("/tmp/ft_mission_status.txt").strip() else "?")
print("v4 tail:")
print("\n".join(read("/tmp/v4_chassis.log").strip().splitlines()[-5:]))
