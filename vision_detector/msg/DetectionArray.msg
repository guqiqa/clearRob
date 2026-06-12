# DetectionArray.msg — 一帧检测结果
std_msgs/Header  header      # 时间戳 + frame_id
Detection[]      detections  # 检测目标数组
uint32           frame_seq   # 帧序号
float32          inference_ms # 本次推理耗时 (ms)
