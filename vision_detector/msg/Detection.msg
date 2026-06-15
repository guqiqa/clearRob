# Detection.msg — 单个检测目标
string   class_name          # 类别名称
float32  confidence          # 置信度 (0.0~1.0)
uint16   class_id            # 类别 ID
float32  x1                   # 边界框左上 x (像素坐标)
float32  y1                   # 边界框左上 y (像素坐标)
float32  x2                   # 边界框右下 x (像素坐标)
float32  y2                   # 边界框右下 y (像素坐标)
float32  pixel_area           # 像素面积
