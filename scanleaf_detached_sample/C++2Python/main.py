import pycv
import cv2
# 创建scan2对象
scanner = pycv.scan2()

src = cv2.imread("leaf.png")
# 调用Process_src函数处理图像
result = scanner.Process_src(src)

# 输出处理结果
print("Leaf color:", result.leaf_color)
print("Leaf length:", result.leaf_length)
print("Leaf width:", result.leaf_width)
print("Leaf length_widith_ratio:", result.length_widith_ratio)
print("Leaf perimeter:", result.leaf_perimeter)
print("Leaf area:", result.leaf_area)
print("Leaf rectangularity:", result.rectangularity)
print("Leaf densification:", result.densification)
print("Leaf circularity:", result.circularity)
print("Leaf sphericity:", result.sphericity)
print("Leaf boundary_energy:", result.boundary_energy)
# 其他属性以此类推

# 如果有错误信息
if result.ERROS:
    print("Error:", result.ERROS)