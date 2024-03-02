from scan import Scan2

# 创建Scan2实例
scanner = Scan2()

# 使用process_src方法分析图片
leaf = scanner.process_src("./leaf.png")

if leaf.errors != "":
    print(leaf.errors)
else:
    print("ok")

print(leaf.leaf_length)
print(leaf.leaf_width)
print(leaf.length_width_ratio)
print(leaf.leaf_perimeter)
print(leaf.leaf_area)
print(leaf.rectangularity)
print(leaf.densification)
print(leaf.circularity)
print(leaf.sphericity)
print(leaf.boundary_energy)
print(leaf.leaf_color)