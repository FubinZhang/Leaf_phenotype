import pycv

# 创建scan2对象
scanner = pycv.scan2()

# 调用Process_src函数处理图像
result = scanner.Process_src("leaf.png")

# 输出处理结果
print("Leaf color:", result.leaf_color)
print("Leaf length:", result.leaf_length)
# 其他属性以此类推

# 如果有错误信息
if result.ERROS:
    print("Error:", result.ERROS)