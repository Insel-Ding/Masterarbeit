
import tkinter as tk
import open3d as o3d

from plyfile import PlyData

# 读取PLY文件
ply = o3d.io.read_point_cloud(r'C:\Users\dings\Documents\Masterarbeit\models\cropped_1.ply')

# 创建Tkinter窗口
root = tk.Tk()

# 创建Tkinter画布
canvas = tk.Canvas(root, width=500, height=500)
canvas.pack()

# 遍历PLY模型中的所有面
for point in ply.points:
    # 获取面的顶点坐标列表
    vertices = ply['vertex']
    # 使用Tkinter的create_polygon方法在画布上绘制多边形
    canvas.create_polygon(vertices, outline='black', fill='gray', width=1)

# 运行窗口
root.mainloop()