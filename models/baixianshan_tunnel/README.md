# 白象山实地巷道 Gazebo 模型

源文件：`白象山巷道_展示精修版_管线修订.blend`（289 MB，Blender 5.x
Zstandard 格式）。源文件保持只读，未被修改。

转换约定：

- 原场景使用米制，范围约为 89.4 × 293.8 × 19.4 m；
- 原场景主巷道相机点 `(-10.0, 17.5, 0.7689)` 的地面设为 Gazebo 原点；
- 原场景绕 Z 轴旋转 -90°，使主巷道北向（Blender +Y）成为任务一前向（Gazebo +X）；
- 无人机出生点为 `(0, 0, 0.1404)`，机头沿 +X；
- 岩壁、地面和顶板按 0.08 比例确定性简化；管线、车辆、风筒等基础设施按 0.25 比例简化；
- 四个导出 OBJ 同时用于视觉、MID360 射线和碰撞，三者不存在几何版本漂移；
- 纹理来自 `.blend` 内嵌数据，不依赖源作者的 Windows 绝对路径。

重新导出：

```bash
/home/nuc/.local/opt/blender-portable/blender-5.2.0-linux-x64/blender \
  --background /path/to/白象山巷道_展示精修版_管线修订.blend \
  --python scripts/export_blend_gazebo_map.py -- models/baixianshan_tunnel
```

导出参数、坐标变换、面数和包围盒记录在 `export_metadata.json`。
