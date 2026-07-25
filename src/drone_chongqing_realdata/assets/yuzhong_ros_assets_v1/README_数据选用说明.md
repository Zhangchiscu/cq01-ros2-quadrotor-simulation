# 重庆渝中 ROS2 地图资产 V1

本包已经把用户上传的 QGIS 数据整理成 ROS2 可直接使用的统一格式。

## 正式运行只需要 3 个文件

1. `dem/yuzhong_dem_clip_32648.tif`
   - 渝中半岛裁剪 DEM
   - EPSG:32648
   - 30 m 分辨率
   - 物理规划高程比例固定为 1.0

2. `vectors/yuzhong_ros_vectors_32648.gpkg`
   - `boundary`
   - `buildings`
   - `roads`
   - `railways`
   - `bridges`
   - `landmarks`
   - `attractions`
   - `planning_roi`
   - 所有图层已经统一为 EPSG:32648

3. `metadata/yuzhong_map_metadata.json`
   - 坐标原点
   - 高程范围
   - ROI 范围
   - 数据数量
   - 建筑高度字段说明

## 推荐规划区域

- 区域：洪崖洞—魁星楼
- 大小：600 m × 600 m
- 建筑数量：109
- 道路数量：89
- 轨道数量：21
- 桥梁要素数量：25
- 地形起伏：91.5 m

这个区域同时具备密集建筑、明显坡地、高差、城市峡谷和重庆地标，适合做：
- 地形高差飞行
- 至少 5 个真实建筑障碍物避障
- 狭窄通道
- 三维 A* 与二维规划对照
- 多目标城市巡检

## 不作为 ROS2 物理输入的文件

- `chongqingyuzhong.qgz`：旧 QGIS 工程
- `yuzhong_display_v1.qgz`：仅用于 QGIS 预览
- `yuzhong_physics_v1.qgz`：其中存在 Qgis2threejs 的垂直夸张设置，不直接作为物理地图
- 所有 `.qto3settings`：仅属于 Qgis2threejs 显示配置
- `yuzhong_dem_raw.tif.tiff`：原始经纬度 DEM，只保留作源数据
- `yuzhong_dem_32648.tif`：未按半岛边界裁剪的备用 DEM
- `OSGeo4W.bat`、`OSGeo4W.ico`：与 ROS2 地图无关
