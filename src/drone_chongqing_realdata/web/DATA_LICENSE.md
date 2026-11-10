# 数据与许可说明

## OpenStreetMap

本项目通过 Overpass API 获取 OpenStreetMap 数据。建筑轮廓、道路、轨道、
桥梁和景点等 OSM 数据受 Open Database License (ODbL) 约束。

展示和提交时必须保留：

`© OpenStreetMap contributors`

## Cesium OSM 三维建筑

Cesium 模式通过 Cesium ion 加载 Cesium OSM Buildings。建筑数据来源为
OpenStreetMap，Cesium 提供 3D Tiles 服务与 estimatedHeight 等属性。

## Cesium 全球地形

Cesium 模式通过用户自己的 Cesium ion 访问令牌 加载 World Terrain。
令牌仅保存在本机浏览器 localStorage 中，不应提交到 Git。

## MapLibre 地形数据

MapLibre 模式使用 MapLibre 官方 3D Terrain 示例所采用的网络 DEM
TileJSON 服务。该模式用于无令牌覆盖率检查，不作为最终离线交付数据源。

## 真实性边界

- 建筑位置与轮廓来自 OSM；
- OSM 有 height 或 building:levels 时优先使用；
- OSM 无高度时，MapLibre 模式使用带来源标记的类别估计；
- Cesium 模式使用 Cesium OSM Buildings 的 estimatedHeight；
- 本阶段没有真实外墙纹理，也不是摄影测量实景网格。
