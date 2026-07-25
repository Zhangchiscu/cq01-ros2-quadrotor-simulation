# CQ01 加分项实验套件

## 1. 目的

该套件统一管理三组重庆完整闭环实验：

1. nominal + truth odom；
2. heavy + truth odom；
3. nominal + noisy odom/IMU closed loop。

三组实验使用相同的重庆场景、P0–P15 路线、扰动、动态障碍、
圆形/八字和最终定点评价。

多无人机已按最终决定排除，不属于本套件失败项。

## 2. 只汇总现有结果

```bash
~/drone_task/your_drone_sim_ws/scripts/run_cq01_bonus_suite.sh --aggregate-only
```

## 3. 从头顺序运行三组实验

```bash
~/drone_task/your_drone_sim_ws/scripts/run_cq01_bonus_suite.sh --run-all
```

预计需要约 20～30 分钟。运行前必须关闭其他 CQ01 和四旋翼节点。

## 4. 最终审计

```bash
~/drone_task/your_drone_sim_ws/scripts/check_cq01_all_bonus_items.sh
```

通过标志：

```text
ALL BONUS ITEMS RESULT: pass=True; failed=[]
```

## 5. 最终归档

归档目录位于：

```text
~/drone_task/your_drone_sim_ws/results/cq01_bonus_suite/
cq01_final_bonus_package_<时间戳>/
```

同时会在共享目录生成同名 `.tar.gz`。
