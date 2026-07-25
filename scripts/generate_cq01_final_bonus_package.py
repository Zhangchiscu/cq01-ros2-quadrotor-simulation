#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


WORKSPACE = Path.home() / "drone_task/your_drone_sim_ws"
RESULT_ROOT = WORKSPACE / "results/cq01_bonus_suite"
SOURCE_ROOT = WORKSPACE / "src"
SCRIPTS_ROOT = WORKSPACE / "scripts"
DOCS_ROOT = WORKSPACE / "docs"
VM_SHARE = Path("/mnt/hgfs/VMShare")


def load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def latest_directory(
    root: Path,
    prefix: str,
    required_file: str,
) -> Optional[Path]:
    candidates = sorted(
        (
            path
            for path in root.iterdir()
            if path.is_dir()
            and path.name.startswith(prefix)
            and (path / required_file).is_file()
        ),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    return candidates[0] if candidates else None


def latest_parameter_comparison(root: Path) -> Optional[Path]:
    return latest_directory(
        root,
        "parameter_profile_comparison_",
        "parameter_profile_summary.json",
    )


def safe_bool(
    payload: Optional[Dict[str, Any]],
    *keys: str,
) -> bool:
    current: Any = payload
    for key in keys:
        if not isinstance(current, dict):
            return False
        current = current.get(key)
    return current is True


def copy_evidence(
    source_dir: Path,
    destination_dir: Path,
    filenames: Iterable[str],
    records: List[Dict[str, str]],
    category: str,
) -> None:
    destination_dir.mkdir(parents=True, exist_ok=True)
    for filename in filenames:
        source = source_dir / filename
        if not source.is_file():
            continue
        destination = destination_dir / filename
        shutil.copy2(source, destination)
        records.append({
            "category": category,
            "source": str(source),
            "packaged_path": str(
                destination.relative_to(destination_dir.parents[1])
            ),
            "size_bytes": str(destination.stat().st_size),
        })


def detect_reference_roots() -> Dict[str, List[str]]:
    detected: Dict[str, List[str]] = {
        "pengyu_sim": [],
        "MARSIM": [],
    }

    search_roots = [
        WORKSPACE,
        WORKSPACE.parent,
        Path.home() / "drone_task",
    ]

    seen: set[Path] = set()
    for root in search_roots:
        if not root.is_dir():
            continue
        for path in root.rglob("*"):
            if not path.is_dir():
                continue
            try:
                resolved = path.resolve()
            except OSError:
                continue
            if resolved in seen:
                continue
            seen.add(resolved)

            lowered = path.name.lower()
            if "pengyu" in lowered:
                detected["pengyu_sim"].append(str(path))
            if "marsim" in lowered or "mar_sim" in lowered:
                detected["MARSIM"].append(str(path))

            if sum(len(values) for values in detected.values()) >= 20:
                break

    for key in detected:
        detected[key] = sorted(set(detected[key]))[:10]
    return detected


def package_file_list(root: Path) -> List[str]:
    package_files: List[str] = []
    if not SOURCE_ROOT.is_dir():
        return package_files

    for manifest in sorted(SOURCE_ROOT.rglob("package.xml")):
        try:
            package_files.append(
                str(manifest.parent.relative_to(SOURCE_ROOT))
            )
        except ValueError:
            package_files.append(str(manifest.parent))
    return package_files


def markdown_table(rows: List[Tuple[str, str, str]]) -> str:
    output = [
        "| 项目 | 当前实现 | 证据 |",
        "|---|---|---|",
    ]
    for item, status, evidence in rows:
        output.append(f"| {item} | {status} | {evidence} |")
    return "\n".join(output)


def main() -> int:
    if not RESULT_ROOT.is_dir():
        print(f"[FAIL] 结果目录不存在：{RESULT_ROOT}")
        return 2

    nominal_dir = latest_directory(
        RESULT_ROOT,
        "nominal_full_",
        "bonus_summary.json",
    )
    heavy_dir = latest_directory(
        RESULT_ROOT,
        "heavy_full_",
        "bonus_summary.json",
    )
    sensor_dir = latest_directory(
        RESULT_ROOT,
        "nominal_nominal_closed_loop_full_",
        "bonus_summary.json",
    )
    parameter_dir = latest_parameter_comparison(RESULT_ROOT)

    nominal = (
        load_json(nominal_dir / "bonus_summary.json")
        if nominal_dir else None
    )
    heavy = (
        load_json(heavy_dir / "bonus_summary.json")
        if heavy_dir else None
    )
    sensor = (
        load_json(sensor_dir / "bonus_summary.json")
        if sensor_dir else None
    )
    parameter = (
        load_json(
            parameter_dir / "parameter_profile_summary.json"
        )
        if parameter_dir else None
    )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    package_dir = (
        RESULT_ROOT
        / f"cq01_final_bonus_package_{timestamp}"
    )
    package_dir.mkdir(parents=True, exist_ok=True)

    evidence_records: List[Dict[str, str]] = []

    common_files = (
        "bonus_summary.json",
        "experiment_manifest.json",
        "telemetry.csv",
        "position_error.png",
        "altitude.png",
        "maximum_rpm.png",
        "body_clearance.png",
    )

    if nominal_dir:
        copy_evidence(
            nominal_dir,
            package_dir / "evidence/nominal_truth",
            common_files,
            evidence_records,
            "nominal_truth",
        )
    if heavy_dir:
        copy_evidence(
            heavy_dir,
            package_dir / "evidence/heavy_truth",
            common_files,
            evidence_records,
            "heavy_truth",
        )
    if sensor_dir:
        copy_evidence(
            sensor_dir,
            package_dir / "evidence/sensor_closed_loop",
            common_files,
            evidence_records,
            "sensor_closed_loop",
        )
    if parameter_dir:
        copy_evidence(
            parameter_dir,
            package_dir / "evidence/parameter_comparison",
            (
                "parameter_profile_summary.json",
                "parameter_profile_rpm_comparison.png",
                "parameter_profile_altitude_comparison.png",
            ),
            evidence_records,
            "parameter_comparison",
        )

    source_evidence_dir = package_dir / "evidence/source"
    source_evidence_dir.mkdir(parents=True, exist_ok=True)

    source_candidates = {
        "v9_bonus_dashboard.py":
            SOURCE_ROOT / "cq01_v9_mc/cq01_v9_mc/v9_bonus_dashboard.py",
        "v9_bonus_recorder.py":
            SOURCE_ROOT / "cq01_v9_mc/cq01_v9_mc/v9_bonus_recorder.py",
        "v9_sensor_suite.py":
            SOURCE_ROOT / "cq01_v9_mc/cq01_v9_mc/v9_sensor_suite.py",
        "v9_sensor_chain_guard.py":
            SOURCE_ROOT / "cq01_v9_mc/cq01_v9_mc/v9_sensor_chain_guard.py",
        "v9_profile_guard.py":
            SOURCE_ROOT / "cq01_v9_mc/cq01_v9_mc/v9_profile_guard.py",
        "v9_high_altitude_wind.py":
            SOURCE_ROOT / "cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py",
        "vehicle_nominal.yaml":
            SOURCE_ROOT / "cq01_v9_mc/config/vehicle_nominal.yaml",
        "vehicle_heavy.yaml":
            SOURCE_ROOT / "cq01_v9_mc/config/vehicle_heavy.yaml",
        "cq01_v9_mc.launch.py":
            SOURCE_ROOT / "cq01_v9_mc/launch/cq01_v9_mc.launch.py",
    }

    for name, source in source_candidates.items():
        if not source.is_file():
            continue
        destination = source_evidence_dir / name
        shutil.copy2(source, destination)
        evidence_records.append({
            "category": "source",
            "source": str(source),
            "packaged_path": str(
                destination.relative_to(package_dir)
            ),
            "size_bytes": str(destination.stat().st_size),
        })

    script_evidence_dir = package_dir / "evidence/scripts"
    script_evidence_dir.mkdir(parents=True, exist_ok=True)
    for name in (
        "run_cq01_final_showcase.sh",
        "run_cq01_nominal_full.sh",
        "run_cq01_heavy_full.sh",
        "run_cq01_sensor_nominal_full.sh",
        "run_cq01_bonus_suite.sh",
        "check_cq01_phase3_strict_after_run.sh",
        "check_cq01_all_bonus_items.sh",
    ):
        source = SCRIPTS_ROOT / name
        if source.is_file():
            destination = script_evidence_dir / name
            shutil.copy2(source, destination)
            evidence_records.append({
                "category": "script",
                "source": str(source),
                "packaged_path": str(
                    destination.relative_to(package_dir)
                ),
                "size_bytes": str(destination.stat().st_size),
            })

    reference_roots = detect_reference_roots()
    packages = package_file_list(SOURCE_ROOT)

    checks = {
        "nominal_truth_full_pass":
            safe_bool(nominal, "pass"),
        "parameter_profile_comparison_pass":
            safe_bool(parameter, "pass"),
        "sensor_closed_loop_full_pass":
            safe_bool(sensor, "pass"),
        "sensor_chain_verified":
            safe_bool(
                sensor,
                "checks",
                "sensor_chain_verified",
            ),
        "profile_verified":
            safe_bool(
                sensor,
                "checks",
                "profile_verified",
            ),
        "h14_retracking_passed":
            safe_bool(
                sensor,
                "checks",
                "h14_high_wind_replan_retracking",
            ),
        "route_and_pattern_passed": (
            safe_bool(
                sensor,
                "checks",
                "route_P0_P15_completed",
            )
            and safe_bool(
                sensor,
                "checks",
                "summit_pattern_passed",
            )
        ),
        "read_only_dashboard_source_present":
            source_candidates[
                "v9_bonus_dashboard.py"
            ].is_file(),
        "unified_recorder_source_present":
            source_candidates[
                "v9_bonus_recorder.py"
            ].is_file(),
        "nominal_and_heavy_configs_present": (
            source_candidates["vehicle_nominal.yaml"].is_file()
            and source_candidates["vehicle_heavy.yaml"].is_file()
        ),
        "one_click_suite_present":
            (SCRIPTS_ROOT / "run_cq01_bonus_suite.sh").is_file(),
        "architecture_comparison_generated": True,
        "multi_uav_excluded_by_user_decision": True,
    }

    required_checks = {
        key: value
        for key, value in checks.items()
        if key != "multi_uav_excluded_by_user_decision"
    }
    failed = [
        key
        for key, passed in required_checks.items()
        if passed is not True
    ]

    metrics = {
        "nominal": (
            nominal.get("metrics", {})
            if isinstance(nominal, dict)
            else {}
        ),
        "heavy_raw_observation": (
            heavy.get("metrics", {})
            if isinstance(heavy, dict)
            else {}
        ),
        "sensor_closed_loop": (
            sensor.get("metrics", {})
            if isinstance(sensor, dict)
            else {}
        ),
        "parameter_comparison": (
            parameter.get("physical_evidence", {})
            if isinstance(parameter, dict)
            else {}
        ),
    }

    final_summary = {
        "pass": not failed,
        "failed_criteria": failed,
        "generated_at":
            datetime.now().isoformat(timespec="seconds"),
        "workspace": str(WORKSPACE),
        "package_dir": str(package_dir),
        "scope":
            "all implemented CQ01 bonus items except multi-UAV",
        "multi_uav_status":
            "excluded_by_user_decision",
        "checks": checks,
        "metrics": metrics,
        "selected_results": {
            "nominal_truth": str(nominal_dir) if nominal_dir else None,
            "heavy_truth": str(heavy_dir) if heavy_dir else None,
            "sensor_closed_loop":
                str(sensor_dir) if sensor_dir else None,
            "parameter_comparison":
                str(parameter_dir) if parameter_dir else None,
        },
        "detected_reference_roots": reference_roots,
        "detected_ros_packages": packages,
    }

    (package_dir / "final_bonus_summary.json").write_text(
        json.dumps(
            final_summary,
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    architecture_text = f"""# CQ01 与参考框架的工程架构对比

## 1. 对比边界

本文件只陈述当前工作空间和项目执行记录能够支持的内容。`pengyu_sim`
与 `MARSIM` 是否作为本机运行时依赖，以工作空间实际检测结果为准。

## 2. 当前 CQ01 实现

当前工程基于 ROS 2 Humble，工作空间中检测到的包包括：

```text
{chr(10).join(packages) if packages else "未检测到 package.xml"}
```

CQ01 已形成以下闭环：

```text
动力学模型
→ 控制器
→ 三维 A* 路径规划
→ 重庆山地/高层建筑场景
→ P0–P15 多阶段任务
→ 风扰动、动态障碍和重新规划
→ 圆形/八字轨迹
→ 统一评价、遥测和只读 GUI
```

此外，系统已经实现两类可切换实验配置：

```text
vehicle profile: nominal / heavy
sensor profile: truth / nominal_closed_loop
```

其中带噪 odom 在 `nominal_closed_loop` 模式下固定进入控制器、A* 与任务节点；
真值 odom 继续用于安全评价、轨迹显示和离线误差计算。

## 3. pengyu_sim

项目记录表明，`pengyu_sim` 是本项目早期四旋翼仿真框架和工程起点。
CQ01 在此基础上扩展了重庆三维场景、P0–P15 任务链、多类扰动、
动态障碍、三维 A*、参数化实验、传感器闭环、GUI 和自动评价。

本机检测到的相关目录：

```text
{chr(10).join(reference_roots["pengyu_sim"]) if reference_roots["pengyu_sim"] else "未在当前工作空间检测到独立 pengyu_sim 目录"}
```

因此，最终提交中应将其表述为“基础框架/上游参考”，而不是将 CQ01
新增模块归属于原始框架。

## 4. MARSIM

项目规划中将 MARSIM 作为复杂场景与传感器仿真的参考框架。
当前 CQ01 最终运行链路以本工作空间中的自有 ROS 2 包为准。

本机检测到的相关目录：

```text
{chr(10).join(reference_roots["MARSIM"]) if reference_roots["MARSIM"] else "未在当前工作空间检测到独立 MARSIM 目录"}
```

若最终仓库没有直接包含 MARSIM 源码或运行依赖，报告应写为
“架构与实验设计参考”，不应写成“直接复现或集成 MARSIM”。

## 5. CQ01 的可辨识增量

相较基础仿真框架，CQ01 当前可单独举证的工程增量包括：

1. 重庆山城地形与密集高层建筑一体化场景；
2. P0–P15 完整路线及多阶段任务编排；
3. 低空/高空强风、爬升扰动、动态障碍与三维 A* 重规划；
4. 圆形和八字轨迹跟踪；
5. nominal/heavy 质量与惯量参数化；
6. 带噪 odom 和简化 IMU，并将带噪状态真实接入反馈闭环；
7. 只读地面站、统一遥测、统计图和严格审计；
8. 一键实验套件及结果归档。

多无人机模块已按用户最终决定从提交范围中排除。
"""

    (package_dir / "architecture_comparison.md").write_text(
        architecture_text,
        encoding="utf-8",
    )

    audit_rows = [
        (
            "标称重庆完整闭环",
            "PASS" if checks["nominal_truth_full_pass"] else "FAIL",
            str(nominal_dir) if nominal_dir else "缺失",
        ),
        (
            "nominal/heavy 参数化",
            "PASS"
            if checks["parameter_profile_comparison_pass"]
            else "FAIL",
            str(parameter_dir) if parameter_dir else "缺失",
        ),
        (
            "带噪传感器真实闭环",
            "PASS"
            if checks["sensor_closed_loop_full_pass"]
            else "FAIL",
            str(sensor_dir) if sensor_dir else "缺失",
        ),
        (
            "只读 GUI",
            "PASS"
            if checks["read_only_dashboard_source_present"]
            else "FAIL",
            str(source_candidates["v9_bonus_dashboard.py"]),
        ),
        (
            "统一记录器与自动图表",
            "PASS"
            if checks["unified_recorder_source_present"]
            else "FAIL",
            str(source_candidates["v9_bonus_recorder.py"]),
        ),
        (
            "一键实验套件",
            "PASS"
            if checks["one_click_suite_present"]
            else "FAIL",
            str(SCRIPTS_ROOT / "run_cq01_bonus_suite.sh"),
        ),
        (
            "参考架构对比",
            "PASS",
            "architecture_comparison.md",
        ),
        (
            "多无人机",
            "EXCLUDED",
            "用户最终决定取消，不计入失败项",
        ),
    ]

    report_text = f"""# CQ01 最终加分项审计报告

生成时间：{final_summary["generated_at"]}

审计范围：除多无人机外，当前已经实现并形成证据链的 CQ01 加分功能。

## 总结

```text
FINAL BONUS AUDIT: {"PASS" if not failed else "FAIL"}
failed_criteria = {failed}
```

## 项目级审计

{markdown_table(audit_rows)}

## 关键结果

### 标称真值闭环

```json
{json.dumps(metrics["nominal"], ensure_ascii=False, indent=2)}
```

### 重载参数化对比

```json
{json.dumps(metrics["parameter_comparison"], ensure_ascii=False, indent=2)}
```

### 带噪传感器闭环

```json
{json.dumps(metrics["sensor_closed_loop"], ensure_ascii=False, indent=2)}
```

## 提交口径

最终主展示使用标称真值闭环。重载配置作为参数化实验，
带噪 odom/IMU 作为传感器闭环实验。三者均运行同一套重庆任务链。

多无人机已按最终决策取消，不应在报告、视频或答辩中继续宣称完成。
"""

    (package_dir / "final_bonus_report.md").write_text(
        report_text,
        encoding="utf-8",
    )

    with (package_dir / "evidence_index.csv").open(
        "w",
        encoding="utf-8",
        newline="",
    ) as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=(
                "category",
                "source",
                "packaged_path",
                "size_bytes",
            ),
        )
        writer.writeheader()
        writer.writerows(evidence_records)

    archive = package_dir.with_suffix(".tar.gz")
    subprocess.run(
        [
            "tar",
            "-czf",
            str(archive),
            "-C",
            str(package_dir.parent),
            package_dir.name,
        ],
        check=True,
    )

    shared_archive: Optional[Path] = None
    if VM_SHARE.is_dir():
        shared_archive = VM_SHARE / archive.name
        shutil.copy2(archive, shared_archive)

    print("=" * 72)
    print("CQ01 FINAL BONUS PACKAGE")
    print("Package directory:", package_dir)
    print("Archive:", archive)
    if shared_archive:
        print("Shared archive:", shared_archive)
    for key, passed in checks.items():
        state = (
            "EXCLUDED"
            if key == "multi_uav_excluded_by_user_decision"
            else ("PASS" if passed else "FAIL")
        )
        print(f"[{state}] {key}")
    print(
        "FINAL BONUS AUDIT: "
        f"pass={not failed}; failed={failed}"
    )
    print("=" * 72)

    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
