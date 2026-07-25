#!/usr/bin/env python3
"""Fetch and convert real OpenStreetMap data for Yuzhong Peninsula."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .common import resolve_web_root


BBOX = {
    "south": 29.505,
    "west": 106.485,
    "north": 29.590,
    "east": 106.605,
}

ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
]

USER_AGENT = (
    "drone-chongqing-realdata/0.1 "
    "(academic ROS2 simulation project)"
)


def empty_collection() -> Dict[str, object]:
    return {
        "type": "FeatureCollection",
        "features": [],
    }


def bbox_string() -> str:
    return (
        f"{BBOX['south']},"
        f"{BBOX['west']},"
        f"{BBOX['north']},"
        f"{BBOX['east']}"
    )


def query_definitions() -> Dict[str, str]:
    bbox = bbox_string()

    return {
        "buildings": f"""
[out:json][timeout:240];
(
  way["building"]({bbox});
  relation["building"]({bbox});
);
out geom;
""".strip(),
        "transport": f"""
[out:json][timeout:240];
(
  way["highway"~"motorway|trunk|primary|secondary|tertiary|residential|service"]({bbox});
  way["railway"]({bbox});
  way["bridge"]({bbox});
);
out geom;
""".strip(),
        "attractions": f"""
[out:json][timeout:180];
(
  nwr["tourism"="attraction"]({bbox});
  nwr["historic"]({bbox});
  nwr["railway"="station"]({bbox});
  nwr["man_made"="bridge"]({bbox});
  nwr["name"~"解放碑|洪崖洞|朝天门|李子坝|鹅岭|十八梯|较场口|魁星楼|白象居|千厮门|来福士"]({bbox});
);
out center;
""".strip(),
    }


def post_query(
    query: str,
    retries_per_endpoint: int = 2,
) -> Tuple[Dict[str, object], str]:
    encoded = urllib.parse.urlencode({
        "data": query,
    }).encode("utf-8")

    errors: List[str] = []

    for endpoint in ENDPOINTS:
        for attempt in range(
            1,
            retries_per_endpoint + 1,
        ):
            request = urllib.request.Request(
                endpoint,
                data=encoded,
                method="POST",
                headers={
                    "User-Agent": USER_AGENT,
                    "Content-Type": (
                        "application/x-www-form-urlencoded"
                    ),
                    "Accept": "application/json",
                },
            )

            try:
                with urllib.request.urlopen(
                    request,
                    timeout=300,
                ) as response:
                    payload = json.loads(
                        response.read().decode("utf-8")
                    )
                return payload, endpoint
            except (
                urllib.error.URLError,
                urllib.error.HTTPError,
                TimeoutError,
                json.JSONDecodeError,
            ) as error:
                errors.append(
                    f"{endpoint} attempt {attempt}: {error}"
                )
                time.sleep(1.5 * attempt)

    raise RuntimeError(
        "All Overpass endpoints failed:\n"
        + "\n".join(errors)
    )


def coordinates_from_geometry(
    geometry: object,
) -> List[List[float]]:
    if not isinstance(geometry, list):
        return []

    coordinates: List[List[float]] = []
    for point in geometry:
        if not isinstance(point, dict):
            continue
        lat = point.get("lat")
        lon = point.get("lon")
        if lat is None or lon is None:
            continue
        coordinates.append([
            float(lon),
            float(lat),
        ])
    return coordinates


def nearly_equal(
    first: Sequence[float],
    second: Sequence[float],
    tolerance: float = 1.0e-7,
) -> bool:
    return (
        abs(float(first[0]) - float(second[0]))
        <= tolerance
        and abs(float(first[1]) - float(second[1]))
        <= tolerance
    )


def close_ring(
    coordinates: List[List[float]],
) -> List[List[float]]:
    if len(coordinates) < 3:
        return []
    output = list(coordinates)
    if not nearly_equal(output[0], output[-1]):
        output.append(list(output[0]))
    return output


def join_segments(
    segments: List[List[List[float]]],
) -> List[List[List[float]]]:
    remaining = [
        list(segment)
        for segment in segments
        if len(segment) >= 2
    ]
    rings: List[List[List[float]]] = []

    while remaining:
        ring = remaining.pop(0)
        changed = True

        while changed and remaining:
            changed = False
            for index, segment in enumerate(remaining):
                if nearly_equal(ring[-1], segment[0]):
                    ring.extend(segment[1:])
                elif nearly_equal(ring[-1], segment[-1]):
                    ring.extend(
                        list(reversed(segment[:-1]))
                    )
                elif nearly_equal(ring[0], segment[-1]):
                    ring = segment[:-1] + ring
                elif nearly_equal(ring[0], segment[0]):
                    ring = (
                        list(reversed(segment[1:]))
                        + ring
                    )
                else:
                    continue

                remaining.pop(index)
                changed = True
                break

        closed = close_ring(ring)
        if closed:
            rings.append(closed)

    return rings


def parse_length_meters(
    value: object,
) -> Optional[float]:
    if value is None:
        return None

    text = str(value).strip().lower()
    if not text:
        return None

    match = re.search(
        r"[-+]?[0-9]*\.?[0-9]+",
        text,
    )
    if match is None:
        return None

    number = float(match.group(0))
    if "ft" in text or "'" in text:
        number *= 0.3048

    if not math.isfinite(number):
        return None
    return max(0.0, number)


def polygon_area_m2(
    ring: Sequence[Sequence[float]],
) -> float:
    if len(ring) < 4:
        return 0.0

    mean_latitude = sum(
        float(point[1])
        for point in ring
    ) / len(ring)

    meters_per_degree_lat = 111_320.0
    meters_per_degree_lon = (
        111_320.0
        * math.cos(math.radians(mean_latitude))
    )

    xy = [
        (
            float(point[0]) * meters_per_degree_lon,
            float(point[1]) * meters_per_degree_lat,
        )
        for point in ring
    ]

    area = 0.0
    for first, second in zip(
        xy,
        xy[1:],
    ):
        area += (
            first[0] * second[1]
            - second[0] * first[1]
        )
    return abs(area) * 0.5


def deterministic_fraction(
    identifier: object,
) -> float:
    digest = hashlib.sha256(
        str(identifier).encode("utf-8")
    ).digest()
    integer = int.from_bytes(
        digest[:4],
        "big",
    )
    return integer / 0xFFFFFFFF


def building_height(
    tags: Dict[str, object],
    area_m2: float,
    identifier: object,
) -> Tuple[float, float, str]:
    recorded = parse_length_meters(
        tags.get("height")
    )
    minimum = parse_length_meters(
        tags.get("min_height")
    )

    if minimum is None:
        min_level = parse_length_meters(
            tags.get("building:min_level")
        )
        minimum = (
            3.0 * min_level
            if min_level is not None
            else 0.0
        )

    if recorded is not None and recorded >= 2.5:
        return (
            max(recorded, minimum + 2.5),
            minimum,
            "osm_height",
        )

    levels = parse_length_meters(
        tags.get("building:levels")
    )
    if levels is not None and levels >= 1.0:
        return (
            max(
                levels * 3.0,
                minimum + 3.0,
            ),
            minimum,
            "osm_levels",
        )

    building_type = str(
        tags.get("building", "")
    ).lower()

    category_defaults = {
        "house": 9.0,
        "detached": 9.0,
        "hut": 4.0,
        "garage": 4.0,
        "garages": 4.0,
        "school": 15.0,
        "university": 21.0,
        "hospital": 24.0,
        "hotel": 30.0,
        "commercial": 33.0,
        "office": 39.0,
        "retail": 18.0,
        "apartments": 30.0,
        "residential": 24.0,
        "industrial": 15.0,
        "warehouse": 12.0,
    }

    default = category_defaults.get(
        building_type,
        18.0,
    )

    if area_m2 >= 1500:
        default = max(default, 36.0)
    elif area_m2 >= 700:
        default = max(default, 27.0)
    elif area_m2 <= 90:
        default = min(default, 12.0)

    variation = (
        deterministic_fraction(identifier)
        - 0.5
    ) * 6.0

    height = max(
        minimum + 3.0,
        default + variation,
    )
    return (
        height,
        minimum,
        "category_estimate",
    )


def feature_properties(
    element: Dict[str, object],
) -> Dict[str, object]:
    tags = dict(
        element.get("tags", {})
        if isinstance(element.get("tags"), dict)
        else {}
    )
    properties: Dict[str, object] = {
        "osm_type": element.get("type"),
        "osm_id": element.get("id"),
    }
    properties.update(tags)
    return properties


def polygon_feature(
    element: Dict[str, object],
    ring: List[List[float]],
    part_index: int = 0,
) -> Optional[Dict[str, object]]:
    closed = close_ring(ring)
    if not closed:
        return None

    properties = feature_properties(element)
    area = polygon_area_m2(closed)
    height, minimum, source = building_height(
        properties,
        area,
        f"{element.get('type')}/"
        f"{element.get('id')}/{part_index}",
    )
    properties.update({
        "height_m": round(height, 2),
        "min_height_m": round(minimum, 2),
        "height_source": source,
        "footprint_area_m2": round(area, 1),
    })

    return {
        "type": "Feature",
        "properties": properties,
        "geometry": {
            "type": "Polygon",
            "coordinates": [closed],
        },
    }


def convert_buildings(
    payload: Dict[str, object],
) -> Dict[str, object]:
    features: List[Dict[str, object]] = []

    for element in payload.get(
        "elements",
        [],
    ):
        if not isinstance(element, dict):
            continue

        element_type = element.get("type")

        if element_type == "way":
            ring = coordinates_from_geometry(
                element.get("geometry")
            )
            feature = polygon_feature(
                element,
                ring,
            )
            if feature is not None:
                features.append(feature)
            continue

        if element_type != "relation":
            continue

        segments: List[List[List[float]]] = []
        for member in element.get(
            "members",
            [],
        ):
            if not isinstance(member, dict):
                continue
            if member.get("role") not in (
                "",
                "outer",
                None,
            ):
                continue
            geometry = coordinates_from_geometry(
                member.get("geometry")
            )
            if geometry:
                segments.append(geometry)

        for part_index, ring in enumerate(
            join_segments(segments)
        ):
            feature = polygon_feature(
                element,
                ring,
                part_index,
            )
            if feature is not None:
                features.append(feature)

    return {
        "type": "FeatureCollection",
        "features": features,
    }


def line_feature(
    element: Dict[str, object],
) -> Optional[Dict[str, object]]:
    coordinates = coordinates_from_geometry(
        element.get("geometry")
    )
    if len(coordinates) < 2:
        return None

    return {
        "type": "Feature",
        "properties": feature_properties(element),
        "geometry": {
            "type": "LineString",
            "coordinates": coordinates,
        },
    }


def convert_transport(
    payload: Dict[str, object],
) -> Dict[str, Dict[str, object]]:
    outputs = {
        "roads": empty_collection(),
        "railways": empty_collection(),
        "bridges": empty_collection(),
    }

    for element in payload.get(
        "elements",
        [],
    ):
        if not isinstance(element, dict):
            continue

        feature = line_feature(element)
        if feature is None:
            continue

        tags = feature["properties"]
        if (
            tags.get("bridge") not in (
                None,
                "no",
                "false",
            )
            or tags.get("man_made") == "bridge"
        ):
            outputs["bridges"]["features"].append(
                feature
            )

        if "railway" in tags:
            outputs["railways"]["features"].append(
                feature
            )

        if "highway" in tags:
            outputs["roads"]["features"].append(
                feature
            )

    return outputs


def element_center(
    element: Dict[str, object],
) -> Optional[List[float]]:
    center = element.get("center")
    if isinstance(center, dict):
        lat = center.get("lat")
        lon = center.get("lon")
        if lat is not None and lon is not None:
            return [
                float(lon),
                float(lat),
            ]

    lat = element.get("lat")
    lon = element.get("lon")
    if lat is not None and lon is not None:
        return [
            float(lon),
            float(lat),
        ]

    geometry = coordinates_from_geometry(
        element.get("geometry")
    )
    if not geometry:
        return None

    return [
        sum(point[0] for point in geometry)
        / len(geometry),
        sum(point[1] for point in geometry)
        / len(geometry),
    ]


def convert_attractions(
    payload: Dict[str, object],
) -> Dict[str, object]:
    features: List[Dict[str, object]] = []
    seen = set()

    for element in payload.get(
        "elements",
        [],
    ):
        if not isinstance(element, dict):
            continue

        center = element_center(element)
        if center is None:
            continue

        properties = feature_properties(element)
        name = properties.get("name")
        key = (
            round(center[0], 7),
            round(center[1], 7),
            str(name),
        )
        if key in seen:
            continue
        seen.add(key)

        features.append({
            "type": "Feature",
            "properties": properties,
            "geometry": {
                "type": "Point",
                "coordinates": center,
            },
        })

    return {
        "type": "FeatureCollection",
        "features": features,
    }


def write_json(
    path: Path,
    payload: object,
) -> None:
    path.write_text(
        json.dumps(
            payload,
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Fetch real OpenStreetMap buildings, roads, railways, "
            "bridges and attractions for Yuzhong Peninsula."
        )
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help=(
            "Output directory. Defaults to the package web/data "
            "directory."
        ),
    )
    parser.add_argument(
        "--keep-raw",
        action="store_true",
        help="Keep raw Overpass JSON responses.",
    )
    arguments = parser.parse_args()

    output_dir = (
        arguments.output_dir.expanduser().resolve()
        if arguments.output_dir
        else resolve_web_root() / "data"
    )
    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    queries = query_definitions()
    raw_payloads: Dict[str, Dict[str, object]] = {}
    endpoint_by_query: Dict[str, str] = {}

    for index, (name, query) in enumerate(
        queries.items(),
        start=1,
    ):
        print(
            f"[{index}/{len(queries)}] Fetching {name}..."
        )
        payload, endpoint = post_query(query)
        raw_payloads[name] = payload
        endpoint_by_query[name] = endpoint

        if arguments.keep_raw:
            write_json(
                output_dir / f"raw_{name}.json",
                payload,
            )

        print(
            f"  received "
            f"{len(payload.get('elements', []))} elements "
            f"from {endpoint}"
        )

    buildings = convert_buildings(
        raw_payloads["buildings"]
    )
    transport = convert_transport(
        raw_payloads["transport"]
    )
    attractions = convert_attractions(
        raw_payloads["attractions"]
    )

    conversion_diagnostics = {
        "buildings": {
            "raw_count": len(
                raw_payloads["buildings"].get(
                    "elements",
                    [],
                )
            ),
            "converted_count": len(
                buildings["features"]
            ),
        },
        "transport": {
            "raw_count": len(
                raw_payloads["transport"].get(
                    "elements",
                    [],
                )
            ),
            "converted_count": (
                len(transport["roads"]["features"])
                + len(transport["railways"]["features"])
                + len(transport["bridges"]["features"])
            ),
        },
    }

    failed_layers = [
        name
        for name, values in conversion_diagnostics.items()
        if values["raw_count"] > 0
        and values["converted_count"] == 0
    ]

    if failed_layers:
        details = []
        for name in failed_layers:
            elements = raw_payloads[name].get(
                "elements",
                [],
            )
            first_keys = (
                sorted(elements[0].keys())
                if elements
                and isinstance(elements[0], dict)
                else []
            )
            details.append(
                f"{name}: raw="
                f"{conversion_diagnostics[name]['raw_count']}, "
                f"converted=0, first_element_keys="
                f"{first_keys}"
            )

        raise RuntimeError(
            "Overpass returned elements but no usable geometry. "
            "The fetch has been stopped instead of overwriting "
            "GeoJSON with empty layers.\n"
            + "\n".join(details)
        )

    outputs = {
        "buildings.geojson": buildings,
        "roads.geojson": transport["roads"],
        "railways.geojson": transport["railways"],
        "bridges.geojson": transport["bridges"],
        "osm_attractions.geojson": attractions,
    }

    for filename, payload in outputs.items():
        write_json(
            output_dir / filename,
            payload,
        )

    height_sources: Dict[str, int] = {}
    for feature in buildings["features"]:
        source = str(
            feature["properties"].get(
                "height_source",
                "unknown",
            )
        )
        height_sources[source] = (
            height_sources.get(source, 0) + 1
        )

    metadata = {}
    for name, payload in raw_payloads.items():
        osm3s = payload.get("osm3s", {})
        metadata[name] = {
            "endpoint": endpoint_by_query[name],
            "timestamp_osm_base": (
                osm3s.get("timestamp_osm_base")
                if isinstance(osm3s, dict)
                else None
            ),
            "raw_element_count": len(
                payload.get("elements", [])
            ),
        }

    manifest = {
        "scene_revision": (
            "yuzhong_realdata_validation_v1"
        ),
        "bbox_wgs84": BBOX,
        "data_sources": {
            "terrain_maplibre": (
                "MapTiler/Mapterhorn raster DEM endpoint "
                "used by the official MapLibre terrain example"
            ),
            "terrain_cesium": (
                "Cesium World Terrain when an ion token is supplied"
            ),
            "buildings_maplibre": (
                "OpenStreetMap building footprints fetched through "
                "Overpass API"
            ),
            "buildings_cesium": (
                "Cesium OSM Buildings when an ion token is supplied"
            ),
            "base_map": "OpenStreetMap raster tiles",
        },
        "overpass_queries": metadata,
        "feature_counts": {
            "buildings": len(
                buildings["features"]
            ),
            "roads": len(
                transport["roads"]["features"]
            ),
            "railways": len(
                transport["railways"]["features"]
            ),
            "bridges": len(
                transport["bridges"]["features"]
            ),
            "osm_attractions": len(
                attractions["features"]
            ),
        },
        "building_height_sources": (
            height_sources
        ),
        "limitations": [
            (
                "Building footprints and coordinates are real OSM data; "
                "facade textures are not included."
            ),
            (
                "When OSM height and level tags are absent, the "
                "MapLibre fallback viewer uses a clearly marked "
                "category estimate."
            ),
            (
                "Cesium OSM Buildings provides its own estimatedHeight "
                "for buildings without recorded height."
            ),
            (
                "This stage validates network data coverage and visual "
                "alignment before ROS collision-map conversion."
            ),
        ],
    }
    write_json(
        output_dir / "manifest.json",
        manifest,
    )

    print()
    print("===== Real-data fetch complete =====")
    for key, value in manifest[
        "feature_counts"
    ].items():
        print(f"{key}: {value}")
    print(
        f"Output directory: {output_dir}"
    )


if __name__ == "__main__":
    main()
