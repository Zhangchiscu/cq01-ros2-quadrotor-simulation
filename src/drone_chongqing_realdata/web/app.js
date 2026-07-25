const SCENE_BOUNDS = [
  [106.485, 29.505],
  [106.605, 29.590],
];

const VIEWS = [
  {
    label: "全半岛",
    center: [106.553, 29.552],
    zoom: 13.25,
    pitch: 68,
    bearing: -27,
  },
  {
    label: "解放碑—洪崖洞",
    center: [106.5788, 29.5595],
    zoom: 15.8,
    pitch: 72,
    bearing: -25,
  },
  {
    label: "李子坝—鹅岭",
    center: [106.5338, 29.5532],
    zoom: 15.2,
    pitch: 72,
    bearing: 25,
  },
  {
    label: "十八梯—较场口",
    center: [106.5708, 29.5505],
    zoom: 16.0,
    pitch: 72,
    bearing: -35,
  },
  {
    label: "朝天门",
    center: [106.5905, 29.5670],
    zoom: 15.4,
    pitch: 70,
    bearing: 35,
  },
  {
    label: "千厮门大桥",
    center: [106.5830, 29.5664],
    zoom: 15.4,
    pitch: 73,
    bearing: -15,
  },
];

const statusElement = document.getElementById("status");
const terrainInput = document.getElementById("terrainScale");
const terrainValue = document.getElementById("terrainValue");
const heightInput = document.getElementById("heightScale");
const heightValue = document.getElementById("heightValue");

let heightMultiplier = Number(heightInput.value);
let buildingsVisible = true;
let transportVisible = true;
let labelsVisible = true;
let landmarkMarkers = [];

const map = new maplibregl.Map({
  container: "map",
  center: VIEWS[0].center,
  zoom: VIEWS[0].zoom,
  pitch: VIEWS[0].pitch,
  bearing: VIEWS[0].bearing,
  maxPitch: 85,
  hash: true,
  style: {
    version: 8,
    sources: {
      osm: {
        type: "raster",
        tiles: [
          "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png",
          "https://b.tile.openstreetmap.org/{z}/{x}/{y}.png",
          "https://c.tile.openstreetmap.org/{z}/{x}/{y}.png",
        ],
        tileSize: 256,
        maxzoom: 19,
        attribution:
          "&copy; OpenStreetMap contributors",
      },
      terrainSource: {
        type: "raster-dem",
        url: "https://tiles.mapterhorn.com/tilejson.json",
      },
      hillshadeSource: {
        type: "raster-dem",
        url: "https://tiles.mapterhorn.com/tilejson.json",
      },
    },
    layers: [
      {
        id: "osm",
        type: "raster",
        source: "osm",
        paint: {
          "raster-saturation": -0.35,
          "raster-brightness-min": 0.18,
          "raster-brightness-max": 0.72,
          "raster-contrast": 0.12,
        },
      },
      {
        id: "hillshade",
        type: "hillshade",
        source: "hillshadeSource",
        paint: {
          "hillshade-shadow-color": "#17202a",
          "hillshade-highlight-color": "#b79b72",
          "hillshade-exaggeration": 0.42,
        },
      },
    ],
    terrain: {
      source: "terrainSource",
      exaggeration: Number(terrainInput.value),
    },
    sky: {},
  },
});

map.addControl(
  new maplibregl.NavigationControl({
    visualizePitch: true,
    showCompass: true,
    showZoom: true,
  }),
  "top-right"
);
map.addControl(new maplibregl.ScaleControl(), "bottom-right");

function setStatus(text, state = "warn") {
  statusElement.textContent = text;
  statusElement.className = `status ${state}`;
}

function cameraTo(view) {
  map.easeTo({
    center: view.center,
    zoom: view.zoom,
    pitch: view.pitch,
    bearing: view.bearing,
    duration: 1500,
  });
}

function buildViewButtons() {
  const container = document.getElementById("viewButtons");
  VIEWS.forEach((view) => {
    const button = document.createElement("button");
    button.textContent = view.label;
    button.addEventListener("click", () => cameraTo(view));
    container.appendChild(button);
  });
}

function buildingHeightExpression() {
  return [
    "*",
    ["to-number", ["get", "height_m"], 15],
    heightMultiplier,
  ];
}

function buildingBaseExpression() {
  return [
    "*",
    ["to-number", ["get", "min_height_m"], 0],
    heightMultiplier,
  ];
}

function updateBuildingHeight() {
  if (!map.getLayer("buildings-3d")) return;
  map.setPaintProperty(
    "buildings-3d",
    "fill-extrusion-height",
    buildingHeightExpression()
  );
  map.setPaintProperty(
    "buildings-3d",
    "fill-extrusion-base",
    buildingBaseExpression()
  );
}

async function loadJson(path) {
  const response = await fetch(path, {
    cache: "no-store",
  });
  if (!response.ok) {
    throw new Error(`${path}: HTTP ${response.status}`);
  }
  return response.json();
}

function addGeoJsonSource(name, data) {
  if (map.getSource(name)) return;
  map.addSource(name, {
    type: "geojson",
    data,
  });
}

function lineWidthExpression() {
  return [
    "interpolate",
    ["linear"],
    ["zoom"],
    11,
    0.4,
    15,
    2.4,
    18,
    5.0,
  ];
}

function addLayers({
  buildings,
  roads,
  railways,
  bridges,
}) {
  addGeoJsonSource("buildings", buildings);
  map.addLayer({
    id: "buildings-3d",
    type: "fill-extrusion",
    source: "buildings",
    minzoom: 11,
    paint: {
      "fill-extrusion-height": buildingHeightExpression(),
      "fill-extrusion-base": buildingBaseExpression(),
      "fill-extrusion-color": [
        "match",
        ["get", "height_source"],
        "osm_height",
        "#62c995",
        "osm_levels",
        "#e9a54a",
        "#758390",
      ],
      "fill-extrusion-opacity": 0.92,
      "fill-extrusion-vertical-gradient": true,
    },
  });

  addGeoJsonSource("roads", roads);
  map.addLayer({
    id: "roads",
    type: "line",
    source: "roads",
    paint: {
      "line-color": [
        "match",
        ["get", "highway"],
        "motorway",
        "#f36d4f",
        "trunk",
        "#ed8e48",
        "primary",
        "#eebf5c",
        "secondary",
        "#b9c9d2",
        "#7e919d",
      ],
      "line-width": lineWidthExpression(),
      "line-opacity": 0.82,
    },
  });

  addGeoJsonSource("railways", railways);
  map.addLayer({
    id: "railways",
    type: "line",
    source: "railways",
    paint: {
      "line-color": "#40d4ff",
      "line-width": [
        "interpolate",
        ["linear"],
        ["zoom"],
        11,
        0.8,
        17,
        4.0,
      ],
      "line-opacity": 0.95,
    },
  });

  addGeoJsonSource("bridges", bridges);
  map.addLayer({
    id: "bridges",
    type: "line",
    source: "bridges",
    paint: {
      "line-color": "#ff4f40",
      "line-width": [
        "interpolate",
        ["linear"],
        ["zoom"],
        11,
        1.2,
        17,
        6.5,
      ],
      "line-opacity": 1.0,
    },
  });

  map.on("click", "buildings-3d", (event) => {
    const feature = event.features?.[0];
    if (!feature) return;

    const properties = feature.properties || {};
    const content = `
      <strong>${properties.name || "OSM建筑"}</strong><br>
      OSM ID：${properties.osm_type}/${properties.osm_id}<br>
      高度：${properties.height_m || "N/A"} m<br>
      高度来源：${properties.height_source || "N/A"}<br>
      楼层：${properties["building:levels"] || "N/A"}<br>
      建筑类型：${properties.building || "N/A"}
    `;

    new maplibregl.Popup()
      .setLngLat(event.lngLat)
      .setHTML(content)
      .addTo(map);
  });

  map.on("mouseenter", "buildings-3d", () => {
    map.getCanvas().style.cursor = "pointer";
  });
  map.on("mouseleave", "buildings-3d", () => {
    map.getCanvas().style.cursor = "";
  });
}

function addLandmarks(data) {
  landmarkMarkers.forEach((marker) => marker.remove());
  landmarkMarkers = [];

  for (const feature of data.features || []) {
    const coordinates = feature.geometry?.coordinates;
    if (!coordinates) continue;

    const element = document.createElement("div");
    element.className = "landmark-marker";

    const marker = new maplibregl.Marker({
      element,
      anchor: "center",
    })
      .setLngLat(coordinates)
      .setPopup(
        new maplibregl.Popup({
          offset: 14,
        }).setHTML(
          `<strong>${feature.properties.name}</strong><br>` +
          `${feature.properties.name_en || ""}`
        )
      )
      .addTo(map);

    landmarkMarkers.push(marker);
  }
}

function setLayerVisibility(layerIds, visible) {
  for (const layerId of layerIds) {
    if (!map.getLayer(layerId)) continue;
    map.setLayoutProperty(
      layerId,
      "visibility",
      visible ? "visible" : "none"
    );
  }
}

async function initializeData() {
  const [
    buildings,
    roads,
    railways,
    bridges,
    landmarks,
    manifest,
  ] = await Promise.all([
    loadJson("./data/buildings.geojson"),
    loadJson("./data/roads.geojson"),
    loadJson("./data/railways.geojson"),
    loadJson("./data/bridges.geojson"),
    loadJson("./data/landmarks.geojson"),
    loadJson("./data/manifest.json"),
  ]);

  addLayers({
    buildings,
    roads,
    railways,
    bridges,
  });
  addLandmarks(landmarks);

  const counts = manifest.feature_counts;
  if (!counts) {
    setStatus(
      "尚未获取OSM数据。\n" +
      "请先在终端运行：\n" +
      "ros2 run drone_chongqing_realdata " +
      "fetch_yuzhong_realdata",
      "warn"
    );
    return;
  }

  setStatus(
    [
      `建筑轮廓：${counts.buildings}`,
      `道路：${counts.roads}`,
      `轨道：${counts.railways}`,
      `桥梁：${counts.bridges}`,
      `OSM景点：${counts.osm_attractions}`,
      "",
      "建筑位置与轮廓来自OSM；",
      "地形来自网络DEM；",
      "无height标签的楼高会明确标记为类别估计。",
    ].join("\n"),
    "ok"
  );
}

terrainInput.addEventListener("input", () => {
  const value = Number(terrainInput.value);
  terrainValue.textContent = value.toFixed(2);
  map.setTerrain({
    source: "terrainSource",
    exaggeration: value,
  });
});

heightInput.addEventListener("input", () => {
  heightMultiplier = Number(heightInput.value);
  heightValue.textContent = heightMultiplier.toFixed(2);
  updateBuildingHeight();
});

document
  .getElementById("toggleBuildings")
  .addEventListener("click", (event) => {
    buildingsVisible = !buildingsVisible;
    setLayerVisibility(
      ["buildings-3d"],
      buildingsVisible
    );
    event.currentTarget.classList.toggle(
      "active",
      buildingsVisible
    );
  });

document
  .getElementById("toggleTransport")
  .addEventListener("click", (event) => {
    transportVisible = !transportVisible;
    setLayerVisibility(
      ["roads", "railways", "bridges"],
      transportVisible
    );
    event.currentTarget.classList.toggle(
      "active",
      transportVisible
    );
  });

document
  .getElementById("toggleLabels")
  .addEventListener("click", (event) => {
    labelsVisible = !labelsVisible;
    for (const marker of landmarkMarkers) {
      marker.getElement().style.display =
        labelsVisible ? "block" : "none";
    }
    event.currentTarget.classList.toggle(
      "active",
      labelsVisible
    );
  });

buildViewButtons();

map.on("load", async () => {
  map.fitBounds(SCENE_BOUNDS, {
    padding: {
      top: 45,
      bottom: 45,
      left: 390,
      right: 45,
    },
    pitch: 68,
    bearing: -27,
    duration: 0,
  });

  try {
    await initializeData();
  } catch (error) {
    console.error(error);
    setStatus(
      `数据加载失败：${error.message}`,
      "error"
    );
  }
});
