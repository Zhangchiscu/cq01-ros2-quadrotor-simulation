const CESIUM_VIEWS = [
  {
    label: "全半岛",
    longitude: 106.553,
    latitude: 29.552,
    height: 6900,
    heading: 330,
    pitch: -38,
  },
  {
    label: "解放碑—洪崖洞",
    longitude: 106.5790,
    latitude: 29.5595,
    height: 1150,
    heading: 330,
    pitch: -30,
  },
  {
    label: "李子坝—鹅岭",
    longitude: 106.5335,
    latitude: 29.5533,
    height: 1250,
    heading: 45,
    pitch: -28,
  },
  {
    label: "十八梯—较场口",
    longitude: 106.5708,
    latitude: 29.5505,
    height: 1050,
    heading: 325,
    pitch: -28,
  },
  {
    label: "朝天门",
    longitude: 106.5905,
    latitude: 29.5670,
    height: 1300,
    heading: 45,
    pitch: -30,
  },
  {
    label: "千厮门大桥",
    longitude: 106.5830,
    latitude: 29.5664,
    height: 900,
    heading: 350,
    pitch: -25,
  },
];

const statusElement = document.getElementById("cesiumStatus");
const tokenInput = document.getElementById("tokenInput");
const terrainInput = document.getElementById("cesiumTerrainScale");
const terrainValue = document.getElementById("cesiumTerrainValue");

let viewer = null;
let buildings = null;
let labelsVisible = true;
let buildingsVisible = true;

function setStatus(text, state = "warn") {
  statusElement.textContent = text;
  statusElement.className = `status ${state}`;
}

function flyToView(view) {
  if (!viewer) return;
  viewer.camera.flyTo({
    destination: Cesium.Cartesian3.fromDegrees(
      view.longitude,
      view.latitude,
      view.height
    ),
    orientation: {
      heading: Cesium.Math.toRadians(view.heading),
      pitch: Cesium.Math.toRadians(view.pitch),
      roll: 0,
    },
    duration: 1.7,
  });
}

function buildViewButtons() {
  const container = document.getElementById("cesiumViewButtons");
  CESIUM_VIEWS.forEach((view) => {
    const button = document.createElement("button");
    button.textContent = view.label;
    button.addEventListener("click", () => flyToView(view));
    container.appendChild(button);
  });
}

async function addLandmarkLabels() {
  const response = await fetch("./data/landmarks.geojson", {
    cache: "no-store",
  });
  const data = await response.json();

  for (const feature of data.features || []) {
    const [longitude, latitude] = feature.geometry.coordinates;
    viewer.entities.add({
      name: feature.properties.name,
      position: Cesium.Cartesian3.fromDegrees(
        longitude,
        latitude,
        8
      ),
      point: {
        pixelSize: 9,
        color: Cesium.Color.fromCssColorString("#ff6d3a"),
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 2,
        heightReference: Cesium.HeightReference.RELATIVE_TO_TERRAIN,
      },
      label: {
        text: feature.properties.name,
        font: "15px sans-serif",
        fillColor: Cesium.Color.WHITE,
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 3,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new Cesium.Cartesian2(0, -18),
        heightReference: Cesium.HeightReference.RELATIVE_TO_TERRAIN,
        distanceDisplayCondition:
          new Cesium.DistanceDisplayCondition(0, 12000),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
      properties: feature.properties,
    });
  }
}

async function initializeCesium(token) {
  if (viewer) {
    viewer.destroy();
    viewer = null;
  }

  Cesium.Ion.defaultAccessToken = token;

  setStatus(
    "正在加载Cesium World Terrain与OSM Buildings……",
    "warn"
  );

  try {
    viewer = new Cesium.Viewer("cesiumContainer", {
      terrain: Cesium.Terrain.fromWorldTerrain({
        requestVertexNormals: true,
        requestWaterMask: true,
      }),
      animation: false,
      timeline: false,
      baseLayerPicker: true,
      geocoder: false,
      homeButton: true,
      sceneModePicker: false,
      navigationHelpButton: false,
      fullscreenButton: true,
      infoBox: true,
      selectionIndicator: true,
      shadows: true,
    });

    viewer.scene.globe.enableLighting = true;
    viewer.scene.fog.enabled = true;
    viewer.scene.verticalExaggeration = Number(
      terrainInput.value
    );

    buildings = await Cesium.createOsmBuildingsAsync({
      enableShowOutline: false,
      showOutline: false,
    });
    viewer.scene.primitives.add(buildings);

    await addLandmarkLabels();
    flyToView(CESIUM_VIEWS[0]);

    setStatus(
      [
        "Cesium真实数据层已加载。",
        "",
        "地形：Cesium World Terrain",
        "建筑：Cesium OSM Buildings",
        "位置与轮廓：OpenStreetMap",
        "无记录高度时：Cesium estimatedHeight",
        "",
        "这是LoD1建筑体块，不是带外墙纹理的摄影测量模型。",
      ].join("\n"),
      "ok"
    );
  } catch (error) {
    console.error(error);
    setStatus(
      `Cesium加载失败：${error.message}\n` +
      "请检查ion令牌和虚拟机网络连接。",
      "error"
    );
  }
}

document
  .getElementById("tokenForm")
  .addEventListener("submit", (event) => {
    event.preventDefault();
    const token = tokenInput.value.trim();
    if (!token) {
      setStatus("令牌不能为空。", "error");
      return;
    }
    localStorage.setItem(
      "yuzhongCesiumIonToken",
      token
    );
    initializeCesium(token);
  });

document
  .getElementById("toggleCesiumBuildings")
  .addEventListener("click", (event) => {
    buildingsVisible = !buildingsVisible;
    if (buildings) {
      buildings.show = buildingsVisible;
    }
    event.currentTarget.classList.toggle(
      "active",
      buildingsVisible
    );
  });

document
  .getElementById("toggleCesiumLabels")
  .addEventListener("click", (event) => {
    labelsVisible = !labelsVisible;
    if (viewer) {
      for (const entity of viewer.entities.values) {
        entity.show = labelsVisible;
      }
    }
    event.currentTarget.classList.toggle(
      "active",
      labelsVisible
    );
  });

document
  .getElementById("clearToken")
  .addEventListener("click", () => {
    localStorage.removeItem("yuzhongCesiumIonToken");
    tokenInput.value = "";
    setStatus(
      "本机浏览器中的Cesium令牌已清除。",
      "warn"
    );
  });

terrainInput.addEventListener("input", () => {
  const value = Number(terrainInput.value);
  terrainValue.textContent = value.toFixed(2);
  if (viewer) {
    viewer.scene.verticalExaggeration = value;
  }
});

buildViewButtons();

const savedToken = localStorage.getItem(
  "yuzhongCesiumIonToken"
);
if (savedToken) {
  tokenInput.value = savedToken;
  initializeCesium(savedToken);
}
