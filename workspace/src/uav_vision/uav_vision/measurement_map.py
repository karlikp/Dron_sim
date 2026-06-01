from __future__ import annotations

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Tuple

import numpy as np
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool

try:
    from px4_msgs.msg import VehicleGlobalPosition
    PX4_GPOS_AVAILABLE = True
except Exception:
    PX4_GPOS_AVAILABLE = False
    VehicleGlobalPosition = None


def declare_measurement_map_parameters(node) -> None:
    node.declare_parameter("enable_measurement_map_markers", False)
    node.declare_parameter("measurement_map_active", False)
    node.declare_parameter("measurement_map_active_topic", "/measurement_map_active")
    node.declare_parameter("measurement_gpos_source", "px4")
    node.declare_parameter("measurement_gpos_topic", "/fmu/out/vehicle_global_position")
    node.declare_parameter("measurement_map_output_dir", "/tmp/uav_measurement_map")
    node.declare_parameter("measurement_map_host", "127.0.0.1")
    node.declare_parameter("measurement_map_port", 8765)
    node.declare_parameter("measurement_map_style", "mapbox://styles/karol-pitera/cmp8lj4m8000f01s38yt5hklq")
    node.declare_parameter(
        "measurement_map_access_token",
        "pk.eyJ1Ijoia2Fyb2wtcGl0ZXJhIiwiYSI6ImNtaW43aTluMjF5ODQza3NmcjIwaDdzdmUifQ.fMq_BbBIl2EtYA_Mdz73WQ",
    )


class MeasurementMap:
    def __init__(self, node, px4_qos_factory) -> None:
        self._node = node
        self._px4_qos_factory = px4_qos_factory
        self.enabled = bool(node.get_parameter("enable_measurement_map_markers").value)
        self.active = bool(node.get_parameter("measurement_map_active").value)
        self._latest_gpos: Optional[Tuple[float, float, float]] = None
        self._features = []
        self._features_lock = threading.Lock()
        self._server = None
        self._thread = None
        self._active_sub = None
        self._gpos_sub = None
        self._last_stop_state = False

        if self.enabled:
            self._init()

    def reset_stop_edge(self) -> None:
        self._last_stop_state = False

    def maybe_add_marker(self, stop_state: bool, min_depth: float, valid_fraction: float) -> None:
        # Kept for compatibility with older callers. Markers are now added by
        # the map UI measurement button, not automatically on obstacle stop.
        self._last_stop_state = stop_state

    def add_measurement_marker(self) -> Tuple[bool, str, Optional[dict]]:
        if not self.enabled:
            return False, "Measurement map is disabled.", None

        if self._latest_gpos is None:
            message = "No global position is available yet; marker skipped."
            self._node.get_logger().warn(message)
            return False, message, None

        lat, lon, alt = self._latest_gpos
        if not np.isfinite(lat) or not np.isfinite(lon):
            message = "Global position is invalid; marker skipped."
            self._node.get_logger().warn(message)
            return False, message, None

        with self._features_lock:
            feature = {
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [lon, lat],
                },
                "properties": {
                    "id": len(self._features) + 1,
                    "title": "Measurement",
                    "lat": lat,
                    "lon": lon,
                    "alt_m": alt,
                    "timestamp_ns": self._node.get_clock().now().nanoseconds,
                    "source": "measurement_button",
                },
            }
            self._features.append(feature)
            self._write_geojson()

        message = f"Measurement marker #{feature['properties']['id']} added."
        self._node.get_logger().info(
            f"{message} lat={lat:.8f}, lon={lon:.8f}, alt={alt:.2f}"
        )
        return True, message, feature

    def shutdown(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None

    def _init(self) -> None:
        os.makedirs(str(self._node.get_parameter("measurement_map_output_dir").value), exist_ok=True)
        self._write_geojson()
        self._init_gps_subscription()
        self._init_activation_subscription()
        self._start_server()

    def _init_gps_subscription(self) -> None:
        source = str(self._node.get_parameter("measurement_gpos_source").value).lower()
        gpos_topic = str(self._node.get_parameter("measurement_gpos_topic").value)

        if source == "px4":
            if not PX4_GPOS_AVAILABLE:
                self._node.get_logger().error(
                    "enable_measurement_map_markers=True but px4 VehicleGlobalPosition is not available."
                )
            else:
                self._gpos_sub = self._node.create_subscription(
                    VehicleGlobalPosition,
                    gpos_topic,
                    self._on_px4_global_position,
                    self._px4_qos_factory(),
                )
                self._node.get_logger().info(f"Measurement map GPS source enabled (PX4): {gpos_topic}")

        elif source == "navsat":
            self._gpos_sub = self._node.create_subscription(
                NavSatFix,
                gpos_topic,
                self._on_navsat_fix,
                self._px4_qos_factory(),
            )
            self._node.get_logger().info(f"Measurement map GPS source enabled (NavSatFix): {gpos_topic}")

        else:
            self._node.get_logger().error(f"Unsupported measurement_gpos_source='{source}'. Use 'px4' or 'navsat'.")

    def _init_activation_subscription(self) -> None:
        active_topic = str(self._node.get_parameter("measurement_map_active_topic").value)
        if active_topic:
            self._active_sub = self._node.create_subscription(Bool, active_topic, self._on_active, 10)
            self._node.get_logger().info(f"Measurement map activation topic: {active_topic}")

    def _on_px4_global_position(self, msg: VehicleGlobalPosition) -> None:
        self._latest_gpos = (float(msg.lat), float(msg.lon), float(msg.alt))

    def _on_navsat_fix(self, msg: NavSatFix) -> None:
        self._latest_gpos = (float(msg.latitude), float(msg.longitude), float(msg.altitude))

    def _on_active(self, msg: Bool) -> None:
        self.active = bool(msg.data)
        self._node.get_logger().info(f"Measurement map marking active={self.active}")

    def _geojson_path(self) -> str:
        out_dir = str(self._node.get_parameter("measurement_map_output_dir").value)
        return os.path.join(out_dir, "measurement_markers.geojson")

    def _write_geojson(self) -> None:
        collection = {
            "type": "FeatureCollection",
            "features": self._features,
        }
        path = self._geojson_path()
        tmp_path = f"{path}.tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(collection, f, ensure_ascii=False)
        os.replace(tmp_path, path)

    def _map_html(self) -> str:
        style = str(self._node.get_parameter("measurement_map_style").value)
        token = str(self._node.get_parameter("measurement_map_access_token").value)
        return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>UAV measurement map</title>
  <link href="https://api.mapbox.com/mapbox-gl-js/v3.11.0/mapbox-gl.css" rel="stylesheet" />
  <script src="https://api.mapbox.com/mapbox-gl-js/v3.11.0/mapbox-gl.js"></script>
  <style>
    html, body, #map {{ height: 100%; margin: 0; }}
    .panel {{
      position: absolute; top: 12px; left: 12px; z-index: 2;
      display: flex; align-items: center; gap: 10px;
      padding: 8px 10px; border-radius: 6px;
      background: rgba(20, 24, 28, 0.82); color: white;
      font: 13px/1.35 system-ui, sans-serif;
    }}
    .panel button {{
      appearance: none; border: 1px solid rgba(255,255,255,0.35);
      border-radius: 5px; background: #ff2d55; color: white;
      padding: 6px 9px; font: inherit; cursor: pointer;
    }}
    .panel button:disabled {{ opacity: 0.6; cursor: wait; }}
    #status {{ color: rgba(255,255,255,0.78); min-width: 130px; }}
  </style>
</head>
<body>
  <div id="map"></div>
  <div class="panel">
    <div><strong>UAV measurement points</strong><br><span id="count">0</span> markers</div>
    <button id="measurement" type="button">measurement</button>
    <span id="status"></span>
  </div>
  <script>
    mapboxgl.accessToken = {json.dumps(token)};
    const map = new mapboxgl.Map({{
      container: "map",
      style: {json.dumps(style)},
      center: [8.547216, 47.399267],
      zoom: 16.19,
      bearing: 83.2,
      pitch: 1
    }});
    map.addControl(new mapboxgl.NavigationControl());

    async function loadMarkers() {{
      const response = await fetch("/markers.geojson?ts=" + Date.now());
      const data = await response.json();
      document.getElementById("count").textContent = data.features.length;

      if (map.getSource("measurement-points")) {{
        map.getSource("measurement-points").setData(data);
      }} else {{
        map.addSource("measurement-points", {{ type: "geojson", data }});
        map.addLayer({{
          id: "measurement-point-halo",
          type: "circle",
          source: "measurement-points",
          paint: {{
            "circle-radius": 14,
            "circle-color": "#ff2d55",
            "circle-opacity": 0.22
          }}
        }});
        map.addLayer({{
          id: "measurement-point",
          type: "circle",
          source: "measurement-points",
          paint: {{
            "circle-radius": 7,
            "circle-color": "#ff2d55",
            "circle-stroke-color": "#ffffff",
            "circle-stroke-width": 2
          }}
        }});
      }}

      if (data.features.length > 0 && !window.__fitDone) {{
        const bounds = new mapboxgl.LngLatBounds();
        data.features.forEach((feature) => bounds.extend(feature.geometry.coordinates));
        map.fitBounds(bounds, {{ padding: 80, maxZoom: 18 }});
        window.__fitDone = true;
      }}
    }}

    async function addMeasurement() {{
      const button = document.getElementById("measurement");
      const status = document.getElementById("status");
      button.disabled = true;
      status.textContent = "adding...";
      try {{
        const response = await fetch("/measurement", {{ method: "POST" }});
        const result = await response.json();
        status.textContent = result.message || (result.ok ? "added" : "failed");
        if (!response.ok || !result.ok) {{
          console.warn("Measurement marker failed", result);
        }}
        await loadMarkers();
      }} catch (error) {{
        status.textContent = "request failed";
        console.error(error);
      }} finally {{
        button.disabled = false;
        setTimeout(() => {{ status.textContent = ""; }}, 3500);
      }}
    }}

    map.on("load", () => {{
      loadMarkers();
      setInterval(loadMarkers, 1000);
      document.getElementById("measurement").addEventListener("click", addMeasurement);
    }});
  </script>
</body>
</html>
"""

    def _start_server(self) -> None:
        host = str(self._node.get_parameter("measurement_map_host").value)
        port = int(self._node.get_parameter("measurement_map_port").value)
        measurement_map = self

        class MeasurementMapHandler(BaseHTTPRequestHandler):
            def do_POST(self) -> None:
                if self.path == "/measurement":
                    ok, message, feature = measurement_map.add_measurement_marker()
                    payload = {"ok": ok, "message": message, "feature": feature}
                    self._send_json(payload, status=200 if ok else 409)
                    return
                self.send_error(404)

            def do_GET(self) -> None:
                if self.path.startswith("/markers.geojson"):
                    self._send_json_file(measurement_map._geojson_path())
                    return
                if self.path == "/" or self.path.startswith("/index.html"):
                    self._send_html(measurement_map._map_html())
                    return
                self.send_error(404)

            def log_message(self, fmt: str, *args) -> None:
                return

            def _send_html(self, html: str) -> None:
                data = html.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def _send_json(self, payload: dict, status: int = 200) -> None:
                data = json.dumps(payload).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def _send_json_file(self, path: str) -> None:
                try:
                    with open(path, "rb") as f:
                        data = f.read()
                except OSError:
                    data = b'{"type":"FeatureCollection","features":[]}'
                self.send_response(200)
                self.send_header("Content-Type", "application/geo+json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

        try:
            self._server = ThreadingHTTPServer((host, port), MeasurementMapHandler)
        except OSError as exc:
            self._node.get_logger().error(f"Could not start measurement map server on {host}:{port}: {exc}")
            return

        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        self._node.get_logger().info(f"Measurement map available at http://{host}:{port}/")
