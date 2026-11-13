"""
map_widget.py - a small NATIVE OpenStreetMap viewer (no browser / WebEngine).

How "slippy map" tiles work: the world is a square image cut into 256x256 px
tiles. At zoom level z there are 2^z x 2^z tiles. Any lat/lon maps to a pixel
position in that giant image (Web-Mercator maths in _world_px). We download
only the tiles that overlap the widget and paint them at the right offsets.

Tiles are cached on disk (QNetworkDiskCache), so after the first run the map
also works offline for places already viewed.
"""
from __future__ import annotations

import math
import tempfile
import time
from collections import deque
from pathlib import Path

from PySide6.QtCore import QPointF, QRectF, Qt, QStandardPaths, QUrl
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPixmap, QPolygonF
from PySide6.QtNetwork import QNetworkAccessManager, QNetworkDiskCache, QNetworkReply, QNetworkRequest
from PySide6.QtWidgets import QWidget

TILE = 256
EARTH_R = 6378137.0
TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
# OSM's tile policy requires an identifying User-Agent.
USER_AGENT = "ROVGroundStation/1.0 (internship assessment; desktop Qt app)"
MIN_ZOOM, MAX_ZOOM = 15, 19
RETRY_AFTER_S = 15.0
MAX_PENDING = 24
MAX_CACHED_TILES = 400


class MapWidget(QWidget):
    def __init__(self, home_lat: float, home_lon: float, zoom: int = 18, parent=None):
        super().__init__(parent)
        self.setMinimumSize(280, 200)
        self.home = (home_lat, home_lon)
        self.center = (home_lat, home_lon)
        self.zoom = zoom
        self.rov = (home_lat, home_lon)
        self.heading = 0.0
        self.trail: deque = deque(maxlen=3000)
        self.trail.append(self.rov)

        self._tiles: dict = {}        # (z,x,y) -> QPixmap
        self._pending: dict = {}      # QNetworkReply -> (z,x,y)
        self._failed: dict = {}       # (z,x,y) -> time of failure

        self._nam = QNetworkAccessManager(self)
        cache = QNetworkDiskCache(self)
        base = QStandardPaths.writableLocation(QStandardPaths.StandardLocation.CacheLocation)
        cache.setCacheDirectory(str(Path(base or tempfile.gettempdir()) / "rov_map_tiles"))
        cache.setMaximumCacheSize(100 * 1024 * 1024)
        self._nam.setCache(cache)
        self._nam.finished.connect(self._on_reply)

    # ---------------------------------------------------------------- geometry
    @staticmethod
    def _world_px(lat: float, lon: float, z: int):
        n = TILE * (2 ** z)
        x = (lon + 180.0) / 360.0 * n
        y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
        return x, y

    def _offset_to_latlon(self, north: float, east: float):
        lat0, lon0 = self.home
        lat = lat0 + math.degrees(north / EARTH_R)
        lon = lon0 + math.degrees(east / (EARTH_R * math.cos(math.radians(lat0))))
        return lat, lon

    def _view_origin(self):
        cx, cy = self._world_px(*self.center, self.zoom)
        return cx - self.width() / 2.0, cy - self.height() / 2.0

    def _metres_per_px(self) -> float:
        return math.cos(math.radians(self.center[0])) * 2 * math.pi * EARTH_R / (TILE * 2 ** self.zoom)

    # ---------------------------------------------------------------- public API
    def set_position(self, north: float, east: float, heading_deg: float) -> None:
        self.rov = self._offset_to_latlon(north, east)
        self.heading = heading_deg
        last = self.trail[-1]
        # only add a trail point when we moved > ~0.3 m (keeps the trail light)
        if abs(self.rov[0] - last[0]) * 111_320 > 0.3 or abs(self.rov[1] - last[1]) * 111_320 > 0.3:
            self.trail.append(self.rov)
        # keep the vehicle in view: re-centre when it nears the edge
        ox, oy = self._view_origin()
        px, py = self._world_px(*self.rov, self.zoom)
        mx, my = self.width() * 0.2, self.height() * 0.2
        if not (ox + mx < px < ox + self.width() - mx and oy + my < py < oy + self.height() - my):
            self.center = self.rov
        self.update()

    def wheelEvent(self, event) -> None:
        step = 1 if event.angleDelta().y() > 0 else -1
        self.zoom = max(MIN_ZOOM, min(MAX_ZOOM, self.zoom + step))
        self.update()

    # ---------------------------------------------------------------- tiles
    def _tile(self, z: int, x: int, y: int):
        key = (z, x, y)
        pix = self._tiles.get(key)
        if pix is not None:
            return pix
        failed_at = self._failed.get(key)
        in_flight = key in self._pending.values()
        if not in_flight and len(self._pending) < MAX_PENDING and (
                failed_at is None or time.monotonic() - failed_at > RETRY_AFTER_S):
            self._request_tile(key)
        return None

    def _request_tile(self, key) -> None:
        z, x, y = key
        req = QNetworkRequest(QUrl(TILE_URL.format(z=z, x=x, y=y)))
        req.setHeader(QNetworkRequest.KnownHeaders.UserAgentHeader, USER_AGENT)
        req.setAttribute(QNetworkRequest.Attribute.CacheLoadControlAttribute,
                         QNetworkRequest.CacheLoadControl.PreferCache)
        self._pending[self._nam.get(req)] = key

    def _on_reply(self, reply: QNetworkReply) -> None:
        key = self._pending.pop(reply, None)
        if key is not None:
            pix = QPixmap()
            if reply.error() == QNetworkReply.NetworkError.NoError and pix.loadFromData(reply.readAll()):
                if len(self._tiles) >= MAX_CACHED_TILES:
                    self._tiles.pop(next(iter(self._tiles)))      # drop the oldest
                self._tiles[key] = pix
                self._failed.pop(key, None)
            else:
                self._failed[key] = time.monotonic()
            self.update()
        reply.deleteLater()

    # ---------------------------------------------------------------- painting
    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#1b2028"))
        z = self.zoom
        n = 2 ** z
        ox, oy = self._view_origin()

        missing = False
        for tx in range(math.floor(ox / TILE), math.floor((ox + self.width()) / TILE) + 1):
            for ty in range(math.floor(oy / TILE), math.floor((oy + self.height()) / TILE) + 1):
                if not 0 <= ty < n:
                    continue
                dest = QPointF(tx * TILE - ox, ty * TILE - oy)
                pix = self._tile(z, tx % n, ty)
                if pix is not None:
                    p.drawPixmap(dest, pix)
                else:
                    missing = True
                    p.setPen(QPen(QColor("#2b323d")))
                    p.drawRect(QRectF(dest, QPointF(dest.x() + TILE, dest.y() + TILE)))

        def to_local(latlon):
            wx, wy = self._world_px(*latlon, z)
            return QPointF(wx - ox, wy - oy)

        # trail
        if len(self.trail) > 1:
            p.setPen(QPen(QColor("#ff9f1c"), 2.5, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            p.drawPolyline(QPolygonF([to_local(pt) for pt in self.trail]))

        # home marker
        h = to_local(self.home)
        p.setPen(QPen(QColor("#ffffff"), 2))
        p.setBrush(QColor("#2ec4b6"))
        p.drawEllipse(h, 5, 5)

        # ROV marker: a triangle pointing along the heading
        r = to_local(self.rov)
        p.save()
        p.translate(r)
        p.rotate(self.heading)             # heading 0 = north = up on screen
        p.setPen(QPen(QColor("#ffffff"), 1.5))
        p.setBrush(QColor("#e63946"))
        p.drawPolygon(QPolygonF([QPointF(0, -12), QPointF(8, 9), QPointF(0, 5), QPointF(-8, 9)]))
        p.restore()

        # scale bar
        mpp = self._metres_per_px()
        length_m = next((m for m in (5, 10, 20, 50, 100, 200) if m / mpp >= 50), 200)
        bar_px = length_m / mpp
        y = self.height() - 14
        p.setPen(QPen(QColor("#111"), 4))
        p.drawLine(QPointF(10, y), QPointF(10 + bar_px, y))
        p.setPen(QPen(QColor("#fff"), 2))
        p.drawLine(QPointF(10, y), QPointF(10 + bar_px, y))
        font = QFont(self.font())
        font.setPointSize(8)
        p.setFont(font)
        p.setPen(QColor("#fff"))
        p.drawText(QPointF(12, y - 5), f"{length_m} m")

        # required OSM attribution
        text = "\u00a9 OpenStreetMap contributors"
        fm = p.fontMetrics()
        w = fm.horizontalAdvance(text) + 10
        p.fillRect(QRectF(self.width() - w, self.height() - 16, w, 16), QColor(255, 255, 255, 190))
        p.setPen(QColor("#222"))
        p.drawText(QPointF(self.width() - w + 5, self.height() - 4), text)

        if missing and not self._tiles:
            p.setPen(QColor("#9aa5b1"))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                       "Loading OpenStreetMap tiles...\n(internet needed the first time)")
