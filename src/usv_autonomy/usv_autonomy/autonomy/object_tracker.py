# ================================================================
# File: object_tracker.py
# Description: Tracks detections to stabilize and predict objects for avoidance
# Author: Andrew Thomas
# Co-Authors: N/A
# Created: 02-15-2026
# Last Modified: 02-18-2026
# Version: 1.0.0
# Dependencies: dataclasses, math, typing, time
# Notes:
# ================================================================


from dataclasses import dataclass
from math import cos, radians, sqrt
from typing import Dict, List, Optional, Tuple
import time


# approximation good enough for local range
METERS_PER_DEG_LAT = 111320.0


@dataclass
class _Track:
    # internal track record for object tracker
    track_id: int
    obj_type: str
    lat: float
    lon: float
    confidence: float
    state: str
    created_at: float
    first_seen: float
    last_seen: float
    hits: int
    misses: int
    vx_mps: float
    vy_mps: float
    speed_mps: float
    is_moving: bool


class ObjectTracker:
    # tracks cv detections over time and outputs stable objects
    # avoid new objects for repeat detections
    # filter one off ghost detections
    # track simple motion and expose is_moving
    # remove stale tracks

    def __init__(
        self,
        association_distance_m: float = 4.0,
        dedupe_distance_m: float = 1.5,
        min_hits_to_confirm: int = 2,
        stale_timeout_s: float = 2.0,
        delete_timeout_s: float = 8.0,
        moving_speed_threshold_mps: float = 0.5,
        prediction_horizon_s: float = 1.0,
        velocity_alpha: float = 0.6,
    ):
        self.association_distance_m = association_distance_m
        self.dedupe_distance_m = dedupe_distance_m
        self.min_hits_to_confirm = min_hits_to_confirm
        self.stale_timeout_s = stale_timeout_s
        self.delete_timeout_s = delete_timeout_s
        self.moving_speed_threshold_mps = moving_speed_threshold_mps
        self.prediction_horizon_s = prediction_horizon_s
        self.velocity_alpha = velocity_alpha

        self._tracks: Dict[int, _Track] = {}
        self._next_id = 1

    def update(self, detections: List[dict], now: Optional[float] = None) -> List[dict]:
        # main entry point called each frame cycle
        # returns active confirmed tracks for planners
        now = time.monotonic() if now is None else now

        # clean and normalize input
        detections = self._normalize_detections(detections)
        detections = self._dedupe_detections(detections)

        # match detections to tracks then update
        assignments, unmatched_track_ids, unmatched_detection_idxs = self._associate(
            detections
        )

        for track_id, det_idx in assignments:
            self._update_track(self._tracks[track_id], detections[det_idx], now)

        for track_id in unmatched_track_ids:
            self._mark_missed(self._tracks[track_id], now)

        for det_idx in unmatched_detection_idxs:
            self._create_track(detections[det_idx], now)

        # update states and remove expired tracks
        self._refresh_states(now)

        return self.get_active_tracks(now=now)

    def get_active_tracks(self, include_tentative: bool = False, now: Optional[float] = None) -> List[dict]:
        # export tracks in dict format used by mission and avoidance code
        now = time.monotonic() if now is None else now
        output = []

        for track in self._tracks.values():
            if track.state == "deleted":
                continue
            if not include_tentative and track.state != "confirmed":
                continue

            pred_lat, pred_lon = self._predict_latlon(track, self.prediction_horizon_s)

            output.append(
                {
                    "id": track.track_id,
                    "type": track.obj_type,
                    "lat": track.lat,
                    "lon": track.lon,
                    "pred_lat": pred_lat,
                    "pred_lon": pred_lon,
                    "confidence": track.confidence,
                    "state": track.state,
                    "hits": track.hits,
                    "misses": track.misses,
                    "is_moving": track.is_moving,
                    "speed_mps": track.speed_mps,
                    "age_s": now - track.created_at,
                    "last_seen_s": now - track.last_seen,
                }
            )

        # sort by id for deterministic output
        output.sort(key=lambda item: item["id"])
        return output

    def reset(self) -> None:
        # hard reset tracker state
        self._tracks.clear()
        self._next_id = 1

    def _normalize_detections(self, detections: List[dict]) -> List[dict]:
        # keep only detections with needed fields
        normalized = []
        for det in detections:
            if "lat" not in det or "lon" not in det:
                continue

            normalized.append(
                {
                    "type": det.get("type", "UNKNOWN"),
                    "lat": float(det["lat"]),
                    "lon": float(det["lon"]),
                    "confidence": float(det.get("confidence", 1.0)),
                }
            )
        return normalized

    def _dedupe_detections(self, detections: List[dict]) -> List[dict]:
        # merge same type detections that are almost identical in one frame
        # this reduces duplicate tracks from cv boxes
        if not detections:
            return []

        used = [False] * len(detections)
        merged = []

        for i, base in enumerate(detections):
            if used[i]:
                continue

            cluster = [base]
            used[i] = True

            for j in range(i + 1, len(detections)):
                if used[j]:
                    continue
                cand = detections[j]

                if cand["type"] != base["type"]:
                    continue

                d_m = self._distance_m(
                    base["lat"], base["lon"], cand["lat"], cand["lon"]
                )
                if d_m <= self.dedupe_distance_m:
                    cluster.append(cand)
                    used[j] = True

            # average cluster into one detection
            n = float(len(cluster))
            lat = sum(item["lat"] for item in cluster) / n
            lon = sum(item["lon"] for item in cluster) / n
            conf = sum(item["confidence"] for item in cluster) / n

            merged.append(
                {
                    "type": base["type"],
                    "lat": lat,
                    "lon": lon,
                    "confidence": conf,
                }
            )

        return merged

    def _associate(self, detections: List[dict]) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        # greedy nearest neighbor assignment by distance and type
        track_ids = list(self._tracks.keys())
        det_indices = list(range(len(detections)))

        candidates = []
        for track_id in track_ids:
            track = self._tracks[track_id]
            for det_idx in det_indices:
                det = detections[det_idx]

                # keep type consistent tracks to avoid id jumps
                if det["type"] != track.obj_type:
                    continue

                d_m = self._distance_m(track.lat, track.lon, det["lat"], det["lon"])
                if d_m <= self.association_distance_m:
                    candidates.append((d_m, track_id, det_idx))

        candidates.sort(key=lambda item: item[0])

        assigned_tracks = set()
        assigned_dets = set()
        assignments = []

        for _d_m, track_id, det_idx in candidates:
            if track_id in assigned_tracks or det_idx in assigned_dets:
                continue
            assigned_tracks.add(track_id)
            assigned_dets.add(det_idx)
            assignments.append((track_id, det_idx))

        unmatched_track_ids = [tid for tid in track_ids if tid not in assigned_tracks]
        unmatched_detection_idxs = [idx for idx in det_indices if idx not in assigned_dets]
        return assignments, unmatched_track_ids, unmatched_detection_idxs

    def _create_track(self, detection: dict, now: float) -> None:
        # new detections start as tentative
        track = _Track(
            track_id=self._next_id,
            obj_type=detection["type"],
            lat=detection["lat"],
            lon=detection["lon"],
            confidence=detection["confidence"],
            state="tentative",
            created_at=now,
            first_seen=now,
            last_seen=now,
            hits=1,
            misses=0,
            vx_mps=0.0,
            vy_mps=0.0,
            speed_mps=0.0,
            is_moving=False,
        )
        self._tracks[track.track_id] = track
        self._next_id += 1

    def _update_track(self, track: _Track, detection: dict, now: float) -> None:
        # update position confidence and velocity with smoothing
        dt = max(now - track.last_seen, 1e-3)

        dx_m, dy_m = self._delta_m(track.lat, track.lon, detection["lat"], detection["lon"])
        inst_vx = dx_m / dt
        inst_vy = dy_m / dt

        track.vx_mps = self.velocity_alpha * inst_vx + (1.0 - self.velocity_alpha) * track.vx_mps
        track.vy_mps = self.velocity_alpha * inst_vy + (1.0 - self.velocity_alpha) * track.vy_mps
        track.speed_mps = sqrt(track.vx_mps * track.vx_mps + track.vy_mps * track.vy_mps)
        track.is_moving = track.speed_mps >= self.moving_speed_threshold_mps

        # smooth confidence to avoid frame noise swings
        track.confidence = 0.7 * track.confidence + 0.3 * detection["confidence"]
        track.lat = detection["lat"]
        track.lon = detection["lon"]
        track.last_seen = now
        track.hits += 1
        track.misses = 0

        # stale track can recover after new hit
        if track.state in ("tentative", "stale"):
            if track.hits >= self.min_hits_to_confirm:
                track.state = "confirmed"

    def _mark_missed(self, track: _Track, now: float) -> None:
        # missed update does not delete immediately
        track.misses += 1
        elapsed = now - track.last_seen

        if elapsed >= self.stale_timeout_s and track.state != "deleted":
            track.state = "stale"

    def _refresh_states(self, now: float) -> None:
        # confirm tentative tracks and delete old stale tracks
        to_delete = []
        for track_id, track in self._tracks.items():
            since_seen = now - track.last_seen

            if track.state == "tentative" and track.hits >= self.min_hits_to_confirm:
                track.state = "confirmed"

            if track.state in ("stale", "tentative") and since_seen >= self.delete_timeout_s:
                track.state = "deleted"
                to_delete.append(track_id)

            if track.state == "confirmed" and since_seen >= self.stale_timeout_s:
                track.state = "stale"

        for track_id in to_delete:
            del self._tracks[track_id]

    def _predict_latlon(self, track: _Track, horizon_s: float) -> Tuple[float, float]:
        # straight line prediction from current velocity
        dx_m = track.vx_mps * horizon_s
        dy_m = track.vy_mps * horizon_s

        pred_lat = track.lat + (dy_m / METERS_PER_DEG_LAT)
        meters_per_deg_lon = METERS_PER_DEG_LAT * cos(radians(track.lat))
        if abs(meters_per_deg_lon) < 1e-6:
            pred_lon = track.lon
        else:
            pred_lon = track.lon + (dx_m / meters_per_deg_lon)
        return pred_lat, pred_lon

    def _distance_m(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        # 2d distance in meters using local flat earth approximation
        dx_m, dy_m = self._delta_m(lat1, lon1, lat2, lon2)
        return sqrt(dx_m * dx_m + dy_m * dy_m)

    def _delta_m(self, lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float]:
        # returns east_m and north_m from point1 to point2
        avg_lat = (lat1 + lat2) * 0.5
        meters_per_deg_lon = METERS_PER_DEG_LAT * cos(radians(avg_lat))
        dx_m = (lon2 - lon1) * meters_per_deg_lon
        dy_m = (lat2 - lat1) * METERS_PER_DEG_LAT
        return dx_m, dy_m

