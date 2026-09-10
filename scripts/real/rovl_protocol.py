"""Read-only Cerulean ROV Locator protocol and geometry helpers."""

from __future__ import division

import math


ROVL_FIELD_NAMES = (
    "ab", "ac", "ae", "sr", "tb", "cb", "te", "er", "ep", "ey",
    "ch", "db", "ah", "ag", "ls", "im", "oc", "idx", "idq",
)

try:
    from geographiclib.geodesic import Geodesic
except ImportError:  # The GUI remains usable with the documented dependency installed later.
    Geodesic = None


def nmea_checksum(body):
    """Return the NMEA XOR checksum for text between $ and *."""
    value = 0
    for byte in body.encode("ascii", "strict"):
        value ^= byte
    return value


def split_and_verify_nmea(sentence):
    """Return ``(body, fields, raw)`` after strict checksum verification."""
    if isinstance(sentence, bytes):
        sentence = sentence.decode("ascii", "replace")
    raw = sentence.strip()
    if not raw.startswith("$") or "*" not in raw:
        raise ValueError("not an NMEA sentence")
    body, checksum_text = raw[1:].split("*", 1)
    checksum_text = checksum_text.strip()
    if len(checksum_text) < 2:
        raise ValueError("missing NMEA checksum")
    try:
        expected = int(checksum_text[:2], 16)
    except ValueError:
        raise ValueError("invalid NMEA checksum")
    actual = nmea_checksum(body)
    if actual != expected:
        raise ValueError("NMEA checksum mismatch: expected %02X, got %02X" % (expected, actual))
    return body, body.split(","), raw


def _float_or_none(value):
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def parse_rovl_sentence(sentence):
    """Parse a checksum-verified ``$USRTH`` sentence.

    The known fields are read by their published positions. Missing/empty
    fields become ``None`` and fields appended by future firmware are retained
    in ``extra_fields`` and otherwise ignored by the viewer.
    """
    body, fields, raw = split_and_verify_nmea(sentence)
    if not fields or fields[0] != "USRTH":
        return None
    values = {}
    payload = fields[1:]
    for index, name in enumerate(ROVL_FIELD_NAMES):
        value = payload[index] if index < len(payload) else ""
        values[name] = _float_or_none(value) if name not in ("im", "oc") else (value or None)
    values.update(
        {
            "message_type": "USRTH",
            "raw": raw,
            "fields": fields,
            "extra_fields": payload[len(ROVL_FIELD_NAMES):],
        }
    )
    return values


def _nmea_coordinate(value, hemisphere, latitude):
    if not value or not hemisphere:
        return None
    try:
        degrees_digits = 2 if latitude else 3
        degrees = float(value[:degrees_digits])
        minutes = float(value[degrees_digits:])
        result = degrees + minutes / 60.0
        if hemisphere.upper() in ("S", "W"):
            result = -result
        return result
    except (TypeError, ValueError):
        return None


def parse_gps_sentence(sentence):
    """Parse GGA/RMC GPS NMEA without sending anything to the receiver."""
    body, fields, raw = split_and_verify_nmea(sentence)
    if not fields:
        return None
    kind = fields[0]
    if len(kind) < 5 or kind[0] != "G" or kind[2:] not in ("GGA", "RMC"):
        return None
    if kind[2:] == "GGA":
        if len(fields) < 7 or not fields[6] or fields[6] == "0":
            return {"fix": False, "raw": raw, "sentence_type": kind}
        lat = _nmea_coordinate(fields[2], fields[3], True)
        lon = _nmea_coordinate(fields[4], fields[5], False)
        return {"fix": lat is not None and lon is not None, "lat": lat, "lon": lon, "raw": raw, "sentence_type": kind}
    if len(fields) < 7 or fields[2].upper() != "A":
        return {"fix": False, "raw": raw, "sentence_type": kind}
    lat = _nmea_coordinate(fields[3], fields[4], True)
    lon = _nmea_coordinate(fields[5], fields[6], False)
    return {"fix": lat is not None and lon is not None, "lat": lat, "lon": lon, "raw": raw, "sentence_type": kind}


def relative_position(rovl):
    """Return ``east_m``, ``north_m`` and horizontal range from $USRTH."""
    slant = rovl.get("sr")
    bearing = rovl.get("cb")
    elevation = rovl.get("te")
    if slant is None or bearing is None or elevation is None:
        return None
    horizontal = math.cos(math.radians(elevation)) * slant
    return {
        "slant_range_m": float(slant),
        "horizontal_range_m": float(horizontal),
        "bearing_deg": float(bearing),
        "elevation_deg": float(elevation),
        "east_m": math.sin(math.radians(bearing)) * horizontal,
        "north_m": math.cos(math.radians(bearing)) * horizontal,
    }


def forward_geodesic(latitude, longitude, bearing_deg, distance_m):
    """Project a topside lat/lon using WGS84 when geographiclib is available."""
    if latitude is None or longitude is None:
        return None
    if Geodesic is not None:
        result = Geodesic.WGS84.Direct(latitude, longitude, bearing_deg, distance_m)
        return result["lat2"], result["lon2"]
    # Offline fallback for environments that have not installed requirements.
    radius = 6371008.8
    lat1 = math.radians(latitude)
    lon1 = math.radians(longitude)
    bearing = math.radians(bearing_deg)
    angular = float(distance_m) / radius
    lat2 = math.asin(math.sin(lat1) * math.cos(angular) + math.cos(lat1) * math.sin(angular) * math.cos(bearing))
    lon2 = lon1 + math.atan2(math.sin(bearing) * math.sin(angular) * math.cos(lat1), math.cos(angular) - math.sin(lat1) * math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)
