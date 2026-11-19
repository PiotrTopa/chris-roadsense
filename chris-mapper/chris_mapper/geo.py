"""Small geodesy helpers. Angles: bearings in degrees clockwise from true north."""
import math

M_PER_DEG_LAT = 111194.9          # mean; the error over Poland is < 0.2 %


def m_per_deg(lat):
    """(metres per degree of latitude, metres per degree of longitude) at this latitude."""
    return M_PER_DEG_LAT, M_PER_DEG_LAT * math.cos(math.radians(lat))


def dist_m(lat1, lon1, lat2, lon2):
    mlat, mlon = m_per_deg(0.5 * (lat1 + lat2))
    return math.hypot((lat2 - lat1) * mlat, (lon2 - lon1) * mlon)


def bearing_deg(lat1, lon1, lat2, lon2):
    mlat, mlon = m_per_deg(0.5 * (lat1 + lat2))
    return math.degrees(math.atan2((lon2 - lon1) * mlon, (lat2 - lat1) * mlat)) % 360.0


def ang_diff(a, b):
    """a - b wrapped to (-180, 180] degrees."""
    d = (a - b) % 360.0
    return d - 360.0 if d > 180.0 else d


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def offset(lat, lon, north_m, east_m):
    mlat, mlon = m_per_deg(lat)
    return lat + north_m / mlat, lon + east_m / mlon


class Local:
    """Equirectangular tangent plane around (lat0, lon0): x east, y north, metres."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.mlat, self.mlon = m_per_deg(lat0)

    def to_xy(self, lat, lon):
        return (lon - self.lon0) * self.mlon, (lat - self.lat0) * self.mlat

    def to_ll(self, x, y):
        return self.lat0 + y / self.mlat, self.lon0 + x / self.mlon
