// Local tangent-plane (NED) <-> WGS84 conversions (equirectangular, accurate
// to a few centimetres within a few kilometres of the origin).

const R_EARTH = 6378137.0;
const DEG = Math.PI / 180;

export class GeoOrigin {
  constructor(lat, lon, altAmsl) {
    this.set(lat, lon, altAmsl);
  }
  set(lat, lon, altAmsl) {
    this.lat = lat;
    this.lon = lon;
    this.alt = altAmsl;
    this.cosLat = Math.cos(lat * DEG);
  }
  /** NED metres -> { lat, lon, alt } (alt AMSL metres) */
  toGeo(ned) {
    return {
      lat: this.lat + (ned[0] / R_EARTH) / DEG,
      lon: this.lon + (ned[1] / (R_EARTH * this.cosLat)) / DEG,
      alt: this.alt - ned[2],
    };
  }
  /** lat/lon/(alt AMSL) -> NED metres */
  toNed(lat, lon, alt = this.alt) {
    return [
      (lat - this.lat) * DEG * R_EARTH,
      (lon - this.lon) * DEG * R_EARTH * this.cosLat,
      this.alt - alt,
    ];
  }
}

/** Great-circle-ish distance in metres between two lat/lon pairs (small distances). */
export function distance(lat1, lon1, lat2, lon2) {
  const x = (lon2 - lon1) * DEG * Math.cos(((lat1 + lat2) / 2) * DEG);
  const y = (lat2 - lat1) * DEG;
  return Math.hypot(x, y) * R_EARTH;
}

/** ISA barometric pressure (hPa) for an altitude AMSL (m). */
export const pressureAt = (altAmsl) => 1013.25 * Math.pow(1 - 2.25577e-5 * altAmsl, 5.25588);
/** ISA temperature (deg C). */
export const temperatureAt = (altAmsl) => 15 - 0.0065 * altAmsl;
