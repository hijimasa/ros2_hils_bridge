# hils_bridge_gps

GPS HILS bridges. Each sub-package converts simulator NavSatFix (and optionally TwistStamped for velocity) into the device's wire protocol.

| Package | Target Device | Real Driver |
|---------|--------------|-------------|
| [hils_bridge_gps_nmea0183](hils_bridge_gps_nmea0183/) | Any NMEA 0183 GPS receiver (GGA/RMC) | nmea_navsat_driver |

NMEA 0183 is an industry standard, so `hils_bridge_gps_nmea0183` works with any compliant receiver. Vendor-specific binary protocols (e.g. u-blox UBX) would live in their own `hils_bridge_gps_<vendor>_<series>` packages when implemented.
