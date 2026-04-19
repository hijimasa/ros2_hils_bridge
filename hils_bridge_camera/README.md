# hils_bridge_camera

Camera HILS bridges. Each sub-package converts simulator Image into the camera's wire protocol so the real camera driver runs unmodified.

| Package | Target Device | Real Driver | Firmware |
|---------|--------------|-------------|----------|
| [hils_bridge_camera_uvc](hils_bridge_camera_uvc/) | Any UVC-compliant USB camera (MJPEG) | usb_cam / cv_camera | rp2040_camera_uvc + rp2040_camera_uvc_spi_sender |

UVC (USB Video Class) is a USB-IF standard, so `hils_bridge_camera_uvc` works with any UVC driver. Vendor-specific protocols (RealSense, GigE Vision, Genicam) would live in their own sub-packages following the `hils_bridge_camera_<vendor>_<series>` convention.
