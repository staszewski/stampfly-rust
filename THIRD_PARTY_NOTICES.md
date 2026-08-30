# Third-party notices

## STMicroelectronics VL53LX driver

The files under `vendor/vl53l3c`, except for the project-specific files listed
below, are derived from the VL53LX driver distributed by STMicroelectronics as
part of X-CUBE-TOF1 / STSW-IMG015:

- <https://github.com/STMicroelectronics/x-cube-tof1>
- <https://www.st.com/en/embedded-software/stsw-img015.html>

The vendored driver identifies itself as VL53LX version 1.2.8, revision 2578.
It is redistributed under the license declarations in the individual source
files and the BSD 3-Clause license in `vendor/vl53l3c/LICENSE.md`.

The following files are project-specific integration code and are covered by
the repository's MIT OR Apache-2.0 license:

- `vendor/vl53l3c/rust_fly_tof_bridge.c`
- `vendor/vl53l3c/rust_fly_vl53lx_platform.c`
