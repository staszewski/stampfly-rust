use std::path::{Path, PathBuf};

fn main() {
    embuild::espidf::sysenv::output();

    let mut vl53l3c = cc::Build::new();
    if let Some(compiler) = find_embuild_tool("xtensa-esp32s3-elf-gcc") {
        vl53l3c.compiler(compiler);
    }
    if let Some(archiver) = find_embuild_tool("xtensa-esp32s3-elf-ar") {
        vl53l3c.archiver(archiver);
    }
    vl53l3c.include("vendor/vl53l3c");
    vl53l3c.flag_if_supported("-Wno-unused-parameter");
    vl53l3c.flag_if_supported("-Wno-missing-field-initializers");
    vl53l3c.flag_if_supported("-Wno-sign-compare");

    const VL53L3C_SOURCES: &[&str] = &[
        "rust_fly_tof_bridge.c",
        "rust_fly_vl53lx_platform.c",
        "vl53lx_api.c",
        "vl53lx_api_calibration.c",
        "vl53lx_api_core.c",
        "vl53lx_api_debug.c",
        "vl53lx_api_preset_modes.c",
        "vl53lx_core.c",
        "vl53lx_core_support.c",
        "vl53lx_dmax.c",
        "vl53lx_hist_algos_gen3.c",
        "vl53lx_hist_algos_gen4.c",
        "vl53lx_hist_char.c",
        "vl53lx_hist_core.c",
        "vl53lx_hist_funcs.c",
        "vl53lx_nvm.c",
        "vl53lx_nvm_debug.c",
        "vl53lx_platform_init.c",
        "vl53lx_platform_ipp.c",
        "vl53lx_register_funcs.c",
        "vl53lx_sigma_estimate.c",
        "vl53lx_silicon_core.c",
        "vl53lx_wait.c",
        "vl53lx_xtalk.c",
    ];

    for source in VL53L3C_SOURCES {
        vl53l3c.file(Path::new("vendor/vl53l3c").join(source));
    }

    vl53l3c.compile("vl53l3c");
}

fn find_embuild_tool(tool_name: &str) -> Option<PathBuf> {
    find_file(
        Path::new(".embuild/espressif/tools/xtensa-esp-elf"),
        tool_name,
    )
}

fn find_file(dir: &Path, file_name: &str) -> Option<PathBuf> {
    let entries = std::fs::read_dir(dir).ok()?;
    for entry in entries.flatten() {
        let path = entry.path();
        if path.file_name().and_then(|name| name.to_str()) == Some(file_name) {
            return Some(path);
        }
        if path.is_dir() {
            if let Some(found) = find_file(&path, file_name) {
                return Some(found);
            }
        }
    }

    None
}
