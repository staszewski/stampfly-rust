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

    for entry in std::fs::read_dir("vendor/vl53l3c").expect("read vendor/vl53l3c") {
        let path = entry.expect("read vendor/vl53l3c entry").path();
        if path.extension().and_then(|ext| ext.to_str()) != Some("c") {
            continue;
        }

        if path.file_name().and_then(|name| name.to_str()) == Some("vl53lx_platform.c") {
            continue;
        }

        vl53l3c.file(path);
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
