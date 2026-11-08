//! Minimal `webbrowser::open` for egui-winit: hands the URL to the desktop shell.

pub fn open(url: &str) -> std::io::Result<()> {
    let cmd = if cfg!(windows) { "explorer" } else if cfg!(target_os = "macos") { "open" } else { "xdg-open" };
    std::process::Command::new(cmd).arg(url).spawn().map(|_| ())
}
