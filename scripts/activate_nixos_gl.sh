# NixOS keeps GPU drivers in /run/opengl-driver, which libraries from
# conda-forge/pixi don't search. No-op on other distros.
if [ -e /etc/NIXOS ] && [ -d /run/opengl-driver ]; then
  export LD_LIBRARY_PATH=/run/opengl-driver/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
  if [ -z "${__EGL_VENDOR_LIBRARY_FILENAMES:-}" ]; then
    export __EGL_VENDOR_LIBRARY_DIRS="${__EGL_VENDOR_LIBRARY_DIRS:-/run/opengl-driver/share/glvnd/egl_vendor.d}"
  fi
  export MUJOCO_GL="${MUJOCO_GL:-egl}"
fi
