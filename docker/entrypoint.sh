#!/bin/bash
set -e

# RENDER_MODE=gpu (default): NVIDIA via PRIME render offload (hybrid Intel/NVIDIA laptop).
# RENDER_MODE=software: Mesa llvmpipe, for machines without a usable GPU.
if [ "${RENDER_MODE}" = "software" ]; then
    unset __GLX_VENDOR_LIBRARY_NAME __NV_PRIME_RENDER_OFFLOAD __EGL_VENDOR_LIBRARY_FILENAMES
    export LIBGL_ALWAYS_SOFTWARE=1
    export MESA_GL_VERSION_OVERRIDE=4.5
fi

source /opt/ros/jazzy/setup.bash
if [ -f "${HOME}/ws/install/setup.bash" ]; then
    source "${HOME}/ws/install/setup.bash"
fi

exec "$@"
