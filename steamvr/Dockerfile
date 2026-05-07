# ══════════════════════════════════════════════════════════════════
# Stage 1 — Vive Pro 2 driver build
# ══════════════════════════════════════════════════════════════════
FROM debian:bookworm-slim AS vive-builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl git ca-certificates build-essential pkg-config \
    gcc-mingw-w64-x86-64 \
 && update-ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# Rust (nightly required)
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
    sh -s -- -y --default-toolchain nightly --profile minimal

ENV PATH="/root/.cargo/bin:${PATH}"

RUN rustup target add x86_64-pc-windows-gnu

# Clone + verify
RUN git clone --depth=1 \
    https://github.com/CertainLach/VivePro2-Linux-Driver.git /vive-src \
 && test -f /vive-src/Cargo.toml

WORKDIR /vive-src

# Build Linux driver
RUN cargo +nightly build --release

# Build lens-server (Windows)
RUN cargo +nightly build --release \
    -p lens-server \
    --target x86_64-pc-windows-gnu

# Stage like install.sh
RUN cp target/x86_64-pc-windows-gnu/release/lens-server.exe \
       dist-proxy/lens-server/lens-server.exe


# ══════════════════════════════════════════════════════════════════
# Stage 2 — Monado build
# ══════════════════════════════════════════════════════════════════
FROM debian:bookworm-slim AS monado-builder

RUN apt-get update && apt-get install -y \
    git ca-certificates \
    cmake ninja-build pkg-config \
    libvulkan-dev \
    libx11-dev libxrandr-dev libxext-dev \
    libxcb1-dev libxcb-dri2-0-dev libxcb-dri3-dev \
    libxcb-present-dev libx11-xcb-dev \
    libxxf86vm-dev \
    libegl1-mesa-dev libgl1-mesa-dev \
    libwayland-dev wayland-protocols \
    libdrm-dev libudev-dev \
    libhidapi-dev libusb-1.0-0-dev \
    libbluetooth-dev \
    libglm-dev \
 && update-ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# Clone with verification
RUN set -eux; \
    git clone --depth=1 https://gitlab.freedesktop.org/monado/monado.git /monado; \
    ls -la /monado; \
    test -f /monado/CMakeLists.txt

WORKDIR /monado

# Configure build (CMake, not Meson)
RUN cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release

# Build
RUN cmake --build build

# Install into /usr/local inside image
RUN cmake --install build


# ══════════════════════════════════════════════════════════════════
# Stage 3 — Runtime image
# ══════════════════════════════════════════════════════════════════
FROM debian:bookworm-slim

ENV USER=steam
ENV HOMEDIR=/home/steam

# Multiarch (needed for Wine + some VR deps)
RUN dpkg --add-architecture i386

# Core runtime deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl locales \
    xorg x11vnc xvfb \
    dbus dbus-x11 udev usbutils \
    wine wine64 \
    lib32gcc-s1 lib32stdc++6 \
    libgl1 libgl1:i386 libegl1 libegl1:i386 \
    libx11-6 libx11-6:i386 \
    libxrandr2 libxrandr2:i386 \
    libxext6 libxext6:i386 \
    libxtst6 libxtst6:i386 \
    libxxf86vm1 libxxf86vm1:i386 \
    libxss1 libxss1:i386 \
    libglib2.0-0 libglib2.0-0:i386 \
    libgtk2.0-0 libgtk2.0-0:i386 \
    libnss3 libnss3:i386 \
    libdbus-1-3 libdbus-1-3:i386 \
    libudev1 libudev1:i386 \
    libusb-1.0-0 libusb-1.0-0:i386 \
    libvulkan1 libvulkan1:i386 vulkan-tools \
    libsdl2-2.0-0 libsdl2-2.0-0:i386 \
    libpipewire-0.3-0 libpipewire-0.3-0:i386 \
    pipewire pipewire-pulse wireplumber \
    pulseaudio \
    libglx-mesa0 libgl1-mesa-dri \
    libglx-mesa0:i386 libgl1-mesa-dri:i386 \
    mesa-utils \
    python3 procps nano \
 && sed -i 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
 && dpkg-reconfigure --frontend=noninteractive locales \
 && rm -rf /var/lib/apt/lists/*


# ── Copy artifacts ───────────────────────────────────────────────

# Vive driver
COPY --from=vive-builder /vive-src/dist-proxy /opt/vive/driver/viveVR

# Monado runtime
COPY --from=monado-builder /usr/local /usr/local


# ── Verify critical binaries ─────────────────────────────────────
RUN ls /opt/vive/driver/viveVR \
 && which monado-service \
 && test -f /usr/local/share/openxr/1/openxr_monado.json


# ── Environment for OpenXR ───────────────────────────────────────
ENV XRT_RUNTIME_JSON=/usr/local/share/openxr/1/openxr_monado.json
ENV XRT_COMPOSITOR_FORCE_XCB=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-steam


# ── User + entrypoint ────────────────────────────────────────────
RUN useradd -m "${USER}"

COPY udev/60-HTC-Vive-perms.rules /etc/udev/rules.d/
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


EXPOSE 5900

ENTRYPOINT ["/entrypoint.sh"]