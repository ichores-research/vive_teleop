FROM debian:bookworm-slim

ENV USER=steam
ENV HOMEDIR="/home/steam"

# ── Step 1: i386 multiarch ────────────────────────────────────────────────────
RUN dpkg --add-architecture i386

# ── Step 2: Install dependencies ─────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        lib32gcc-s1 lib32stdc++6 ca-certificates curl locales \
        xvfb x11vnc \
        libgl1 libgl1:i386 libgles2 libegl1 libegl1:i386 \
        libx11-6 libx11-6:i386 libxrandr2 libxrandr2:i386 \
        libxext6 libxext6:i386 libxtst6 libxtst6:i386 \
        libxxf86vm1 libxxf86vm1:i386 libxss1 libxss1:i386 \
        libglib2.0-0 libglib2.0-0:i386 libgtk2.0-0 libgtk2.0-0:i386 \
        libnss3 libnss3:i386 libdbus-1-3 libdbus-1-3:i386 \
        libudev1 libudev1:i386 libusb-1.0-0 libusb-1.0-0:i386 \
        libvulkan1 libvulkan1:i386 vulkan-tools \
        libsdl2-2.0-0 libsdl2-2.0-0:i386 \
        libpipewire-0.3-0 libpipewire-0.3-0:i386 \
        libxcb-dri3-0 libxcb-present0 libxcb-icccm4 libxcb-sync1 \
        libnm0 libtinfo6 libcap2 libasound2 libasound2:i386 \
        libcap2-bin \
        libglx-mesa0 libglx0 libgl1-mesa-dri \
        libglx-mesa0:i386 libgl1-mesa-dri:i386 \
        mesa-utils \
        libqt5core5a libqt5gui5 libqt5widgets5 libqt5dbus5 \
        libqt5network5 libqt5opengl5 libopengl0 \
        libqt5multimedia5 libqt5multimediawidgets5 \
        libqt5printsupport5 \
        gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
        pipewire pipewire-pulse wireplumber libspa-0.2-modules \
        pulseaudio \
        dbus dbus-x11 udev usbutils \
        python3 tar xz-utils procps nano \
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && dpkg-reconfigure --frontend=noninteractive locales \
    && rm -rf /var/lib/apt/lists/*

# ── Step 3: Setup User & Entrypoint ──────────────────────────────────────────
RUN useradd -m "${USER}"

COPY udev/60-HTC-Vive-perms.rules /etc/udev/rules.d/60-HTC-Vive-perms.rules
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 5900
ENTRYPOINT ["/entrypoint.sh"]