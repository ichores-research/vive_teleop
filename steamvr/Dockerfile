FROM debian:bookworm-slim

ENV USER=steam
ENV HOMEDIR="/home/steam"

# ── Step 1: i386 multiarch ──────────────────────────────────────────────────
RUN dpkg --add-architecture i386

# ── Step 2: Install dependencies ─────────────────────────────────────────────
# We keep your original list as these are essential for the SteamVR runtime
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
        pulseaudio python3 tar xz-utils procps nano \
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && dpkg-reconfigure --frontend=noninteractive locales \
    && rm -rf /var/lib/apt/lists/*

# ── Step 3: Setup User & Entrypoint ──────────────────────────────────────────
RUN useradd -m "${USER}"

# Copy your udev rules
COPY udev/60-HTC-Vive-perms.rules /etc/udev/rules.d/60-HTC-Vive-perms.rules

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 5900
ENTRYPOINT ["/entrypoint.sh"]