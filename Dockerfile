# ============================================================
#  AART Slot Car Braking Module — Docker Toolchain
#  Target MCU : STM32G041J6M6 (Cortex-M0+)
#  Library    : libopencm3 (git submodule)
#  Host       : macOS Apple Silicon (linux/arm64)
#
#  Stages:
#    base      — Debian slim + system deps
#    toolchain — ARM GCC + OpenOCD + libopencm3 pre-built
#    cubeprog  — optional: + STM32CubeProgrammer
#
#  Build toolchain image:
#    docker build --target toolchain -t aart-brake:toolchain .
#
#  Build with CubeProgrammer (needs installer in ./installers/):
#    docker build --target cubeprog -t aart-brake:full .
# ============================================================

ARG DEBIAN_VERSION=bookworm
ARG ARM_GCC_VERSION=13.3.rel1
# Pin libopencm3 to a known-good commit with G0 support
ARG LIBOPENCM3_REF=master

# ── Stage 1: base ────────────────────────────────────────────
FROM debian:${DEBIAN_VERSION}-slim AS base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    make \
    # Fetch / unpack
    curl \
    ca-certificates \
    xz-utils \
    unzip \
    # Git (for libopencm3 submodule)
    git \
    # OpenOCD — from Debian package (Bookworm ships 0.12.0)
    openocd \
    # Python (for libopencm3 scripts + helpers)
    python3 \
    # Runtime
    xxd \
    nano \
    && rm -rf /var/lib/apt/lists/*

# ── Stage 2: toolchain ───────────────────────────────────────
FROM base AS toolchain

ARG ARM_GCC_VERSION
ARG LIBOPENCM3_REF

# ── ARM GCC (official ARM release, native arm64 binary) ──────
RUN set -eux; \
    URL="https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_GCC_VERSION}/binrel/arm-gnu-toolchain-${ARM_GCC_VERSION}-aarch64-arm-none-eabi.tar.xz"; \
    echo "Downloading ARM GCC ${ARM_GCC_VERSION}..."; \
    curl -fSL "${URL}" -o /tmp/arm-gcc.tar.xz; \
    mkdir -p /opt/arm-gcc; \
    tar -xJf /tmp/arm-gcc.tar.xz --strip-components=1 -C /opt/arm-gcc; \
    rm /tmp/arm-gcc.tar.xz; \
    /opt/arm-gcc/bin/arm-none-eabi-gcc --version

ENV PATH="/opt/arm-gcc/bin:${PATH}"

# ── Verify OpenOCD installed correctly ───────────────────────
RUN openocd --version

# ── libopencm3 — clone and pre-build for STM32G0 ─────────────
# Built once here; every firmware build reuses the cached result.
RUN set -eux; \
    git clone --depth=1 --branch "${LIBOPENCM3_REF}" \
        https://github.com/libopencm3/libopencm3.git \
        /opt/libopencm3; \
    cd /opt/libopencm3; \
    make TARGETS="stm32/g0" -j"$(nproc)"; \
    echo "libopencm3 built OK"

ENV LIBOPENCM3_DIR=/opt/libopencm3

# ── udev rules (copy to host with: docker cp aart-brake:/etc/udev/rules.d .) ──
RUN mkdir -p /etc/udev/rules.d && \
    cat > /etc/udev/rules.d/49-stlink.rules << 'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374f", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3753", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1366", ATTRS{idProduct}=="0101", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1366", ATTRS{idProduct}=="0105", MODE="0666", GROUP="plugdev"
EOF

# ── Workspace ─────────────────────────────────────────────────
WORKDIR /workspace
COPY firmware/ /workspace/firmware/
COPY scripts/  /workspace/scripts/
RUN chmod +x /workspace/scripts/*.sh

ENV CROSS_COMPILE=arm-none-eabi-

LABEL org.opencontainers.image.title="AART Brake Toolchain (libopencm3)" \
      org.opencontainers.image.description="STM32G041 cross-compile + OpenOCD, no proprietary libs" \
      org.opencontainers.image.version="1.1"

CMD ["/bin/bash"]

# ── Stage 3: cubeprog (OPTIONAL) ─────────────────────────────
# Requires installer in ./installers/SetupSTM32CubeProgrammer.linux
# Build: docker build --target cubeprog -t aart-brake:full .
FROM toolchain AS cubeprog

RUN apt-get update && apt-get install -y --no-install-recommends \
    default-jre-headless \
    libglib2.0-0 \
    libgl1 \
    && rm -rf /var/lib/apt/lists/*

ARG CUBEPROG_INSTALLER=SetupSTM32CubeProgrammer.linux
COPY installers/${CUBEPROG_INSTALLER} /tmp/cubeprog-installer.linux

RUN set -eux; \
    chmod +x /tmp/cubeprog-installer.linux; \
    /tmp/cubeprog-installer.linux \
        --mode unattended \
        --InstallFolder /opt/STM32CubeProgrammer; \
    rm /tmp/cubeprog-installer.linux

ENV PATH="/opt/STM32CubeProgrammer/bin:${PATH}"
RUN STM32_Programmer_CLI --version || true

CMD ["/bin/bash"]
