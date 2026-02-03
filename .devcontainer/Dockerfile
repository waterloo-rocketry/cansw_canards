# Use Ubuntu as the base image
FROM --platform=linux/amd64 ubuntu:22.04

# Set non-interactive mode for apt-get
ENV DEBIAN_FRONTEND=noninteractive

# Install essential dependencies
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        wget \
        ca-certificates \
        gnupg && \
    wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - && \
    echo "deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-19 main" >> /etc/apt/sources.list && \
    apt-get update && \
    apt-get install --no-install-recommends -y \
        gcc g++ make libc6-dev \
        gdb \
        git \
        cmake \
        ninja-build \
        lcov \
        locales \
        xz-utils \
        nano \
        clang-format-19 && \
        ln -sf /usr/bin/clang-format-19 /usr/local/bin/clang-format && \
        sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
        locale-gen && \
        rm -rf /var/lib/apt/lists/*

# Set path for clang
ENV CLANG_FORMAT="/usr/bin/clang-format-19"

# Set locale, needed by lcov
ENV LANG=en_US.UTF-8  
ENV LANGUAGE=en_US:en  
ENV LC_ALL=en_US.UTF-8

# Set the working directory inside the container
WORKDIR /workspace

# Default command when the container starts
CMD ["/bin/bash"]
