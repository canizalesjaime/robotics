FROM ubuntu:latest

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    neovim \
    git \
    curl \
    ripgrep \
    fd-find \
    unzip \
    build-essential \
    python3 \
    python3-pip \
    nodejs \
    npm \
    tmux \
    ripgrep \
    fd-find \
    pandoc \
    poppler-utils \
    glow \
    zathura \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install beautifulsoup4 requests --break-system-packages
