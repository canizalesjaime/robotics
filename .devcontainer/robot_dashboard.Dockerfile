FROM ubuntu:latest

SHELL [ "/bin/bash" , "-c" ]

# ubuntu basics programming c++ and python
RUN apt update && apt install -y \
    neovim \
    git \
    curl \
    git \
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


RUN pip3 install beautifulsoup4 requests fastapi uvicorn ultralytics opencv-python --break-system-packages
RUN pip3 install --no-cache-dir torch torchvision torchaudio matplotlib scikit-learn pandas notebook --break-system-packages
RUN apt update && apt install -y libgl1 libglib2.0-0


# postgresql 
RUN apt update && apt install -y postgresql
RUN locale-gen en_US.UTF-8
RUN update-locale LANG=en_US.UTF-8
RUN echo "export LANG=en_US.UTF-8" >> ~/.bashrc
RUN echo "export LANGUAGE=en_US.UTF-8" >> ~/.bashrc
RUN echo "export LC_ALL=en_US.UTF-8" >> ~/.bashrc


#jupyter notebook --ip=0.0.0.0 --port=8888 --no-browser --allow-root
# ENV DEBIAN_FRONTEND=noninteractive

# # Create a new Vite React project & install Tailwind
# RUN npm create vite@latest my-robot-dashboard -- --template react && \
#     cd my-robot-dashboard && \
#     npm install && \
#     npm install -D tailwindcss postcss autoprefixer && \
#     npx tailwindcss init -p
