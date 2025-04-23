FROM ubuntu:latest

SHELL [ "/bin/bash" , "-c" ]

RUN apt update && apt upgrade -y

RUN apt install vim -y
RUN apt install build-essential -y


RUN apt install -y python3
RUN apt install python3-pip -y
RUN pip install --no-cache-dir torch torchvision torchaudio matplotlib scikit-learn pandas notebook requests --break-system-packages

#jupyter notebook --ip=0.0.0.0 --port=8888 --no-browser --allow-root

