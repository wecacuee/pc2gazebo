#FROM ros:melodic-perception-bionic
FROM votenet

RUN apt-get update && \
    apt-get install -y wget apt-transport-https && \
    rm -rf /var/lib/apt/lists/*

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

COPY apt-get-requirements.txt /tmp/apt-get-requirements.txt
RUN apt-get update && \
    apt-get install -y $(cat /tmp/apt-get-requirements.txt) && \
    rm -rf /var/lib/apt/lists/*


COPY pip-requirements.txt /tmp/pip-requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/pip-requirements.txt
RUN pip install --no-cache-dir -r /tmp/pip-requirements.txt


ENV CUDA_VERSION 10.0.130

ENV CUDA_PKG_VERSION 10.0=$CUDA_VERSION-1
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
cuda-cudart-$CUDA_PKG_VERSION && \
ln -s cuda-10.0 /usr/local/cuda && \
rm -rf /var/lib/apt/lists/*




















