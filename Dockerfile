FROM duckietown/rpi-ros-kinetic-base
RUN [ "cross-build-start" ]

WORKDIR /workspace

#This file is expected by duckietown-slimremote, but this directory is read-only
#TODO: Instead, we should be setting an environement parameter for RPi and read that from duckietown-slimremote/setup.py
#RUN mkdir -p  /sys/firmware/devicetree/base
#RUN echo "Raspberry" > /sys/firmware/devicetree/base/model

RUN git clone https://github.com/duckietown/gym-duckietown-agent agent

RUN apt-get update -y && \
    apt-get install -y --no-install-recommends git \
    python-dev \
    libpng-dev \
    python-pip \
    python-wheel \
    build-essential \
    libfreetype6-dev \
    tmux \
    libatlas-base-dev && \
    apt-get install -y --no-install-recommends libpng12-dev && \
    pip install -e git+https://github.com/duckietown/duckietown-slimremote.git#egg=duckietown-slimremote && \
    apt-get remove -y git && \
    rm -rf /var/lib/apt/lists/*

COPY agent-ros.py agent
COPY ros-envs/envs agent/gym_duckietown_agent/envs
COPY ros-envs/__init__.py agent/gym_duckietown_agent/__init__.py

RUN pip install -e agent
RUN [ "cross-build-end" ]
    
# setup entrypoint
#COPY ./ros_entrypoint.sh /

# ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# CMD python agent/agent-ros.py --no-render
