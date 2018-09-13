FROM ros:kinetic-perception

WORKDIR /workspace

RUN apt-get update -y && \
    apt-get install -y --no-install-recommends git \
    libpng-dev \
    build-essential \
    libfreetype6-dev \
    tmux \
    python-dev \
    python-wheel \
    python-pip \
    libatlas-base-dev && \
    apt-get install -y --no-install-recommends libpng12-dev && \
    pip install -e git+https://github.com/duckietown/duckietown-slimremote.git#egg=duckietown-slimremote && \
    apt-get remove -y git && \
    rm -rf /var/lib/apt/lists/*

COPY . agent
RUN pip install -e agent
CMD python agent/rosbridge.py

