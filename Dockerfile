FROM duckietown/rpi-duckiebot-base:master18

RUN ["cross-build-start"]

WORKDIR /workspace

RUN pip install -e git+https://github.com/duckietown/duckietown-slimremote.git#egg=duckietown-slimremote

RUN pip install --user --upgrade pillow

COPY rosagent.py ./
COPY setup.py ./

RUN pip install -e .

# Change from here
RUN /bin/bash -c "mkdir -p catkin_ws/src/"

# Copy or init your packages in here
COPY dt_dependent_node catkin_ws/src/dt_dependent_node
RUN chmod +x catkin_ws/src/dt_dependent_node/dt_dependent_node.py

RUN /bin/bash -c "cd catkin_ws/src/"

# Do not change the below line! This ensures that your workspace is overlayed on top of the Duckietown stack!  
# MAKE sure this line is present in the build: This workspace overlays: /home/software/catkin_ws/devel;/opt/ros/kinetic
RUN /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash && catkin_init_workspace && cd ../.."
RUN /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash && catkin_make -j -C catkin_ws/"
RUN /bin/bash -c "source catkin_ws/devel/setup.bash"


RUN ["cross-build-end"]
