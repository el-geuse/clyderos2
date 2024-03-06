FROM althack/ros2:humble-cuda-gazebo-nvidia

# ** [Optional] Uncomment this section to install additional packages. **

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends python3-serial \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
COPY ${WORKSPACE}/ament_flake8.ini /opt/ros/humble/lib/python3.10/site-packages/ament_flake8/configuration/ament_flake8.ini
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# auto-source Gazebo
RUN echo "source /usr/share/gazebo/setup.bash" >> /home/ros/.bashrc