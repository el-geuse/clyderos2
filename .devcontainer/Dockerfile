FROM althack/ros2:humble-gazebo

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends python3-serial curl\
   && curl -s https://install.husarnet.com/install.sh | bash \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

# Adding Husarnet, a P2P VPN

ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# Set environment variables for Husarnet
ENV HUSARNET_JOINCODE your_join_code
ENV HUSARNET_HOSTNAME your_hostname