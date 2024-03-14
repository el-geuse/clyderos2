FROM althack/ros2:humble-cuda-gazebo-nvidia

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends python3-serial curl \
   && curl -s https://install.husarnet.com/install.sh | bash \
   && curl -L https://github.com/husarnet/husarnet-dds/releases/download/v1.3.5/husarnet-dds-linux-amd64 -o /usr/local/bin/husarnet-dds \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
   
COPY ${WORKSPACE}/ament_flake8.ini /opt/ros/humble/lib/python3.10/site-packages/ament_flake8/configuration/ament_flake8.ini


# Husarnet ROS 2 DDS configuration
# RUN chmod +x /usr/local/bin/husarnet-dds \
#    && export DISCOVERY_SERVER_PORT=11811 \
#    && husarnet-dds singleshot \
#    && fast-discovery-server -i 0 -x /var/tmp/husarnet-dds/fastdds-ds-server.xml

RUN chmod +x /usr/local/bin/husarnet-dds
RUN export DISCOVERY_SERVER_PORT=11811
RUN husarnet-dds singleshot
RUN fast-discovery-server -i 0 -x /var/tmp/husarnet-dds/fastdds-ds-server.xml

ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# auto-source Gazebo
RUN echo "source /usr/share/gazebo/setup.bash" >> /home/ros/.bashrc

# Set environment variables for Husarnet
ENV HUSARNET_JOINCODE fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/Baoygf74uwy7cFsVoa65pD
ENV HUSARNET_HOSTNAME clydepc2

