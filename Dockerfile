#  Copyright (C) 2024 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

# NAV2 extensions build file
# Use ROS Humble as the base image
FROM ros:humble-ros-base

ARG GIT_BRANCH="develop"
ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"
ENV ROS_DISTRO=humble

RUN mkdir -p /root/c1t_ws/src/navigation2_extensions

COPY . /root/c1t_ws/src/navigation2_extensions/
COPY docker/install_dependencies.sh /root/c1t_ws/
COPY docker/build.sh /root/c1t_ws/

WORKDIR /root/c1t_ws/

RUN chmod +x ./install_dependencies.sh ./build.sh
RUN ./install_dependencies.sh
RUN ./build.sh

LABEL org.label-schema.schema-version="1.0" \
      org.label-schema.name="navigation2-extensions" \
      org.label-schema.description="CARMA 1tenth nav2 extensions" \
      org.label-schema.vendor="Leidos" \
      org.label-schema.version="${VERSION}" \
      org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA" \
      org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/navigation2_extensions" \
      org.label-schema.vcs-ref="${VCS_REF}" \
      org.label-schema.build-date="${BUILD_DATE}"

ENTRYPOINT ["/bin/bash"]
