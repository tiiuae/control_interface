FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-latest AS builder

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb & writes it to /main_ws/
RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:stable

ENTRYPOINT /entrypoint.sh

COPY entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/ros-*-control-interface_*_amd64.deb /control-interface.deb

# need update because ROS people have a habit of removing old packages pretty fast
RUN apt update && apt install -y mavsdk \
	&& dpkg -i /control-interface.deb && rm /control-interface.deb

