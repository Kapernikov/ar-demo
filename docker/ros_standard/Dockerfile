FROM ros:kinetic
MAINTAINER Frank Dekervel <frank@kapernikov.com>

WORKDIR /


# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-pip \
    python-zmq wget curl \
    && rm -rf /var/lib/apt/lists/*


### install eclipse neon
RUN cd /opt \
	&& wget 'http://mirror.ibcp.fr/pub/eclipse//technology/epp/downloads/release/neon/2/eclipse-cpp-neon-2-linux-gtk-x86_64.tar.gz' \
	&& tar -zxf eclipse-cpp-neon-2-linux-gtk-x86_64.tar.gz \
	&& rm eclipse-cpp-neon-2-linux-gtk-x86_64.tar.gz \
	&& ln -sf /opt/eclipse/eclipse /usr/local/bin/eclipse



RUN apt-get update && apt-get install -y openjdk-8-jdk \
    ros-kinetic-camera-info-manager ros-kinetic-cv-bridge less vim joe libavcodec-extra \
    && rm -rf /var/lib/apt/lists/*

RUN cd /opt \
	&& wget 'http://s.baslerweb.com/media/documents/pylon-5.0.5.9000--RC8-x86_64.tar.gz' \
	&& tar -zxf pylon-5.0.5.9000--RC8-x86_64.tar.gz \
	&& cd /opt/pylon-5.0.5.9000-x86_64 \
	&& mkdir /etc/udev/rules.d \
	&& (yes | bash ./setup-usb.sh) \
	&& cd /opt \
	&& tar -zxf /opt/pylon-5.0.5.9000-x86_64/pylonSDK-5.0.5.9000-x86_64.tar.gz


RUN ln -sf /usr/bin/python /usr/local/bin/python
ADD bashrc_ros /etc/bashrc_ros