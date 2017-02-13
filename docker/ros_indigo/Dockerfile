FROM ros:indigo
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
kd



RUN apt-get update && apt-get install -y openjdk-8-jdk \
    ros-indigo-camera-info-manager ros-indigo-cv-bridge less vim joe libavcodec-extra \
    && rm -rf /var/lib/apt/lists/*

RUN cd /opt \
	&& wget 'http://s.baslerweb.com/media/documents/pylon-5.0.5.9000--RC8-x86_64.tar.gz' \
	&& tar -zxf pylon-5.0.5.9000--RC8-x86_64.tar.gz \
	&& cd /opt/pylon-5.0.5.9000-x86_64 \
	&& mkdir /etc/udev/rules.d \
	&& (yes | bash ./setup-usb.sh) \
	&& cd /opt \
	&& tar -zxf /opt/pylon-5.0.5.9000-x86_64/pylonSDK-5.0.5.9000-x86_64.tar.gz

RUN apt-get update && apt-get install -y \
    libzmq3-dev libzmqpp3 libczmq-dev libzmqpp-dev libopencv-dev \
    libicu-dev freeglut3 freeglut3-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libxine2-dev \
    && rm -rf /var/lib/apt/lists/*


RUN cd /opt \
	&& wget http://sourceforge.net/projects/aruco/files/1.3.0/aruco-1.3.0.tgz \
	&& tar zxvf aruco-1.3.0.tgz \
	&& cd aruco-1.3.0 \
	&& mkdir build && cd build \
	&& OpenCV_DIR=/usr/share/OpenCV/ cmake .. \
	&& make -j2 && make install \
	&& rm -rf /opt/aruco*

RUN ln -sf /usr/bin/python /usr/local/bin/python
ADD bashrc_ros /etc/bashrc_ros