FROM ubuntu:16.10
MAINTAINER Frank Dekervel <frank@kapernikov.com>

RUN apt-get update && \
        apt-get install -y \
        build-essential \
        cmake \
        git \
        wget \
        unzip \
        yasm \
        pkg-config \
        libswscale-dev \
        libtbb2 \
        libtbb-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libavformat-dev \
        libpq-dev \
	llvm-dev python3-pip

RUN pip3 install numpy

WORKDIR /
RUN wget https://github.com/Itseez/opencv/archive/3.2.0.zip \
&& wget -O contrib.zip https://github.com/opencv/opencv_contrib/archive/3.2.0.zip \
&& unzip contrib.zip \
&& unzip 3.2.0.zip \
&& cd /opencv-3.2.0 \
&& mkdir /opencv-3.2.0/cmake_binary \
&& cd /opencv-3.2.0/cmake_binary \
&& cmake -DBUILD_TIFF=ON \
  -DBUILD_opencv_java=OFF \
  -DWITH_CUDA=OFF \
  -DENABLE_AVX=ON \
  -DOPENCV_EXTRA_MODULES_PATH=/opencv_contrib-3.2.0/modules \
  -DWITH_OPENGL=ON \
  -DWITH_OPENCL=ON \
  -DWITH_IPP=ON \
  -DWITH_TBB=ON \
  -DWITH_EIGEN=ON \
  -DWITH_V4L=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DCMAKE_BUILD_TYPE=RELEASE \
  -DCMAKE_INSTALL_PREFIX=$(python3.5 -c "import sys; print(sys.prefix)") \
  -DPYTHON_EXECUTABLE=$(which python3.5) \
  -DPYTHON_INCLUDE_DIR=$(python3.5 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
  -DPYTHON_PACKAGES_PATH=$(python3.5 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") .. \
&& make -j2 install \
&& rm /3.2.0.zip \
&& rm -r /opencv-3.2.0 \
&& rm -r /opencv_contrib-3.2.0

RUN pip3 install ipywidgets flake8 pep8 requests bs4 lxml --upgrade pip pandas jupyter seaborn matplotlib iso8601 xlrd xlsxwriter scikit-image scikit-learn tsfresh peakutils
RUN sh -c "jupyter nbextension enable --py --sys-prefix widgetsnbextension"


RUN echo "deb http://http.debian.net/debian jessie-backports main" > /etc/apt/sources.list.d/backports.list

#RUN apt-get update && apt-get install -y software-properties-common \
#    && add-apt-repository ppa:webupd8team/java -y  && apt-get update \
#    && apt-get install -y oracle-java8-installer oracle-java8-set-default \
#    ros-indigo-camera-info-manager ros-indigo-cv-bridge less vim joe libavcodec-extra \
#    && rm -rf /var/lib/apt/lists/*


RUN apt-get update && apt-get -y  install openjdk-9-jdk

RUN cd /opt \
	&& wget 'http://mirror.ibcp.fr/pub/eclipse//technology/epp/downloads/release/neon/2/eclipse-cpp-neon-2-linux-gtk-x86_64.tar.gz' \
	&& tar -zxf eclipse-cpp-neon-2-linux-gtk-x86_64.tar.gz \
	&& rm eclipse-cpp-neon-2-linux-gtk-x86_64.tar.gz \
	&& ln -sf /opt/eclipse/eclipse /usr/local/bin/eclipse

RUN apt-get update && apt-get -y install libeigen3-dev libflann-dev libvtk6-dev libqhull-dev libopenni-dev 
RUN apt-get -y install libpcl-dev pcl-tools cython3
RUN mkdir -p /opt/source
WORKDIR /opt/source
RUN git clone https://github.com/riyasdeen/python-pcl.git
WORKDIR /opt/source/python-pcl
RUN cat /usr/lib/x86_64-linux-gnu/pkgconfig/pcl_features-1.8.pc | sed -e 's|\spcl_2d-1.8||g;' > /tmp/n.pc && cp /tmp/n.pc  /usr/lib/x86_64-linux-gnu/pkgconfig/pcl_features-1.8.pc
RUN cat pcl/__init__.py  | sed -e 's|from\s\.pcl_visualization|\#NO PCL_VIS|g;' >  pcl/__init__.py-new && mv pcl/__init__.py-new pcl/__init__.py && python3 setup.py install

