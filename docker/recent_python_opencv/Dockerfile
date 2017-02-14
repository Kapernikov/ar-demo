FROM python:3.5
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
        libjasper-dev \
        libavformat-dev \
        libpq-dev \
	llvm-dev

RUN pip install numpy

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

RUN pip install ipywidgets flake8 pep8 requests bs4 lxml --upgrade pip pandas jupyter seaborn matplotlib iso8601 xlrd xlsxwriter scikit-image scikit-learn tsfresh peakutils
RUN sh -c "jupyter nbextension enable --py --sys-prefix widgetsnbextension"


RUN echo "deb http://http.debian.net/debian jessie-backports main" > /etc/apt/sources.list.d/backports.list

RUN cd /opt \
	&& wget 'http://s.baslerweb.com/media/documents/pylon-5.0.5.9000--RC8-x86_64.tar.gz' \
	&& tar -zxf pylon-5.0.5.9000--RC8-x86_64.tar.gz \
	&& cd /opt/pylon-5.0.5.9000-x86_64 \
	&& mkdir -p /etc/udev/rules.d \
	&& (yes | bash ./setup-usb.sh) \
	&& cd /opt \
	&& tar -zxf /opt/pylon-5.0.5.9000-x86_64/pylonSDK-5.0.5.9000-x86_64.tar.gz

