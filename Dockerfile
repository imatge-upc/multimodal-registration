FROM ubuntu:15.10

# Install pcl dependencies
RUN apt-get update && apt-get install -y \
		g++ \
		cmake cmake-gui \
		doxygen \
		mpi-default-dev openmpi-bin openmpi-common \
		libflann1.8 libflann-dev \
		libeigen3-dev \
		libboost-all-dev \
		libvtk5.10-qt4 libvtk5.10 libvtk5-dev \
		libqhull* \
		libusb-dev \
		libgtest-dev \
		git-core freeglut3-dev pkg-config \
		build-essential	libxmu-dev libxi-dev \
		libusb-1.0-0-dev graphviz mono-complete \
		phonon-backend-gstreamer \
		phonon-backend-vlc \
		libopenni-dev \
	--no-install-recommends && rm -rf /var/lib/apt/lists/*
	#PCL will be compiled from trunk


# Install OpenCV dependencies
RUN apt-get update && apt-get install -y \
		libopencv-dev \ 
		build-essential \
		checkinstall \
		cmake \
		pkg-config \
		yasm \
		libtiff5-dev \
		libjpeg-dev \
		libjasper-dev \
		libavcodec-dev \
		libavformat-dev \
		libswscale-dev \
		libdc1394-22-dev \
		libxine2-dev \
		libgstreamer0.10-dev \
		libgstreamer-plugins-base0.10-dev \
		libv4l-dev \
		python-dev \
		python-numpy \
		libtbb-dev \
		libqt4-dev \
		libgtk2.0-dev \
#		libfaac-dev \
		libmp3lame-dev \
		libopencore-amrnb-dev \
		libopencore-amrwb-dev \
		libtheora-dev \
		libvorbis-dev \
		libxvidcore-dev \
		x264 \
		v4l-utils \
# 		ffmpeg \
		unzip \
	--no-install-recommends && rm -rf /var/lib/apt/lists/*
	# OpenCV will be compiled from trunk

# Install armadillo
RUN apt-get update && apt-get install -y \
		libarmadillo-dev \
	--no-install-recommends && rm -rf /var/lib/apt/lists/*

# Install GDB 7.8 (try to replace version)
#RUN apt-get install -y libpython3-stdlib libpython3.4
#COPY gdb_7.8-0ubuntu1_amd64.deb /
#RUN dpkg -i /gdb_7.8-0ubuntu1_amd64.deb 
#RUN chmod +s /usr/bin/gdb
RUN apt-get update && apt-get install -y \
		gdb gdbserver\
	--no-install-recommends && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
		g++-multilib \
	--no-install-recommends && rm -rf /var/lib/apt/lists/*

WORKDIR /src/

ENV DISPLAY :0 

