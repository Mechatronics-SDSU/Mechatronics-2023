# @Zix
# Mechatronics Docker File
# Installs Dependencies (including ROS2) and has all convenience scripts needed
# If you have docker install you can copy this file and use the following commands 
#       sudo docker build -t "mechatronics" .
#       sudo docker run -it mechatronics
# Now you will be inside the terminal of the robot with the needed environment


FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive

# RUN apt-get install libxcb-xinerama0
# RUN apt-get install '^libxcb.*-dev' libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev

RUN \
    set -eux && \
    apt-get update && \
    apt-get -yq upgrade && \
    apt-get -yq install \
        aptitude apt-rdepends bash build-essential ccache clang clang-tidy cmake cppcheck curl doxygen diffstat gawk gdb git gnupg gperf iputils-ping \
        libboost-all-dev libfcgi-dev libgfortran5 libgl1-mesa-dev libjemalloc-dev libjemalloc2 libmlpack-dev libtbb-dev libyaml-cpp-dev \
        linux-tools-generic nano nasm ninja-build openjdk-11-jdk openssh-server openssl pkg-config python3 python-is-python3 spawn-fcgi \
        sudo tini unzip valgrind wget zip texinfo gcc-multilib chrpath socat cpio xz-utils debianutils libegl1-mesa \
        patch perl tar rsync bc libelf-dev libssl-dev libsdl1.2-dev xterm mesa-common-dev whois software-properties-common \
        libx11-xcb-dev libxcb-dri3-dev libxcb-icccm4-dev libxcb-image0-dev libxcb-keysyms1-dev libxcb-randr0-dev libxcb-render-util0-dev \
        libxcb-render0-dev libxcb-shape0-dev libxcb-sync-dev libxcb-util-dev libxcb-xfixes0-dev libxcb-xinerama0-dev libxcb-xkb-dev xorg-dev \
        libconfuse-dev libnl-3-dev libnl-route-3-dev libncurses-dev dh-autoreconf freeglut3 freeglut3-dev libglfw3-dev \
        apt-transport-https g++ graphviz xdot golang-go qt6-base-dev libglx-dev libx11-dev libmd4c-dev libmd4c-html0-dev \
        mesa-utils libglu1-mesa-dev libglew-dev libglm-dev libao-dev libmpg123-dev libclang-dev libxcb-xinerama0  '^libxcb.*-dev' \
        libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev && \
    go version && \
    ssh-keygen -A && \
    curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor > /usr/share/keyrings/bazel.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list && \
    apt-get update && \
    apt-get -yq install bazel && \
    bazel --version && \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb && \
    dpkg -i cuda-keyring_1.0-1_all.deb && \
    apt-get update && \
    apt-get -yq install cuda && \
    apt-get -yq autoremove && \
    apt-get -yq autoclean && \
    apt-get -yq clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* && \
    exit 0

# Install python pip
RUN \
    set -eux && \
    python3 --version && \
    curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python3 get-pip.py && \
    rm get-pip.py && \
    python3 -m pip install -U pip && \
    pip3 --version && \
    pip3 install --upgrade pip setuptools wheel && \
    pip3 --version && \
    exit 0

# Install python pip packages
RUN \
    set -eux && \
    pip3 --version && \
    pip3 install --upgrade pip setuptools wheel && \
    pip3 --version && \
    pip3 install --upgrade autoenv autopep8 cmake-format clang-format conan conan_package_tools meson && \
    pip3 install --upgrade cppclean flawfinder lizard pygments pybind11 GitPython pexpect subunit Jinja2 pylint CLinters && \
    exit 0

# Install vcpkg
RUN \
    set -eux && \
    cd /root && \
    git clone https://github.com/Microsoft/vcpkg.git && \
    cd vcpkg && \
    ./bootstrap-vcpkg.sh && \
    exit 0

# Install conan
RUN \
    set -eux && \
    conan profile new default --detect  && \
    conan profile update settings.compiler.libcxx=libstdc++11 default && \
    conan remote list && \
    conan remote add bincrafters https://api.bintray.com/conan/bincrafters/public-conan && \
    exit 0

# Install buildifier
RUN go install github.com/bazelbuild/buildtools/buildifier@latest

# Setup ssh
RUN \
    set -eux && \
    mkdir -p /var/run/sshd && \
    mkdir -p /root/.ssh && \
    sed -ri 's/^#?PermitRootLogin\s+.*/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -ri 's/UsePAM yes/#UsePAM yes/g' /etc/ssh/sshd_config && \
    groupadd -g 1000 myuser && \
    useradd --system --no-log-init --create-home --home-dir /home/myuser --gid myuser --groups sudo --uid 1000 --shell /bin/bash myuser && \
    echo 'root:root' | chpasswd && \
    echo 'myuser:myuser' | chpasswd && \
    exit 0

ENV IGNORE_CC_MISMATCH=1
ENV PATH=$PATH:/usr/local/cuda/bin
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64

#########################################################################################################################################################

RUN apt-get -y update
RUN apt-get -y upgrade
RUN apt-get install -y build-essential
RUN apt-get install -y cmake
RUN apt-get install -y gdb
RUN apt-get install -y git
RUN apt-get install -y nlohmann-json3-dev 
RUN apt-get install -y tmux
RUN apt-get install -y can-utils
RUN apt-get install -y g++
RUN apt-get install -y python3
RUN apt-get install -y python3-pip
RUN apt-get install -y curl  
RUN apt-get install -y wget 
RUN apt-get install -y xrdp 
RUN apt-get install libxcb-xinerama0

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends keyboard-configuration language-pack-en && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends wget curl git build-essential ca-certificates tzdata tmux gnupg2 \
        vim sudo lsb-release locales bash-completion zsh iproute2 iputils-ping net-tools dnsutils && \
    rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
RUN locale-gen en_US.UTF-8
ENV ROS_DISTRO=humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-desktop && \
    apt-get install -y --no-install-recommends python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-colcon-mixin python3-vcstool && \
    apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-joint-state-publisher-gui && \
    rm -rf /var/lib/apt/lists/*
RUN rosdep init && \
    rosdep update
EXPOSE 5555


COPY app/ app/
WORKDIR /app/scripts/ros_scripts
RUN . /opt/ros/${ROS_DISTRO}/setup.sh
# RUN ./build.sh
# RUN rm -r /usr/lib/python3.7/site-packages/cv2/qt/plugins 

COPY app/scripts/system /usr/local/bin/

WORKDIR /app


