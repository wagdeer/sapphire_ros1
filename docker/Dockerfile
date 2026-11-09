# syntax=docker/dockerfile:1.7

ARG ROS_BASE_IMAGE=ros:noetic-ros-base-focal

FROM ${ROS_BASE_IMAGE} AS runtime-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG APT_RETRIES=5
ARG UBUNTU_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu
ARG ROS_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu

LABEL org.opencontainers.image.title="Sapphire ROS 1 development toolchain" \
      org.opencontainers.image.description="ROS 1 Noetic toolchain for the shared Sapphire LIO core" \
      org.opencontainers.image.source="https://github.com/wagdeer/sapphire_ros1" \
      org.opencontainers.image.licenses="MIT"

ENV SAPPHIRE_DEPS_PREFIX=/opt/sapphire-deps
ENV CMAKE_PREFIX_PATH=${SAPPHIRE_DEPS_PREFIX}:/opt/ros/noetic
ENV LD_LIBRARY_PATH=${SAPPHIRE_DEPS_PREFIX}/lib:/opt/ros/noetic/lib:/usr/local/lib
ENV PKG_CONFIG_PATH=${SAPPHIRE_DEPS_PREFIX}/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
        | gpg --dearmor --yes \
            --output /usr/share/keyrings/ros-archive-keyring.gpg \
    && gpg --show-keys --with-colons \
            /usr/share/keyrings/ros-archive-keyring.gpg \
        | grep -q "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654" \
    && sed -i -E \
        "s#signed-by=[^ ]+#signed-by=/usr/share/keyrings/ros-archive-keyring.gpg#" \
        /etc/apt/sources.list.d/*.list \
    && sed -i -E \
        -e "s#https?://(archive|security)\.ubuntu\.com/ubuntu#${UBUNTU_MIRROR}#g" \
        -e "s#https?://snapshots\.ros\.org/noetic/final/ubuntu#${ROS_MIRROR}#g" \
        -e "s#https?://packages\.ros\.org/ros/ubuntu#${ROS_MIRROR}#g" \
        /etc/apt/sources.list /etc/apt/sources.list.d/*.list \
    && apt-get -o "Acquire::Retries=${APT_RETRIES}" update \
    && apt-get -o "Acquire::Retries=${APT_RETRIES}" install \
        -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        git \
        libboost-all-dev \
        libflann-dev \
        libgtest-dev \
        libmetis-dev \
        libomp-dev \
        libopenni2-dev \
        libspdlog-dev \
        libtbb-dev \
        libusb-1.0-0-dev \
        ninja-build \
        pkg-config \
        python3-catkin-tools \
        ros-noetic-pcl-conversions \
        ros-noetic-rosbag \
        ros-noetic-rviz \
        ros-noetic-tf2-ros \
        wget \
    && rm -rf /var/lib/apt/lists/*

FROM runtime-base AS dependency-builder

ARG DEBIAN_FRONTEND=noninteractive
ARG EIGEN_REF=3.4.0
ARG EIGEN_COMMIT=3147391d946bb4b6c68edd901f2add6ac1f31f8c
ARG PCL_REF=pcl-1.14.1
ARG PCL_COMMIT=5f608cfb5397fe848c7d61c4ae5f5b1ab760ba80
ARG GTSAM_REF=4.2.0
ARG GTSAM_COMMIT=4f66a491ffc83cf092d0d818b11dc35135521612
ARG SMALL_GICP_REF=v1.0.1
ARG SMALL_GICP_COMMIT=57c1106daf83c2c79ee0c58a9c7ed0032298ff4e
ARG TOMLPLUSPLUS_REF=v3.4.0
ARG TOMLPLUSPLUS_COMMIT=30172438cee64926dc41fdd9c11fb3ba5b2ba9de

RUN git clone --depth 1 --branch "${EIGEN_REF}" \
        https://gitlab.com/libeigen/eigen.git /tmp/eigen \
    && test "$(git -C /tmp/eigen rev-parse HEAD)" = "${EIGEN_COMMIT}" \
    && cmake -S /tmp/eigen -B /tmp/eigen/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${SAPPHIRE_DEPS_PREFIX}" \
        -DBUILD_TESTING=OFF \
    && cmake --build /tmp/eigen/build --parallel "$(nproc)" \
    && cmake --install /tmp/eigen/build \
    && rm -rf /tmp/eigen

RUN git clone --depth 1 --branch "${TOMLPLUSPLUS_REF}" \
        https://github.com/marzer/tomlplusplus.git /tmp/tomlplusplus \
    && test "$(git -C /tmp/tomlplusplus rev-parse HEAD)" = \
        "${TOMLPLUSPLUS_COMMIT}" \
    && cmake -S /tmp/tomlplusplus -B /tmp/tomlplusplus/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${SAPPHIRE_DEPS_PREFIX}" \
        -DTOMLPLUSPLUS_BUILD_TESTS=OFF \
        -DTOMLPLUSPLUS_BUILD_EXAMPLES=OFF \
    && cmake --build /tmp/tomlplusplus/build --parallel "$(nproc)" \
    && cmake --install /tmp/tomlplusplus/build \
    && rm -rf /tmp/tomlplusplus

RUN git clone --depth 1 --branch "${PCL_REF}" \
        https://github.com/PointCloudLibrary/pcl.git /tmp/pcl \
    && test "$(git -C /tmp/pcl rev-parse HEAD)" = "${PCL_COMMIT}" \
    && cmake -S /tmp/pcl -B /tmp/pcl/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${SAPPHIRE_DEPS_PREFIX}" \
        -DCMAKE_PREFIX_PATH="${SAPPHIRE_DEPS_PREFIX}" \
        -DCMAKE_INSTALL_RPATH="${SAPPHIRE_DEPS_PREFIX}/lib" \
        -DEigen3_DIR="${SAPPHIRE_DEPS_PREFIX}/share/eigen3/cmake" \
        -DPCL_SHARED_LIBS=ON \
        -DPCL_ENABLE_MARCHNATIVE=OFF \
        -DPCL_ENABLE_SSE=OFF \
        -DPCL_ENABLE_AVX=OFF \
        -DWITH_OPENGL=OFF \
        -DWITH_VTK=OFF \
        -DWITH_QT=OFF \
        -DBUILD_visualization=OFF \
        -DBUILD_apps=OFF \
        -DBUILD_examples=OFF \
        -DBUILD_tools=OFF \
        -DBUILD_simulation=OFF \
        -DBUILD_global_tests=OFF \
        -DBUILD_registration=ON \
        -DBUILD_filters=ON \
        -DBUILD_features=ON \
        -DBUILD_keypoints=OFF \
        -DBUILD_segmentation=ON \
        -DBUILD_surface=OFF \
        -DBUILD_recognition=OFF \
        -DBUILD_people=OFF \
        -DBUILD_tracking=OFF \
        -DBUILD_outofcore=OFF \
        -DBUILD_ml=OFF \
        -DBUILD_stereo=OFF \
        -DWITH_OPENNI2=OFF \
        -DWITH_ENSENSO=OFF \
        -DWITH_DAVIDSDK=OFF \
        -DWITH_DSSDK=OFF \
        -DWITH_PCAP=OFF \
        -DWITH_PNG=OFF \
        -DWITH_LIBUSB=OFF \
        -DWITH_OPENMP=ON \
    && cmake --build /tmp/pcl/build --parallel "$(nproc)" \
    && cmake --install /tmp/pcl/build \
    && rm -rf /tmp/pcl

RUN git clone --depth 1 --branch "${GTSAM_REF}" \
        https://github.com/borglab/gtsam.git /tmp/gtsam \
    && test "$(git -C /tmp/gtsam rev-parse HEAD)" = "${GTSAM_COMMIT}" \
    && cmake -S /tmp/gtsam -B /tmp/gtsam/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${SAPPHIRE_DEPS_PREFIX}" \
        -DCMAKE_PREFIX_PATH="${SAPPHIRE_DEPS_PREFIX}" \
        -DCMAKE_INSTALL_RPATH="${SAPPHIRE_DEPS_PREFIX}/lib" \
        -DEigen3_DIR="${SAPPHIRE_DEPS_PREFIX}/share/eigen3/cmake" \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF \
    && cmake --build /tmp/gtsam/build --parallel "$(nproc)" \
    && cmake --install /tmp/gtsam/build \
    && rm -rf /tmp/gtsam

RUN git clone --depth 1 --branch "${SMALL_GICP_REF}" \
        https://github.com/koide3/small_gicp.git /tmp/small_gicp \
    && test "$(git -C /tmp/small_gicp rev-parse HEAD)" = \
        "${SMALL_GICP_COMMIT}" \
    && cmake -S /tmp/small_gicp -B /tmp/small_gicp/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${SAPPHIRE_DEPS_PREFIX}" \
        -DCMAKE_PREFIX_PATH="${SAPPHIRE_DEPS_PREFIX}" \
        -DCMAKE_INSTALL_RPATH="${SAPPHIRE_DEPS_PREFIX}/lib" \
        -DEigen3_DIR="${SAPPHIRE_DEPS_PREFIX}/share/eigen3/cmake" \
        -DPCL_DIR="${SAPPHIRE_DEPS_PREFIX}/lib/cmake/pcl" \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_BENCHMARKS=OFF \
        -DBUILD_WITH_MARCH_NATIVE=OFF \
    && cmake --build /tmp/small_gicp/build --parallel "$(nproc)" \
    && cmake --install /tmp/small_gicp/build \
    && rm -rf /tmp/small_gicp

FROM runtime-base AS final

ARG USER_NAME=sapphire
ARG USER_UID=1000
ARG USER_GID=1000

COPY --from=dependency-builder \
    /opt/sapphire-deps /opt/sapphire-deps

RUN printf '%s\n' "${SAPPHIRE_DEPS_PREFIX}/lib" \
        > /etc/ld.so.conf.d/sapphire-deps.conf \
    && ldconfig \
    && groupadd --gid "${USER_GID}" "${USER_NAME}" \
    && useradd \
        --uid "${USER_UID}" \
        --gid "${USER_GID}" \
        --create-home \
        --shell /bin/bash \
        "${USER_NAME}" \
    && install -d -o "${USER_NAME}" -g "${USER_GID}" /workspace

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod 0755 /entrypoint.sh

USER ${USER_NAME}
WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

