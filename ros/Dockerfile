FROM osrf/ros:humble-desktop-full
ARG xauth_cookie
# Install ros and gui things
RUN apt update
RUN apt install -y ros-humble-dynamixel-sdk xauth
RUN xauth add ${xauth_cookie}

# Install neovim beta
RUN apt install -y ninja-build gettext libtool libtool-bin autoconf automake cmake g++ pkg-config unzip curl doxygen
RUN git clone https://github.com/neovim/neovim /tmp/neovim
WORKDIR "/tmp/neovim/"
RUN make CMAKE_BUILD_TYPE=RelWithDebInfo
RUN make install

# Install my neovim configs
RUN apt install -y nodejs npm python3.10-venv curl wget
RUN git clone https://github.com/devon-g/neovim /root/.config/nvim/ -b experimental

# Source ros setup
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

WORKDIR "/root/"
