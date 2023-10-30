FROM f1tenth_gym_ros
RUN \
    pip3 install --upgrade cython
RUN \
    git clone https://github.com/f1tenth/range_libc.git && \
    cd range_libc/pywrapper && \
    python3 setup.py install && \
    cd /sim_ws/src && \
    git clone https://github.com/f1tenth/particle_filter.git && \
    rosdep install -r --from-paths . --ignore-src --rosdistro foxy -y
