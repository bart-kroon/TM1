FROM tmiv-checks:20231122

# Copy the test material for the integration test
COPY tmiv_integration_test /tmiv_integration_test

# Set-up ccache to speed up CI
RUN /usr/sbin/update-ccache-symlinks
ENV PATH=/usr/lib/ccache:$PATH
ENV CCACHE_DIR=/cache/.ccache

# Copy dependencies and build script
#
# Having built the dependencies locally, copy:
#   C:\Data\TM1\.deps\source -> source
#   C:\Data\TM1\scripts\build\build_dependencies.py -> build_dependencies.py
RUN mkdir -p /dependencies/build
COPY source /dependencies/source
COPY HM /dependencies/build/HM
COPY build_dependencies.json /dependencies/build/build_dependencies.json
COPY build_dependencies.py /dependencies/build/build_dependencies.py
COPY probe_compiler_features /dependencies/build/probe_compiler_features

# Build the dependencies in the Docker environment (Clang)
ENV CXX=clang++-17\ -w\ -stdlib=libc++
ENV CC=clang-17\ -w
RUN /dependencies/build/build_dependencies.py \
  --source-dir /dependencies/source \
  --build-dir /dependencies/build/clang-Debug \
  --install-dir /dependencies/install/clang-Debug \
  --build-type Debug -j 2 --cxx-standard 17

# Build the dependencies in the Docker environment (GCC)
ENV CXX=g++-13\ -w
ENV CC=gcc-13\ -w
RUN /dependencies/build/build_dependencies.py \
  --source-dir /dependencies/source \
  --build-dir /dependencies/build/gcc-Release \
  --install-dir /dependencies/install/gcc-Release \
  --build-type Release -j 2

# Remove the build directory but keep the source directory
RUN rm -r /dependencies/build
