FROM ubuntu@sha256:dfc10878be8d8fc9c61cbff33166cb1d1fe44391539243703c72766894fa834a
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.12-dev \
    python3-pip \
    cmake \
    make \
    g++ \
    git
RUN rm -rf /var/lib/apt/lists/*
WORKDIR /home/cpp-project-template
COPY . .
RUN ./build_all.sh
EXPOSE 80
CMD ["/bin/bash", "-c", "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/cpp-project-template/build/install/lib && ./example/build/example_app && python3 example/main.py"]
