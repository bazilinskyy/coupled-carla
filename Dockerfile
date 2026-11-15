# Coupled CARLA's map maker and SUMO-side tools in a container (Linux). No CARLA inside: CARLA runs on the host.
# How to build and use it: docs/docker.md.
#   docker build -t coupled-carla .
#   docker run --rm -it --network host -v "$PWD":/coupled-carla --user "$(id -u):$(id -g)" coupled-carla
FROM python:3.12-slim

# libgomp: SUMO's netconvert; libgl1, libglib2.0-0, libxrender1, libxext6, libsm6: what the SUMO and Pillow/matplotlib
# wheels load at import time even without a display; ca-certificates for the open data services (HTTPS)
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates libgomp1 libgl1 libglib2.0-0 libxrender1 libxext6 libsm6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /coupled-carla
# the dependencies first (a layer that survives code changes), then the code
COPY pyproject.toml README.md LICENSE ./
COPY coupledcarla/__init__.py coupledcarla/__init__.py
RUN pip install --no-cache-dir -e ".[maps]"
COPY . .
RUN pip install --no-cache-dir --no-deps -e .

# SUMO from the eclipse-sumo wheel; PROJ data for netconvert's projections
ENV SUMO_HOME=/usr/local/lib/python3.12/site-packages/sumo
ENV PATH=$SUMO_HOME/bin:$PATH \
    PROJ_LIB=/usr/local/lib/python3.12/site-packages/sumo/data/proj \
    PYTHONUNBUFFERED=1 \
    HOME=/home/coupled-carla \
    MPLBACKEND=Agg
# a home any --user can write to (caches in ~/.cache/coupled-carla, matplotlib's config)
RUN mkdir -p $HOME && chmod 1777 $HOME \
    && netconvert --version | head -1 && python -c "import coupledcarla.mapmaker, sumolib, traci"

# the map maker's page (python -m coupledcarla.mapmaker --web) listens on 127.0.0.1:8777 only: run the container
# with --network host so that the host's browser reaches it at http://127.0.0.1:8777/
EXPOSE 8777
CMD ["coupled-carla-map", "--web", "--no-browser"]
