FROM quay.io/jupyter/base-notebook:latest

# Install git to make setuptools_scm work
USER root
RUN apt update && \
    apt install --no-install-recommends --yes \
      build-essential \
      git && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*
USER ${NB_USER}

# Install compiler toolchain into the environment
RUN mamba install -c conda-forge gcc gxx

# Pull in the Conda environment first to avoid unnecessary rebuilds
COPY --chown=${NB_UID} environment-dev.yml ${HOME}/

# Install packages into the conda environment
RUN mamba env update -n base --file ${HOME}/environment-dev.yml && \
    mamba clean -a -q -y && \
    rm ${HOME}/environment-dev.yml

# Copy the rest of the files
COPY --chown=${NB_UID} . ${HOME}/helios

# Install the Helios package
RUN conda run -n base python -m pip install -v ${HOME}/helios
