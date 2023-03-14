FROM zephyrprojectrtos/zephyr-build:latest

USER root

RUN mkdir -p /awaresite && chmod 777 /awaresite

USER user

RUN cd /awaresite && \
    west init -m https://github.com/nrfconnect/sdk-nrf --mr main && \
    west update && \
    west zephyr-export && \
    mkdir app

WORKDIR /awaresite/app

ENV ZEPHYR_BASE=/awaresite/zephyr
