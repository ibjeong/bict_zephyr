Setting Up Environment
----------------------

Build `awaresite-build` docker image.

    $ docker build -t awaresite-build docker/


Building the Code
-----------------

Run menuconfig

    $ docker run -it --rm -v ${PWD}/app:/awaresite/app awaresite-build west build -t menuconfig

Build app.

    $ docker run -it --rm -v ${PWD}/app:/awaresite/app awaresite-build west build
