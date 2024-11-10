git pull
sudo docker build -t zmq-server -f Dockerfile.server .
sudo docker run --privileged -p 5555:5555 --device=/dev/ttyS1 -it zmq-server -e SERIAL_PORT=/dev/ttyS1


