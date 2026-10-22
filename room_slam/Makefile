# Convenience targets.  Run from the room_slam/ directory.
# Everything here assumes the virtualenv in .venv (see docs/STEP_BY_STEP_GUIDE.md).

PY      ?= .venv/bin/python
DB      ?= data/room.db
NAME    ?= room

.PHONY: help install demo record map live check replay test test-cpp clean

help:
	@echo "make install   install the python dependencies into .venv"
	@echo "make demo      simulate a room and build a map (no hardware needed)"
	@echo "make record    listen for the ESP32 and record to $(DB)"
	@echo "make live      record + rebuild the map every 60 sweeps"
	@echo "make map       build the map from $(DB)"
	@echo "make check     print live stream stats / ascii view"
	@echo "make test      run the python unit tests"
	@echo "make test-cpp  compile and run the lidar parser test"

install:
	python3 -m venv .venv
	.venv/bin/pip install --upgrade pip
	.venv/bin/pip install -r requirements.txt
	@echo "done - now run: make demo"

demo:
	$(PY) -m slam2d.simulate_room --out data/demo.db
	$(PY) -m slam2d.run_slam --db data/demo.db --name demo --show

record:
	$(PY) -m slam2d.record --db $(DB)

live:
	$(PY) -m slam2d.record --db $(DB) --snapshot-every 60

map:
	$(PY) -m slam2d.run_slam --db $(DB) --name $(NAME) --show

check:
	$(PY) -m slam2d.check_stream --seconds 15

replay:
	$(PY) -m slam2d.replay --db data/demo.db --host 127.0.0.1

test:
	$(PY) -m unittest discover -s unit_test -v

test-cpp:
	g++ -O2 -std=c++11 -Wall -Wextra -I firmware/esp32_rplidar_bridge \
	    unit_test/test_rplidar_parser.cpp -o /tmp/test_rplidar
	/tmp/test_rplidar

clean:
	rm -rf maps/*.png maps/*.g2o maps/*.npz data/demo.db data/demo.db-*
