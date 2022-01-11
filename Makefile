MPY_TREE := $(PWD)/../micropython

export PATH := $(MPY_TREE)/tools:$(MPY_TREE)/ports/unix:$(MPY_TREE)/mpy-cross:$(PATH)

MPY_VERSION := 1.17
VERSION ?= 3.1.0
PORT ?= /dev/ttyUSB0
SDK_ENV := docker run --rm -v $(HOME):$(HOME) -u $(UID):$(GID) -w $(PWD) "registry.hub.docker.com/larsks/esp-open-sdk"

FROZEN_MANIFEST := $(PWD)/manifest.py

all: firmware

requirements:
	mkdir -p lib
	curl -s -o lib/mqtt_as.py https://raw.githubusercontent.com/peterhinch/micropython-mqtt/master/mqtt_as/mqtt_as.py

	# asyncio v3 primitives from Peter Hinch
	mkdir -p lib/primitives
	curl -s -o lib/primitives/__init__.py https://raw.githubusercontent.com/peterhinch/micropython-async/master/v3/primitives/__init__.py

firmware:
	$(SDK_ENV) make -C $(MPY_TREE)/ports/esp8266 clean-modules
	$(SDK_ENV) make -C $(MPY_TREE)/ports/esp8266 \
		FROZEN_MANIFEST=$(FROZEN_MANIFEST) \
		MICROPY_VFS_FAT=0 \
		MICROPY_PY_BTREE=0 \

clean:
	$(SDK_ENV) make -C $(MPY_TREE)/ports/esp8266 clean

deploy: erase flash

erase:
	esptool.py --port $(PORT) --baud 460800 erase_flash

flash:
	esptool.py --port $(PORT) --baud 460800 write_flash  --flash_size=4MB -fm dio 0x0 $(MPY_TREE)/ports/esp8266/build-GENERIC/firmware-combined.bin

micropython: $(MPY_TREE)
	git clone https://github.com/micropython/micropython.git $(MPY_TREE)
	cd $(MPY_TREE); git checkout v$(MICROPYVERSION); git submodule init lib/axtls lib/berkeley-db-1.xx; git submodule update
	$(SDK_ENV) make -C $(MPY_TREE) mpy-cross
	make -C $(MPY_TREE)/ports/unix

	$(SDK_ENV) make -C $(MPY_TREE)/ports/esp8266 submodules 

bootstrap: micropython requirements


# linting!
black:
	find homie -name '*.py' | grep -v with_errors | xargs black --line-length=79 --safe $(ARGS)

isort:
	isort --recursive --apply homie

autoflake:
	find homie -name '*.py' | xargs autoflake --in-place --remove-unused-variables

# isort: must come first as black reformats the imports again
# black: must be >19
lint: autoflake isort black
