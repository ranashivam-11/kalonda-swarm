# Kalonda swarm - top-level build. `make help` lists the targets.
PY      ?= python3
JOBS    ?= 4
BUILD   := firmware/build

.PHONY: help firmware asan esp32 test test-fast campaign campaign-quick benchmarks ja figures all clean

help:
	@echo "firmware        host build of the C core + unit tests (needs cmake, a C99 compiler)"
	@echo "asan            the same under AddressSanitizer/UBSan"
	@echo "esp32           ESP-IDF build of the node image (needs ESP-IDF 5.x in the environment)"
	@echo "test            full Python suite (builds the SIL library first)"
	@echo "campaign        firmware-in-the-loop campaign behind the README results (~2 h on 4 cores)"
	@echo "campaign-quick  1/12-length smoke run of the campaign"
	@echo "benchmarks ja figures   model references, J-A identification, plots"
	@echo "all             firmware test campaign benchmarks ja figures"

firmware:
	cmake -S firmware -B $(BUILD) -DCMAKE_BUILD_TYPE=Release
	cmake --build $(BUILD) -j $(JOBS)
	cd $(BUILD) && ctest --output-on-failure

asan:
	cmake -S firmware -B firmware/build-asan -DCMAKE_BUILD_TYPE=Debug -DKS_SANITIZE=ON
	cmake --build firmware/build-asan -j $(JOBS)
	cd firmware/build-asan && ctest --output-on-failure

esp32:
	cd firmware/ports/esp32 && idf.py build

test: firmware
	$(PY) -m pytest python/tests

test-fast: firmware
	$(PY) -m pytest python/tests -k "not dynamo_energy_balance and not decisions_obey"

campaign: firmware
	$(PY) scripts/run_sil_campaign.py -j $(JOBS)

campaign-quick: firmware
	$(PY) scripts/run_sil_campaign.py -j $(JOBS) --quick

benchmarks:
	$(PY) scripts/model_benchmarks.py

ja:
	$(PY) scripts/identify_ja.py -j $(JOBS)

figures:
	$(PY) scripts/make_figures.py

all: firmware test campaign benchmarks ja figures

clean:
	rm -rf firmware/build firmware/build-asan
