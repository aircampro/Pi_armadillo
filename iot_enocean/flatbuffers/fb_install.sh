sudo apt update
sudo apt -y install build-essential cmake git

export FLATBUFFERS_BUILD_PATH=.flatbuffers
git clone --depth 1 https://github.com/google/flatbuffers.git "${FLATBUFFERS_BUILD_PATH}"
cmake -S "${FLATBUFFERS_BUILD_PATH}" -B "${FLATBUFFERS_BUILD_PATH}" -G 'Unix Makefiles' -DFLATBUFFERS_BUILD_TEST=OFF -DFLATBUFFERS_BUILD_FLATLIB=OFF -DFLATBUFFERS_BUILD_FLATHASH=OFF

sudo make install -C "${FLATBUFFERS_BUILD_PATH}"

flatc -–version
flatc --python --gen-onefile *fbs
