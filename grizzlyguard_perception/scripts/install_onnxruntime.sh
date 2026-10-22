#!/usr/bin/env bash
# One-time install of the ONNX Runtime C++ release used by grizzlyguard_perception.
# Installs to ~/onnxruntime-linux-x64[-gpu]-<version> and points ~/onnxruntime at it,
# which is where CMakeLists.txt looks by default.
#
#   bash install_onnxruntime.sh          # CPU build (default)
#   bash install_onnxruntime.sh --gpu    # CUDA 12 build, for execution_provider: cuda
set -euo pipefail

VERSION="1.22.0"
FLAVOUR="linux-x64"
if [[ "${1:-}" == "--gpu" ]]; then FLAVOUR="linux-x64-gpu"; fi

NAME="onnxruntime-${FLAVOUR}-${VERSION}"
DEST="${HOME}/${NAME}"
URL="https://github.com/microsoft/onnxruntime/releases/download/v${VERSION}/${NAME}.tgz"

if [[ -f "${DEST}/include/onnxruntime_cxx_api.h" ]]; then
  echo "Already installed: ${DEST}"
else
  echo "Downloading ${URL}"
  curl -fL --retry 3 -o "/tmp/${NAME}.tgz" "${URL}"
  tar -xzf "/tmp/${NAME}.tgz" -C "${HOME}"
  rm -f "/tmp/${NAME}.tgz"
fi

if [[ -e "${HOME}/onnxruntime" && ! -L "${HOME}/onnxruntime" ]]; then
  echo "NOTE: ${HOME}/onnxruntime exists and is not a symlink; leaving it alone."
  echo "      Build with: colcon build --cmake-args -DONNXRUNTIME_ROOT=${DEST}"
else
  ln -sfn "${DEST}" "${HOME}/onnxruntime"
  echo "~/onnxruntime -> ${DEST}"
fi