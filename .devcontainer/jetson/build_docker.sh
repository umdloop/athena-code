#!/usr/bin/env bash
set -euo pipefail

config_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${config_dir}/../.." && pwd)"
machine_type="$(basename -- "${config_dir}")"
image_name="athena-code:${machine_type}"

docker build \
  --file "${config_dir}/Dockerfile" \
  --build-arg "MACHINE_TYPE=${machine_type}" \
  --build-arg "USER_UID=$(id -u)" \
  --build-arg "USER_GID=$(id -g)" \
  --tag "${image_name}" \
  "${repo_root}"

echo "Built ${image_name}"
