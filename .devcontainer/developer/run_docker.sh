#!/usr/bin/env bash
set -euo pipefail

config_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${config_dir}/../.." && pwd)"
machine_type="$(basename -- "${config_dir}")"
image_name="athena-code:${machine_type}"
command=("$@")
if [[ ${#command[@]} -eq 0 ]]; then
  command=(bash)
fi

docker_args=(
  --rm
  --interactive
  --tty
  --network host
  --ipc host
  --privileged
  --env "ROS_DISTRO=humble"
  --env "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
  --volume "${repo_root}:/athena-code"
  --workdir /athena-code
)

if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
  docker_args+=(
    --env "DISPLAY=${DISPLAY}"
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
  )
fi

docker run "${docker_args[@]}" "${image_name}" "${command[@]}"
