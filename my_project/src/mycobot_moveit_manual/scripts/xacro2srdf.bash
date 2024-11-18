#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `mycobot_description` package
# https://github.com/AndrejOrsula/panda_ign_moveit2/blob/master/panda_description/scripts/xacro2sdf.bash


SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/srdf/panda.srdf.xacro"
SRDF_PATH="$(dirname "${SCRIPT_DIR}")/srdf/panda.srdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=panda
)

# Remove old SRDF file
rm "${SRDF_PATH}" 2>/dev/null

# Process xacro into SRDF
xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" -o "${SRDF_PATH}" &&
echo "Created new ${SRDF_PATH}"