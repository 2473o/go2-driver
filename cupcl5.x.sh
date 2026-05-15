#!/usr/bin/env bash

SCRIPT_ROOT=$(cd $(dirname "${BASH_SOURCE[0]}"); pwd)

# jetpack 5.x cuPCL path
# export CUPCL_PATH=/home/unitree/ros2_ws/3rdparty/cuPCL

# X86_64 Library Path
export CUPCL_PATH=$SCRIPT_ROOT/lib/cuPCL

CUPCL_MODULES=(cuCluster cuFilter cuICP cuNDT cuOctree cuSegmentation)

for module in "${CUPCL_MODULES[@]}"; do
  lib_dir="${CUPCL_PATH}/${module}/lib"
  if [ -d "${lib_dir}" ]; then
    case ":${LD_LIBRARY_PATH}:" in
      *":${lib_dir}:"*) ;;
      *)
        if [ -n "${LD_LIBRARY_PATH}" ]; then
          export LD_LIBRARY_PATH="${lib_dir}:${LD_LIBRARY_PATH}"
        else
          export LD_LIBRARY_PATH="${lib_dir}"
        fi
        ;;
    esac
  fi
done

echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"
