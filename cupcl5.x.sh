#!/usr/bin/env bash

# jetpack 5.x cuPCL path
CUPCL_PATH=/home/unitree/ros2_ws/3rdparty/cuPCL
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
