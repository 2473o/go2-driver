ROOT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}"); pwd)
echo "ROOT_DIR: ${ROOT_DIR}"

build_package(){
package_name=$1
package_version=$2
shift 2
cmake_args=("$@")

if [ ! -d $ROOT_DIR/src/$package_name-$package_version/build ]; then
    mkdir $ROOT_DIR/src/$package_name-$package_version/build
else
    rm -rf $ROOT_DIR/src/$package_name-$package_version/build/*
fi

cd $ROOT_DIR/src/$package_name-$package_version/build

cmake .. -GNinja \
 -DCMAKE_BUILD_TYPE=Release \
 -DCMAKE_INSTALL_PREFIX=$ROOT_DIR/$package_name-$package_version \
 -DCMAKE_CXX_COMPILER_LAUNCHER=sccache \
 -DCMAKE_C_COMPILER_LAUNCHER=sccache \
  "${cmake_args[@]}"

 ninja && ninja install
}


build_unitree_sdk2(){

package_name="unitree_sdk2"
barnch_="main"
build_package $package_name $barnch_

}

build_unitree_sdk2