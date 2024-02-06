#!/bin/bash
git submodule update --init --recursive
pip install virtualenv

virtualenv acados_env
source acados_env/bin/activate

pip install -e ./acados/interfaces/acados_template
pip install PyYAML
pip install rospkg

cd acados
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
# add more optional arguments e.g. -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> above
make install -j4
