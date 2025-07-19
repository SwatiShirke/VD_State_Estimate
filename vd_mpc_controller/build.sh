#!/bin/bash
echo ""
echo "building ackerman nmpc"
cd ../../..

source install/setup.bash
cd src/Vehicle_MPC/vd_mpc_controller
source ~/acados/mpcenv/bin/activate
cd scripts
python3 acados_controller.py 

cd ..
cp scripts/c_generated_code/ackerman_model_model/ackerman_model_model.h include/vd_mpc_controller
cp scripts/c_generated_code/acados_solver_ackerman_model.h include/vd_mpc_controller

cd ../../.. 
colcon build --packages-select vd_mpc_controller
source install/setup.bash



