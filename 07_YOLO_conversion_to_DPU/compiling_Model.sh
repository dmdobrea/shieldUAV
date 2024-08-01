#!/bin/sh

if [ $1 = zcu102 ]; then
      ARCH=/opt/vitis_ai/compiler/arch/DPUCZDX8G/ZCU102/arch.json
      TARGET=zcu102
      echo "-----------------------------------------"
      echo "COMPILING MODEL FOR ZCU102.."
      echo "-----------------------------------------"
elif [ $1 = zcu104 ]; then
      ARCH=/opt/vitis_ai/compiler/arch/DPUCZDX8G/ZCU104/arch.json
      TARGET=zcu104
      echo "-----------------------------------------"
      echo "COMPILING MODEL FOR ZCU104.."
      echo "-----------------------------------------"
elif [ $1 = vck190 ]; then
      ARCH=/opt/vitis_ai/compiler/arch/DPUCVDX8G/VCK190/arch.json
      TARGET=vck190
      echo "-----------------------------------------"
      echo "COMPILING MODEL FOR VCK190.."
      echo "-----------------------------------------"
elif [ $1 = v70 ]; then
      ARCH=/opt/vitis_ai/compiler/arch/DPUCV2DX8G/V70/arch.json   
      TARGET=v70
      echo "-----------------------------------------"
      echo "COMPILING MODEL FOR ALVEO V70.."
      echo "-----------------------------------------"
elif [ $1 = vek280 ]; then
      ARCH=/opt/vitis_ai/compiler/arch/DPUCV2DX8G/VEK280/arch.json
      TARGET=vek280
      echo "-----------------------------------------"
      echo "COMPILING MODEL FOR VEK280"
      echo "-----------------------------------------"
elif [ $1 = vck5000 ]; then
      ARCH=/opt/vitis_ai/compiler/arch/DPUCVDX8H/VCK50004PE/arch.json
      TARGET=vck5000
      echo "-----------------------------------------"
      echo "COMPILING MODEL FOR VCK5000"
      echo "-----------------------------------------"
elif [ $1 = KR260 ]; then
      ARCH=/opt/vitis_ai/compiler/arch/DPUCZDX8G/KV260/arch.json
      TARGET=KR260
      echo "-----------------------------------------"
      echo "COMPILING MODEL FOR KR260"
      echo "-----------------------------------------"
else
      echo  "Target not found. Valid choices are: zcu102, zcu104, vck190, vck5000, vek280, v70 ...exiting"
      exit 1
fi

CNN_MODEL=$2

compile() {
    vai_c_tensorflow2 \
	--model           build4/${CNN_MODEL} \
	--arch            $ARCH \
	--output_dir      build4/compiled_${TARGET} \
	--net_name        ${TARGET}_${CNN_MODEL} \
        --options         '{"input_shape": "1,416,416,3"}'
#	--options         "{'mode':'debug'}" 
}

compile #2>&1 | tee build/compile_$TARGET.log

echo "-----------------------------------------"
echo "MODEL COMPILED"
echo "-----------------------------------------"
