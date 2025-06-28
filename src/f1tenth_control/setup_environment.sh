#!/bin/bash
# Otimizações específicas para ARM64/RPi4B

# Flags de compilação ARM64
export CFLAGS="-O3 -march=armv8-a+simd -mcpu=cortex-a72 -mtune=cortex-a72 -mfpu=neon-fp-armv8"
export CXXFLAGS="$CFLAGS -std=gnu++17"
export LDFLAGS="-Wl,-O1 -Wl,--as-needed"

# Otimização Python
export PYTHONOPTIMIZE=2
export PYTHON_MALLOC_STATS=0

# Build otimizado
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS="-j2"

# DDS middleware otimizado
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Configuração de rede para ROS2 (ajuste o caminho se necessário)
export CYCLONEDX_URI="file://${HOME}/cyclonedx.xml"

echo "Ambiente otimizado configurado para ARM64/RPi4B"
echo "Use 'source setup_environment.sh' antes de compilar." 