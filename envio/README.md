# FusionRegAlloc
Fusion Based Register Allocator

Para compilar use o CMake no diretório root do projeto:

    cmake -G "Unix Makefiles" .

Para executar o alocador de registradores usando o llc no diretório root do projeto:

llc -load RegAllocLLVMRegAllocFusion.so -regalloc=fusion <bitcode file>


