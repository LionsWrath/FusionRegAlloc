#include "FusionRegAlloc.h"

char FusionRegAlloc::ID = 0;
FusionRegAlloc::FusionRegAlloc() : MachineFunctionPass(ID) {}

static RegisterRegAlloc FusionRegAlloc("fusionregalloc", "Fusion Based Register Allocation", createFusionRegisterAllocator);

void FusionRegAlloc::getAnalysisUsage(AnalysisUsage &AU) const {
    //Add Required
}

const char *FusionRegAlloc::getPassName() const {
    return "Fusion Based Register Allocation";
}

//outside class
FunctionPass* llvm::createFusionRegisterAllocator() {
    return new FusionRegAlloc();
}
