#include "FusionRegAlloc.h"

char FusionRegAlloc::ID = 0;

static RegisterRegAlloc FusionRegAlloc("fusionregalloc", 
        "Fusion Based Register Allocation", createFusionRegisterAllocator);

FunctionPass* llvm::createFusionRegisterAllocator() {
    return new FusionRegAlloc();
}

//------------------------------------------------------------------

//Constructor
FusionRegAlloc::FusionRegAlloc() : MachineFunctionPass(ID) {
    //Interface needed
    initializeLiveIntervalsPass(*PassRegistry::getPassRegistry());
    initializeVirtRegMapPass(*PassRegistry::getPassRegistry());
    initializeLiveRegMatrixPass(*PassRegistry::getPassRegistry());
}

//Return the Pass Name
const char* FusionRegAlloc::getPassName() const {
    return "Fusion Based Register Allocation";
}

//Maybe we will have to add more
void FusionRegAlloc::getAnalysisUsage(AnalysisUsage &AU) const {
    AU.setPreservesCFG();
    
    //Interface needed
    AU.addRequired<LiveIntervals>();
    AU.addPreserved<LiveIntervals>();
    AU.addRequired<VirtRegMap>();
    AU.addPreserved<VirtRegMap>();
    AU.addRequired<LiveRegMatrix>();
    AU.addPreserved<LiveRegMatrix>();
    
    MachineFunctionPass::getAnalysisUsage(AU);
}

//Interface functions - Some missing
void releaseMemory() {
    SpillerInstance.reset();
}

Spiller &spiller() { return *SpillerInstance; }

unsigned selectOrSplit(LiveIntervals &VirtReg, SmallVectorImpl<unsigned> &splitLRVs) {}

bool runOnMachineFunction(MachineFunction &mf) {
    //Initialize Variables
    MF = &mf;

    //Not sure if needed
    TII = MF->getSubtarget().getInstrInfo();
    TRI = MF->getSubtarget().getRegisterInfo()i;
    RCI.runOnMachineFunction(mf);

    //Initialize Interface
    RegAllocBase::init(getAnalysis<VirtRegMap>(),
                       getAnalysis<LiveIntervals>(),
                       getAnalysis<LiveRegMatrix>());
   
    allocatePhysRegs(); //Interface that calls seedLiveRegs
    //Clean memory and return changed
    releaseMemory();
    return true;
}


