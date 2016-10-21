#ifndef FUSION_BASED_REG_ALLOC_H
#define FUSION_BASED_REG_ALLOC_H

#include "llvm/Pass.h"
#include "llvm/CodeGen/RegAllocRegistry.h"
#include "llvm/Support/raw_ostream.h"

//Verify functions of the LiveRangeEdit delegate

using namespace llvm {
    class FusionRegAlloc : public MachineFunctionPass,
                           public RegAllocBase,
                           private LiveRangeEdit::Delegate { //Baseado no RAGreedy
       
        //Context
        MachineFunction *MF;

        //Useful Interfaces
        const TargetInstrInfo *TII;
        const TargetRegisterInfo *TRI;
        RegisterClassInfo RCI;

        public:
            static char ID;

            FusionRegAlloc();

            virtual const char *getPassName() const;
            
            virtual void getAnalysisUsage(AnalysisUsage &AU) const;
            virtual void releaseMemory();
            virtual Spiller &spiller();
            virtual void enqueue(LiveInterval *LI);
            virtual LiveInterval *dequeue();
            virtual unsigned selectOrSplit(LiveInterval&, SmallVectorImpl<unsigned>&);
            virtual void aboutToRemoveInterval(LiveInterval &);
             
            // Perform register allocation.
            virtual bool runOnMachineFunction(MachineFunction &mf); 
    };
}

#endif
