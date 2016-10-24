#ifndef FUSION_BASED_REG_ALLOC_H
#define FUSION_BASED_REG_ALLOC_H

#include "llvm/Pass.h"
#include "llvm/CodeGen/RegAllocRegistry.h"
#include "llvm/Support/raw_ostream.h"

#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetSubtargetInfo.h"

using namespace llvm;

//Verify functions of the LiveRangeEdit delegate

namespace {
    class FusionRegAlloc : public MachineFunctionPass,
                           public RegAllocBase,
                           private LiveRangeEdit::Delegate { //Baseado no RAGreedy
       
        //Context
        MachineFunction *MF;

        //State
        std::unique_ptr<Spiller> SpillerInstance;

        //Useful Interfaces
        const TargetInstrInfo *TII;
        //Present in RegAllocBase - needed to remove?
        const TargetRegisterInfo *TRI;     
        RegisterClassInfo RCI;              //Provides dynamic information about target register classes

        public:
            static char ID;

            FusionRegAlloc();

            virtual const char *getPassName() const;
            
            virtual void getAnalysisUsage(AnalysisUsage &AU) const;
            virtual void releaseMemory();
            virtual Spiller &spiller();
            virtual void enqueue(LiveInterval *LI);
            virtual LiveInterval* dequeue();
            virtual unsigned selectOrSplit(LiveInterval&, SmallVectorImpl<unsigned>&);
            virtual void aboutToRemoveInterval(LiveInterval &);
             
            virtual bool runOnMachineFunction(MachineFunction &mf); 
    };
}

#endif
