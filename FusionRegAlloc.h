#ifndef FUSION_BASED_REG_ALLOC_H
#define FUSION_BASED_REG_ALLOC_H

#include "llvm/Pass.h"
#include "llvm/CodeGen/RegAllocRegistry.h"
#include "llvm/Support/raw_ostream.h"

#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetSubtargetInfo.h"

#include "Interference.h"

using namespace llvm;

//Verify functions of the LiveRangeEdit delegate

namespace {
    class FusionRegAlloc : public MachineFunctionPass,
                           public RegAllocBase { 

        // Interference Graph Implementation
        typedef std::list<INode> InterferenceGraph;

        //Context
        MachineFunction *MF;

        //State
        std::unique_ptr<Spiller> SpillerInstance;
        InterferenceGraph IGraph;

        //Useful Interfaces
        //const TargetInstrInfo *TII;
        //Present in RegAllocBase - needed to remove?
        //const TargetRegisterInfo *TRI;     
        //RegisterClassInfo RCI;              //Provides dynamic information about target register classes

        public:
            static char ID;

            FusionRegAlloc();

            virtual const char* getPassName() const;
            virtual void getAnalysisUsage(AnalysisUsage&) const;

            virtual bool runOnMachineFunction(MachineFunction&); 
            
            //Interface base 
            virtual void releaseMemory();
            virtual Spiller& spiller();
            virtual void enqueue(LiveInterval*);
            virtual LiveInterval* dequeue();
            virtual unsigned selectOrSplit(LiveInterval&, SmallVectorImpl<unsigned>&);
            virtual void aboutToRemoveInterval(LiveInterval&);
             
        private:
            void buildGraph();
            void removeNodeFromGraph(INode*);

            // Return a Free register - not sure
            unsigned getFreeReg(const LiveInterval* LI);
    };
}

#endif
