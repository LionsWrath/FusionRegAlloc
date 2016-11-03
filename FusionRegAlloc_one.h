#ifndef FUSION_BASED_REG_ALLOC_H
#define FUSION_BASED_REG_ALLOC_H

#include "llvm/Pass.h"
#include "llvm/PassAnalysisSupport.h"

#include "llvm/CodeGen/RegAllocRegistry.h"
#include "llvm/CodeGen/LiveIntervalAnalysis.h"
#include "llvm/CodeGen/LiveRangeEdit.h"
#include "llvm/CodeGen/LiveRangeMatrix.h"
#include "llvm/CodeGen/LiveStackAnalysis.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegAllocRegistry.h"
#include "llvm/CodeGen/VirtRegMap.h"

#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

#include "Spiller.h"
#include "RegAllocBase.h"
#include "LiveDebugVariables.h"

#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetSubtargetInfo.h"

#include <set>

using namespace llvm;

#define DEBUG_TYPE "regalloc"

///-------------------------///
///    Interference Graph   ///
///-------------------------///

class INode { // Interference Node
    typedef std::set<INode*> NeighborList; // Change the data Structure

    LiveInterval* LI;
    NeighborList Neighbors;

    INode();

public:
    INode(LiveInterval* li) : LI(li) {}

    // Begin Iterator
    typedef NeighborList::iterator iterator;
    typedef NeighborList::const_iterator const_iterator;

    iterator begin()                { return Neighbors.begin(); }
    iterator end()                  { return Neighbors.end(); }
    const_iterator begin() const    { return Neighbors.begin(); }
    const_iterator end() const      { return Neighbors.end(); }
    // End Iterator
    
    // Check interference with another Node
    bool checkInterference(const INode* Node) const {
        return LI->overlaps(*Node->LI);
    }

    // Check interference with another Live Interval
    bool checkInterference(const LiveInterval& L) const {
        return LI->overlaps(L);
    }

    // Getter of live interval
    LiveInterval* getLiveInterval() { return LI; }
    const LiveInterval* getLiveInterval() const { return LI; }

    // Getter of register color associated
    // Register of the interval this node represents
    unsigned getColor() const {
        return LI->reg;
    }

    // Add Neighbor to NeighborList
    void addNeighbor(INode* Node) {
        Neighbors.insert(Node);
    }

    // Remove Neighbor to NeighborList
    void removeNeighbor(INode* Node) {
        Neighbors.erase(Node);
    }

    // Verify if exist a Interfering Node
    bool hasNeighbors() const {
        return !Neighbors.empty();
    }

    // Return number of interfering Nodes
    unsigned numNeighbors() const {
       return unsigned(Neighbors.size()); 
    }

    // Print the interval related to this node
    void print(std::ostream& Out) const {
        Out << "Node :: " << *LI;
    }
};

std::ostream& operator << (std::ostream& Out, const INode& N) {
    N.print(Out);
    return Out;
}

///-------------------------///
///      FusionRegAlloc     ///
///-------------------------///

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

            virtual bool runOnMachineFunction(MachineFunction&) = 0; 
            
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

//------------------------------------------------------------------Implementation

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
void FusionRegAlloc::releaseMemory() {
    SpillerInstance.reset();
}

Spiller &FusionRegAlloc::spiller() { return *SpillerInstance; }

void FusionRegAlloc::enqueue(LiveInterval* li) { return; }

LiveInterval* FusionRegAlloc::dequeue() { return nullptr; }

unsigned FusionRegAlloc::selectOrSplit(LiveInterval &VirtReg, SmallVectorImpl<unsigned> &splitLRVs) {}

bool FusionRegAlloc::runOnMachineFunction(MachineFunction &mf) {
    //Initialize Variables
    MF = &mf;

    //Not sure if needed
    //TII = MF->getSubtarget().getInstrInfo();
    //TRI = MF->getSubtarget().getRegisterInfo();
    //RCI.runOnMachineFunction(mf);

    //Initialize Interface
    RegAllocBase::init(getAnalysis<VirtRegMap>(),
                       getAnalysis<LiveIntervals>(),
                       getAnalysis<LiveRegMatrix>());
   
    allocatePhysRegs(); //Interface that calls seedLiveRegs
    //Clean memory and return changed
    releaseMemory();
    return true;
}

#endif
