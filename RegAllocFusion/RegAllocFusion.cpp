//===------- RegAllocFusion.cpp - Fusion Based Register Allocator * C++ -*-===//
//
//                  Put names and shit like that here
//
//===----------------------------------------------------------------------===//
//
// This file contains the Fusion Register Allocator.
//
//  TODO:
//      - Verify region formation with Basic Blocks in mind
//          - Get CFG ( Basic Blocks as Region )
//          - Order EDGES ( Priotity List )
//          - Create a Interference Graph for each region
//      - Check how many live ranges must be spilled within each region
//      - Graph Fusion
//          - Begin Fusing the graphs in the EDGE order
//          - Always maintain the simplificability
//          - Return an unique Interference Graph
//      - Color Assignment
//          - Choose a technique?
//      - Code Insertion
//
//  Classes to take a look:
//      - LiveRangeCalc.h       -> Compute Live Ranges from scratch
//      - LiveRangeAnalysis.h
//      - SplitKit.h            -> calcLiveBlockInfo
//      - MachineBasicBlock.h
//===----------------------------------------------------------------------===//

#include "llvm/Pass.h"
#include "llvm/PassAnalysisSupport.h"
#include "llvm/CodeGen/RegAllocRegistry.h"
#include "llvm/CodeGen/LiveInterval.h"
#include "llvm/CodeGen/LiveIntervalAnalysis.h"
#include "llvm/CodeGen/LiveRangeEdit.h"
#include "llvm/CodeGen/LiveRegMatrix.h"
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
#include "llvm/Support/BranchProbability.h"

#include "/home/lionswrath/llvm/lib/CodeGen/Spiller.h"
#include "/home/lionswrath/llvm/lib/CodeGen/RegAllocBase.h"
#include "/home/lionswrath/llvm/lib/CodeGen/LiveDebugVariables.h"

#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetSubtargetInfo.h"

#include <set>
#include <memory>
#include <queue>

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

    // Iterators
    typedef NeighborList::iterator iterator;
    typedef NeighborList::const_iterator const_iterator;

    iterator begin()                { return Neighbors.begin(); }
    iterator end()                  { return Neighbors.end(); }
    const_iterator begin() const    { return Neighbors.begin(); }
    const_iterator end() const      { return Neighbors.end(); }
    
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
    void print(raw_ostream& Out) const {
        Out << "Node :: " << LI; // Check LiveInterval printing
    }
};

raw_ostream& operator<< (raw_ostream& Out, const INode& N) {
    N.print(Out);
    return Out;
}

///-------------------------///
///      Region Graph       ///
///-------------------------///

class RNode {    
    // Interference Graph Implementation
    typedef std::list<INode> InterferenceGraph;
    typedef std::set<RNode*> RegionNeighborList; // Change set later - Successors

    int Number;

    InterferenceGraph IGraph;
    RegionNeighborList RNeighbors;

    //RNode();
public:
    RNode(int N) : Number(N) {}
    
    // Iterators
    typedef RegionNeighborList::iterator iterator;
    typedef RegionNeighborList::const_iterator const_iterator;

    iterator begin()                { return RNeighbors.begin(); }
    iterator end()                  { return RNeighbors.end(); }
    const_iterator begin() const    { return RNeighbors.begin(); }
    const_iterator end() const      { return RNeighbors.end(); }
   
    // Getter of live interval
    InterferenceGraph* getInterferenceGraph() { return &IGraph; }
    const InterferenceGraph* getInterferenceGraph() const { return &IGraph; }

    // Add Neighbor to NeighborList
    void addNeighbor(RNode* Node) {
        RNeighbors.insert(Node);
    }

    // Remove Neighbor to NeighborList
    void removeNeighbor(RNode* Node) {
        RNeighbors.erase(Node);
    }

    // Verify if exist a Interfering Node
    bool hasNeighbors() const {
        return !RNeighbors.empty();
    }

    // Return number of interfering Nodes
    unsigned numNeighbors() const {
       return unsigned(RNeighbors.size()); 
    }

    unsigned getNumber() const {
        return unsigned(Number);
    }

    // Print the interval related to this node
    void print(raw_ostream& Out) const {
        Out << "Region Number :  " << Number << '\n';
        
        for (auto it = RNeighbors.begin(); it != RNeighbors.end(); it++) {
            Out << "    Edge: " << (*it)->getNumber() << '\n';
        }
    }
};

raw_ostream& operator<< (raw_ostream& Out, const RNode& N) {
    N.print(Out);
    return Out;
}

///-------------------------///
///      FusionRegAlloc     ///
///-------------------------///

class FusionRegAlloc : public MachineFunctionPass,
                       public RegAllocBase { 

    typedef std::vector<RNode*> RegionGraph;
    typedef std::vector<MachineBasicBlock*> MachineBasicBlockList;
    
    MachineFunction *MF;
    LiveIntervals* LIS;
    VirtRegMap* VRM;

    std::unique_ptr<Spiller> SpillerInstance;
    
    RegionGraph RGraph;
    MachineBasicBlockList MBBList;

    // Priority Queue Implementation
    struct Edge {
        RNode *src, *dst;
        BranchProbability weight;

        Edge(RNode *s, RNode * d, BranchProbability w): src(s), dst(d), weight(w) {}

        //float getWeight() { return weight; }
        //void setWeight(float w) { weight = w; }

        bool operator<(const Edge &e) const {
            return weight < e.weight;
        }
    };

    struct CompEdges {
        bool operator() (Edge *A, Edge *B) const {
            return A < B;
        }
    };

    std::priority_queue<Edge*, std::vector<Edge*>, CompEdges> Queue;

    public:
        static char ID;

        FusionRegAlloc();

        //Override - Mudar tudo aqui
        const char* getPassName() const override;
        void getAnalysisUsage(AnalysisUsage&) const override;

        bool runOnMachineFunction(MachineFunction&) override; 
        
        //Interface base 
        void releaseMemory() override;
        Spiller& spiller() override;
        void enqueue(LiveInterval*) override;
        LiveInterval* dequeue() override;
        unsigned selectOrSplit(LiveInterval&, SmallVectorImpl<unsigned>&) override;
        void aboutToRemoveInterval(LiveInterval&) override;
         
    private:
        void buildRegionGraph();
        void linkSuccessors();
        void sortEdges();
        void addRegion(MachineBasicBlock*);
        RNode* getRegion(int);
        void printRegionGraph(raw_ostream&);
};

//------------------------------------------------------------------Implementation

char FusionRegAlloc::ID = 0;

FunctionPass* createFusionRegisterAllocator() {
    return new FusionRegAlloc();
}

static RegisterRegAlloc FusionRegAlloc("fusion", 
        "Fusion Based Register Allocation", createFusionRegisterAllocator);


//------------------------------------------------------------------

//Constructor
FusionRegAlloc::FusionRegAlloc() : MachineFunctionPass(ID) {
    //Interface needed
    initializeLiveIntervalsPass(*PassRegistry::getPassRegistry());
    initializeVirtRegMapPass(*PassRegistry::getPassRegistry());
    //initializeLiveRegMatrixPass(*PassRegistry::getPassRegistry());
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
    //AU.addRequired<LiveRegMatrix>();
    //AU.addPreserved<LiveRegMatrix>();
    
    MachineFunctionPass::getAnalysisUsage(AU);
}

//Interface functions - Some missing

void FusionRegAlloc::releaseMemory() { SpillerInstance.reset(); }

Spiller &FusionRegAlloc::spiller() { return *SpillerInstance; }

void FusionRegAlloc::enqueue(LiveInterval* li) { return; }

LiveInterval* FusionRegAlloc::dequeue() { return nullptr; }

unsigned FusionRegAlloc::selectOrSplit(LiveInterval &VirtReg, SmallVectorImpl<unsigned> &splitLRVs) { return 0U;}

void FusionRegAlloc::aboutToRemoveInterval(LiveInterval &LI) { return; }

//----------------------------------------------------------------- Region Graph

void FusionRegAlloc::printRegionGraph(raw_ostream& Out) {
    
    for (auto it = RGraph.begin(); it != RGraph.end(); it++) {
        RNode *node = *it;

        node->print(Out);
    }
}

void FusionRegAlloc::addRegion(MachineBasicBlock* MBB) {
    RNode *node = new RNode(MBB->getNumber());

    RGraph.push_back(node);
    MBBList.push_back(MBB);
}

RNode* FusionRegAlloc::getRegion(int Number) {

    for (auto it = RGraph.begin(); it != RGraph.end(); it++) {
        RNode *actual = *it;
        if (actual->getNumber() == Number) return actual; 
    }
}

void FusionRegAlloc::buildRegionGraph() {
    RGraph.clear();

    for (MachineFunction::iterator it = MF->begin(); it != MF->end(); it++) {
        MachineBasicBlock *MBB = &(*it);
        MBB->print(outs());

        addRegion(MBB);
    }

    linkSuccessors();
}

void FusionRegAlloc::linkSuccessors() {

    for (int i=0; i<MBBList.size(); i++) {
        MachineBasicBlock *MBB = MBBList[i];
        for (auto it = MBB->succ_begin(); it != MBB->succ_end(); it++) {
            MachineBasicBlock *SUC = *it;
            RNode *node = getRegion(SUC->getNumber());

            RGraph[i]->addNeighbor(node);
        }
    }
}

void FusionRegAlloc::sortEdges() {
    
}

//------------------------------------------------------------------------------

bool FusionRegAlloc::runOnMachineFunction(MachineFunction &mf) {
    //Initialize Variables
    MF = &mf;
    LIS = &getAnalysis<LiveIntervals>(); 
    VRM = &getAnalysis<VirtRegMap>();

    buildRegionGraph();
    printRegionGraph(outs());

    //Initialize Interface
    //RegAllocBase::init(getAnalysis<VirtRegMap>(),
    //                   getAnalysis<LiveIntervals>(),
    //                   getAnalysis<LiveRegMatrix>());
   
    //allocatePhysRegs(); //Interface that calls seedLiveRegs
    
    //Clean memory and return changed
    releaseMemory();
    return true;
}
