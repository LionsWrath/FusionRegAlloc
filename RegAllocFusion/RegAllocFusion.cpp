//===------- RegAllocFusion.cpp - Fusion Based Register Allocator * C++ -*-===//
//
//                  Put names and shit like that here
//
//===----------------------------------------------------------------------===//
//
// This file contains the Fusion Register Allocator.
//
//  TODO:
//      OK - Verify region formation with Basic Blocks in mind
//          OK - Get CFG ( Basic Blocks as Region )
//          OK - Order EDGES ( Priotity List )
//          OK - Create a Interference Graph for each region
//      - Fix the Interference Graph creation (Verify Regs)
//      OK - Check how many live ranges must be spilled within each region
//      - Graph Fusion
//          - Begin Fusing the graphs in the EDGE order
//          - Always maintain the simplificability
//          - Return an unique Interference Graph
//      - Transform our GI to LI 
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
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/LiveStackAnalysis.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineBranchProbabilityInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegAllocRegistry.h"
#include "llvm/CodeGen/RegisterClassInfo.h"
#include "llvm/CodeGen/VirtRegMap.h"

#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/BranchProbability.h"

#include "/home/lionswrath/llvm/lib/CodeGen/Spiller.h"
#include "/home/lionswrath/llvm/lib/CodeGen/RegAllocBase.h"
#include "/home/lionswrath/llvm/lib/CodeGen/LiveDebugVariables.h"

#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetRegisterInfo.h"
#include "llvm/Target/TargetSubtargetInfo.h"

#include <set>
#include <memory>
#include <queue>
#include <tuple>
#include <algorithm>

using namespace llvm;

#define DEBUG_TYPE "regalloc"

///-------------------------///
///      Golden Interval    ///
///-------------------------///

class GoldenInterval {
public:
    GoldenInterval(unsigned Reg): reg(Reg), spillStatus(0) {}
    
    struct GoldenSegment {
        SlotIndex start;
        SlotIndex end;

        GoldenSegment(SlotIndex s, SlotIndex e): start(s), end(e) {
            assert(s < e && "Cannot create empty or backwards golden segment!");
        }

        bool contains(SlotIndex I) const {
            return start <= I && I < end;
        }

        bool containsInterval(SlotIndex s, SlotIndex e) const {
            assert(s < e && "Backwards Interval");
            return (start <= s && s < end) && (start < e && e <= end);
        }

        bool operator<(const GoldenSegment &Other) const {
            return std::tie(start, end) < std::tie(Other.start, Other.end);
        }

        bool operator>(const GoldenSegment &Other) const {
            return std::tie(start, end) > std::tie(Other.start, Other.end);
        }

        bool operator==(const GoldenSegment &Other) const {
            return start == Other.start && end == Other.end;
        }
    };

    typedef std::vector<GoldenSegment> Segments;

    typedef Segments::iterator iterator;
    iterator begin() { return segments.begin(); }
    iterator end()   { return segments.end(); }

    typedef Segments::const_iterator const_iterator;
    const_iterator begin() const { return segments.begin(); }
    const_iterator end() const  { return segments.end(); }
 
    SlotIndex slot_begin() { return segments.front().start; }
    SlotIndex slot_end() { return segments.back().end; }

    SlotIndex slot_begin() const { return segments.front().start; }
    SlotIndex slot_end() const { return segments.back().end; }

    const unsigned reg;
    Segments segments;

    bool spillStatus;
    
    bool isSpilled() { return spillStatus; }

    bool spillLR() { return spillStatus = 1; }

    // Unite two GI segments
    void join(GoldenInterval *src) {
        for (auto it = src->begin(); it != src->end(); it++) {
            addSegment(*it);
        } 
    }

    bool isEmpty() {
        return segments.empty();
    }

    void addSegment(GoldenSegment GS) {
        segments.push_back(GS);
    }

    void addSegment(SlotIndex s, SlotIndex e) {
        segments.push_back(GoldenSegment(s,e)); 
    }

    bool overlapFrom(GoldenSegment &LGS, GoldenSegment &RGS) {
        if (LGS.start < RGS.start) {
            if (LGS.end > RGS.end) {
                return true;
            } else {
                if (LGS.end > RGS.start) {
                    return true;
                } else return false;
            }
        } else {
            if (LGS.end < RGS.end) {
                return true;
            } else {
                if (LGS.start < RGS.end) {
                    return true;
                } else return false;
            }
        }
    }

    bool overlaps(GoldenInterval Other) {
        if (Other.isEmpty() || isEmpty()) return false;

        for (auto it = begin(); it != end(); it++) {
            for (auto ti = Other.begin(); ti != Other.end(); ti++) {
                if (overlapFrom(*it, *ti)) return true; 
            }
        }

        return false;
    }

    void print(raw_ostream& Out) const {
        Out << "Golden Interval Segments: " << segments.size() << "\n"; // Check LiveInterval printing
    }

};

///-------------------------///
///    Interference Graph   ///
///-------------------------///

class INode { // Interference Node
    typedef std::set<INode*> NeighborList; // Change the data Structure

    GoldenInterval* GI;
    NeighborList Neighbors;

    INode();

public:
    INode(GoldenInterval* gi) : GI(gi) {}


    // Iterators
    typedef NeighborList::iterator iterator;
    typedef NeighborList::const_iterator const_iterator;

    iterator begin()                { return Neighbors.begin(); }
    iterator end()                  { return Neighbors.end(); }
    const_iterator begin() const    { return Neighbors.begin(); }
    const_iterator end() const      { return Neighbors.end(); }
   
    // Const function cannot change variables values
    bool checkSpill() const {
        if (GI->isSpilled()) return true;
        return false;
    }
    
    void markAsSpilled() {
        GI->spillLR();
    }

    // Check interference with another Node
    bool checkInterference(const INode* Node) const {
        return GI->overlaps(*Node->GI);
    }

    // Check interference with another Live Interval
    bool checkInterference(const GoldenInterval& G) const {
        return GI->overlaps(G);
    }

    std::vector<unsigned> getAllPhysRegs(RegisterClassInfo *RCI, MachineRegisterInfo *MRI) {
        std::vector<unsigned> allPhysRegs;

        ArrayRef<MCPhysReg> Order = RCI->getOrder(MRI->getRegClass(getColor()));

        //Add all physRegs that can be used by it
        for (auto it = Order.begin(); it != Order.end(); it++) {
            allPhysRegs.push_back(*it);
        }
        
        return allPhysRegs; 
    }

    // Verify if ths node is colorable
    bool isColorable(RegisterClassInfo *RCI, MachineRegisterInfo *MRI) {
        //For each virtual register
        
        std::vector<unsigned> allPhysRegs = getAllPhysRegs(RCI, MRI);
        
        if (allPhysRegs.size() < numNeighbors()) return false;
        return true;
    }

    // Getter of golden interval
    GoldenInterval* getGoldenInterval() { return GI; }
    const GoldenInterval* getGoldenInterval() const { return GI; }

    // Getter of register color associated
    // Register of the interval this node represents
    unsigned getColor() const {
        return GI->reg;
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

    // Merge two INode edges
    // Set automatically verify duplicates
    void merge(INode *src) {
        for (auto it = src->begin(); it != src->end(); it++) {
            addNeighbor(*it); 
        } 
    }

    // Print the interval related to this node
    void print(raw_ostream& Out) const {
        Out << "            Related Reg to Node :: " << GI->reg << "\n";
        for (auto it = Neighbors.begin(); it != Neighbors.end(); it++) {
            Out << "                Neighbor: ";
        }
    }

    void printInterval(raw_ostream& Out) const {
        Out << "[" << GI->slot_begin() << ", " << GI->slot_end() << ")";
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
    typedef std::vector<GoldenInterval*> GoldenIntervalList;

    int Number;

    InterferenceGraph IGraph;
    RegionNeighborList RNeighbors;
    GoldenIntervalList GIntervals;                  // Put all "LIs" here

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

    //Testing
    //Find all registers on the IGraph
    std::set<unsigned> getRelatedRegisters() {
        std::set<unsigned> relatedRegisters;

        for (auto it = IGraph.begin(); it != IGraph.end(); it++) {
            relatedRegisters.insert(it->getColor()); 
        }

        return relatedRegisters;
    }

    //Testing
    INode* getNode(unsigned reg) {
        for (auto it = IGraph.begin(); it != IGraph.end(); it++) {
            if (it->getColor() == reg)
               return &(*it); 
        }
    }

    // Maybe is a good idea use a enum to classify each type of golden segment
    // I am getting the right regs - question mark
    void constructRegionLI(LiveIntervals &LIS, const MachineRegisterInfo &MRI) {
        SlotIndex Start, Stop, Begin, End;
        std::tie(Start, Stop) = LIS.getSlotIndexes()->getMBBRange(Number);

        outs() << "[ " << Start << ", " << Stop << " ]" << '\n';
        for (unsigned i=0, e = MRI.getNumVirtRegs(); i != e; ++i) {
            unsigned Reg = TargetRegisterInfo::index2VirtReg(i);
            
            if (MRI.reg_nodbg_empty(Reg)) continue;

            const LiveInterval *LI = &LIS.getInterval(Reg);
            GoldenInterval *gi = new GoldenInterval(Reg);

            // Verify all Segments of LI
            for (auto it = LI->begin(); it != LI->end(); it++) {
                Begin = (*it).start;
                End = (*it).end;
                
                if (Begin >= Start && End <= Stop) {
                    //outs() << "All in: \n";
                    //outs() << "     ------------------------------------------" << '\n';
                    //outs() << "     Register: " << Reg << "\n     " << *LI << '\n';
                    //outs() << "     ------------------------------------------" << '\n';
                    
                    gi->addSegment(Begin, End);

                }

                if (Begin >= Start && End >= Stop && Begin <= Stop) {
                    //outs() << "Begin in block and Continue: \n";
                    //outs() << "     ------------------------------------------" << '\n';
                    //outs() << "     Register: " << Reg << "\n     " << *LI << '\n';
                    //outs() << "     ------------------------------------------" << '\n';

                    gi->addSegment(Begin, Stop);
                }

                if (Begin <= Start && End <= Stop && End >= Start) {
                    //outs() << "Come into the block and ends: \n";
                    //outs() << "     ------------------------------------------" << '\n';
                    //outs() << "     Register: " << Reg << "\n     " << *LI << '\n';
                    //outs() << "     ------------------------------------------" << '\n';
                    
                    gi->addSegment(Start, End);
                }

                if (Begin <= Start && End >= Stop) {
                    //outs() << "Live-Throught: \n";
                    //outs() << "     ------------------------------------------" << '\n';
                    //outs() << "     Register: " << Reg << "\n     " << *LI << '\n';
                    //outs() << "     ------------------------------------------" << '\n';

                    gi->addSegment(Start, Stop);
                }
            }
      
            // Probably not deleting object when not adding
            // Only add if is not empty
            if (!gi->isEmpty()) {
                GIntervals.push_back(gi);
            } 
        }
    }

    // Graph maintenance functions 


    // Testing
    void removeGoldenInterval(GoldenInterval *GI) {
        for (auto it = GIntervals.begin(); it != GIntervals.end(); it++) {
            if (GI->reg == (*it)->reg)
                GIntervals.erase(it);
        }
    }

    // Testing
    void addInterferenceNode(INode *node) {
        IGraph.push_back(*node);
        GIntervals.push_back(node->getGoldenInterval());
    }

    // Problem: IGraph uses non pointer on list
    // Change that lates
    void removeInterferenceNode(INode *node) {
        removeGoldenInterval(node->getGoldenInterval());

        for (auto it = IGraph.begin(); it != IGraph.end(); it++) {
            if (it->getGoldenInterval() == node->getGoldenInterval())
                IGraph.erase(it);
        }
    }

    // Add all INodes from another region IGraph
    void addAllInterferenceNodes(RNode *src) {
        InterferenceGraph *IG = src->getInterferenceGraph();

        for (auto it = IG->begin(); it != IG->end(); it++) {
            addInterferenceNode(&*it);
        } 
    }

    // Dont remove the src node
    void replaceEdge(INode *src, INode *dst) {
        for (auto it = IGraph.begin(); it != IGraph.end(); it++) {
            for (auto ti = it->begin(); ti != it->end(); it++) {
                if (*ti == src) {
                    it->removeNeighbor(*ti);
                    it->addNeighbor(dst);
                }
            }
        }
    }

    void buildInterferenceGraph(LiveIntervals &LIS, VirtRegMap &VRM) {
        IGraph.clear();

        // Add all nodes with virtual registers to graph
        for (auto it = GIntervals.begin(); it != GIntervals.end(); it++) {
            GoldenInterval *GI = *it;

            // Verify if this reg already has a Phys
            if (VRM.hasPhys(GI->reg)) continue;

            IGraph.push_back(INode(GI));
        }
        
        // Verify interferences between all nodes
        for (auto it = IGraph.begin(); it != IGraph.end(); it++) {
            for (auto ti = IGraph.begin(); ti != IGraph.end(); ti++) {
                if (it == ti) continue;
                if ((*it).checkInterference(&(*ti))) {
                    // Add two sides 
                    (*it).addNeighbor(&(*ti));
                    (*ti).addNeighbor(&(*it));
                } 
            }
        }
    }

    // Count the number of necessary spills in a interference graph
    // Testing
    int getNecessarySpills(RegisterClassInfo *RCI, MachineRegisterInfo *MRI) {
        std::vector<unsigned> allPhysRegs;
        int numberOfSpills = 0;

        //For each virtual register
        for (auto it = IGraph.begin(); it != IGraph.end(); it++) {
            if (!it->isColorable(RCI, MRI)) numberOfSpills++;
        }
       
        return numberOfSpills; 
    }
    
    // bool checkSpill(unsigned reg) {
        // //For each virtual register
        // for (auto it = IGraph.begin(); it != IGraph.end(); it++) {
            // if (it->getColor() == reg) 
                // return it->checkSpill();
        // }        
    // }

    // Print the interval related to this node
    void print(raw_ostream& Out) const {
        Out << "    RNode#" << Number << '\n';
        
        for (auto it = RNeighbors.begin(); it != RNeighbors.end(); it++) {
            Out << "        -> RNode#" << (*it)->getNumber() << '\n';
        }
        Out << "        Quantity of Intervals: " << GIntervals.size() << "\n";
        for (auto it = GIntervals.begin(); it != GIntervals.end(); it++) {
            GoldenInterval *gi = *it;
            Out << "            Reg: " << gi->reg <<" - [" << gi->slot_begin() << ", " << gi->slot_end() << ")\n";
        }
        Out << "        Interference Graph: " << IGraph.size() << "\n";
        for (auto it = IGraph.begin(); it != IGraph.end(); it++) {
            Out << "            INode :: ";
            (*it).printInterval(Out);
            Out << " -> ";
            for (auto ti = (*it).begin(); ti != (*it).end(); ti++) {
                (*(*ti)).printInterval(Out);
                Out << ", ";
            }
            Out << "\n";
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

    struct Span {
        INode *from;
        INode *to;

        Span(INode *f, INode *t): from(f), to(t) {}
    };

    typedef std::vector<Span> span_e;
    
    typedef std::vector<RNode*> RegionGraph;
    typedef std::vector<MachineBasicBlock*> MachineBasicBlockList;
    
    MachineFunction *MF;
    LiveIntervals *LIS;
    VirtRegMap *VRM;
    RegisterClassInfo RCI;

    MachineRegisterInfo *MRI; 
    const TargetRegisterInfo *TRI;

    std::unique_ptr<Spiller> SpillerInstance;
    
    RegionGraph RGraph;
    MachineBasicBlockList MBBList;

    // Priority Queue Implementation
    struct Edge {
        RNode *src, *dst;
        BranchProbability weight;

        Edge(RNode *s, RNode * d, BranchProbability w): src(s), dst(d), weight(w) {}

        bool operator<(const Edge &e) const {
            return weight < e.weight;
        }

        bool operator>(const Edge &e) const {
            return weight > e.weight;
        }

        RNode* getFrom() const {
            return src;
        }

        RNode* getTo() {
            return dst;
        }

        void print(raw_ostream &Out) const {
            Out << "RNode#" << src->getNumber() << " -> RNode#" << dst->getNumber();
            Out << "\n    Weight: " << weight << '\n';
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
        MachineBasicBlock* getMBB(int);
        void printRegionGraph(raw_ostream&);

        //Fusion part
        span_e findSpanning(RNode*, RNode*);    
        void fuzeRegions();
        bool isSplitable(Span);
        void coalesceLiveRanges(span_e, RNode*);
        void mergingCliques(RNode*);
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
    
    MachineFunctionPass::getAnalysisUsage(AU);
}

//Interface functions

void FusionRegAlloc::releaseMemory() { SpillerInstance.reset(); }

Spiller &FusionRegAlloc::spiller() { return *SpillerInstance; }

void FusionRegAlloc::enqueue(LiveInterval* li) { return; }

LiveInterval* FusionRegAlloc::dequeue() { return nullptr; }

unsigned FusionRegAlloc::selectOrSplit(LiveInterval &VirtReg, SmallVectorImpl<unsigned> &splitLRVs) { return 0U;}

void FusionRegAlloc::aboutToRemoveInterval(LiveInterval &LI) { return; }

//----------------------------------------------------------------- Region Graph

void FusionRegAlloc::printRegionGraph(raw_ostream& Out) {
    Out << "\nRegion Graph:\n";

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

    return nullptr;
}

MachineBasicBlock* FusionRegAlloc::getMBB(int Number) {
    for (auto it = MBBList.begin(); it != MBBList.end(); it++) {
        MachineBasicBlock *MBB = *it;
        if (MBB->getNumber() == Number) return MBB; 
    }

    return nullptr;
}

void FusionRegAlloc::buildRegionGraph() {
    RGraph.clear();

    for (MachineFunction::iterator it = MF->begin(); it != MF->end(); it++) {
        MachineBasicBlock *MBB = &(*it);

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
    MachineBranchProbabilityInfo MBPI;    
    
    for (int i=0; i<MBBList.size(); i++) {
        MachineBasicBlock *MBB = MBBList[i];
        for (auto it = MBB->succ_begin(); it != MBB->succ_end(); it++) {
            MachineBasicBlock *SUC = *it;

            BranchProbability BP = MBPI.getEdgeProbability(MBB, it);
            Edge *e = new Edge(RGraph[i], getRegion(SUC->getNumber()), BP);

            Queue.push(e);
        }
    }
}

//-------------------------------------------------------------------------FUSION

// When it was spilled the other nodes are already corrected
// Create a new Region too
void FusionRegAlloc::coalesceLiveRanges(span_e span, RNode *RN) {

    for (auto it = span.begin(); it != span.end(); it++) {
        if (isSplitable(*it)) {
            GoldenInterval *GI = new GoldenInterval(it->from->getColor());
            
            // Unioning Two Live ranges
            GI->join(it->from->getGoldenInterval());
            GI->join(it->to->getGoldenInterval());

            INode *IN = new INode(GI);

            // Merging Interference Edges
            IN->merge(it->from);
            IN->merge(it->to);  

            // Propagating Attributes
            IN->markAsSpilled();

            // Replacing wrong edges and removing the coalesced Nodes
            RN->replaceEdge(it->from, IN);
            RN->removeInterferenceNode(it->from);
            RN->replaceEdge(it->from, IN);
            RN->removeInterferenceNode(it->to);

            RN->addInterferenceNode(IN);
        } 
    }
}

bool FusionRegAlloc::isSplitable(Span s) {
    if (s.from->checkSpill() && s.to->checkSpill())
        return true;
    return false;
}

// Testing
// Create a span_e with all data spanning
FusionRegAlloc::span_e FusionRegAlloc::findSpanning(RNode *b1, RNode *b2) {
    FusionRegAlloc::span_e span;

    std::set<unsigned> related1 = b1->getRelatedRegisters();
    std::set<unsigned> related2 = b2->getRelatedRegisters();

    // Verificar se as variáveis continuam
    for (auto it = related1.begin(); it != related1.end(); it++) {
        if (related2.find(*it) != related2.end()) {
            Span temp = Span(b1->getNode(*it), b2->getNode(*it));
            span.push_back(temp);
        }
    }

    return span;
}

void FusionRegAlloc::fuzeRegions() {
    RNode *RN;
    
    while (Queue.empty()) {
        Edge *e = Queue.top();

        RN = new RNode(e->getFrom()->getNumber() + e->getTo()->getNumber());
        
        RN->addAllInterferenceNodes(e->getFrom());
        RN->addAllInterferenceNodes(e->getTo());

        span_e span = findSpanning(e->getFrom(), e->getTo());
        coalesceLiveRanges(span, RN);
        mergingCliques(RN);        

        Queue.pop();
    }
}

//------------------------------------------------------------------------------

bool FusionRegAlloc::runOnMachineFunction(MachineFunction &mf) {
    //Initialize Variables
    MF = &mf;
    LIS = &getAnalysis<LiveIntervals>(); 
    VRM = &getAnalysis<VirtRegMap>();

    MRI = &MF->getRegInfo();
    TRI = &VRM->getTargetRegInfo();

    MRI->freezeReservedRegs(*MF);
    RCI.runOnMachineFunction(*MF);      
    
    buildRegionGraph();
    sortEdges();

    // Create a function for this later
    for (auto it = RGraph.begin(); it != RGraph.end(); it++) {
        RNode *node = *it;
        
        node->constructRegionLI(*LIS, *MRI);
        node->buildInterferenceGraph(*LIS, *VRM);
    }

    for (auto it = RGraph.begin(); it != RGraph.end(); it++) {
        RNode *node = *it;
        
        outs() << "-> RNode#" << node->getNumber() << ": " << node->getNecessarySpills(&RCI, MRI) << "\n";
    }

    fuzeRegions();
    
    //For each virtual register
    //for (unsigned i=0, e = MRI->getNumVirtRegs(); i != e; ++i) {
        //unsigned Reg = TargetRegisterInfo::index2VirtReg(i);

        //if (MRI->reg_nodbg_empty(Reg)) continue;

        //MachineRegisterInfo::reg_iterator ri = MRI->reg_begin(Reg);
      
        // Extracting uses of each Reg 
        //for (auto it = MRI->reg_begin(Reg); it != MRI->reg_end(); it++) {
            //outs() << *(*it).getParent() << " "; 
        //}
        //outs() << "-----------------------\n\n";
         
        //ArrayRef<MCPhysReg> Order = RCI.getOrder(MRI->getRegClass(Reg));

        //Print all physRegs that can be used by it
        //for (auto it = Order.begin(); it != Order.end(); it++) {
            //outs() << TRI->getName((*it)) << " ";
            //if (MRI->isReserved((*it))) outs() << "R";
            //else outs() << "N";
            //if (MRI->isAllocatable((*it))) outs() << "A ";
            //else outs() << "N ";
        //}
        //outs() << "\n";
    //}

    // begin coloring of the interference graph
    // Maybe correct some link errors because of the physreg set

    //----------------------------------------------------------------Stopped here

    //Initialize Interface
    //RegAllocBase::init(getAnalysis<VirtRegMap>(),
    //                   getAnalysis<LiveIntervals>(),
    //                   getAnalysis<LiveRegMatrix>());
   
    //allocatePhysRegs(); //Interface that calls seedLiveRegs
    
    //Clean memory and return changed
    //releaseMemory();
    return true;
}
