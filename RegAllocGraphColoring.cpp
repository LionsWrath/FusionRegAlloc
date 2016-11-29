//===-- RegAllocGraphColoring.cpp - Priority-Based Coloring Reg Allocator -===//
//
//                     The LLVM Compiler Infrastructure
//
// This file was developed by Bill Wendling and is distributed under the
// University of Illinois Open Source License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements a priority-based coloring approach to register
// allocation. This is based upon the paper:
//
//   Chow, F., and Hennessy, J. The Priority-Based Coloring Approach to Register
//   Allocation. In ACM Transactions on Programming Languages and Systems, Vol.
//   12, No. 4, October 1990.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "regalloc"

#include "PhysRegTracker.h"
#include "VirtRegMap.h"
#include "llvm/Function.h"
#include "llvm/ADT/BitMatrix.h"
#include "llvm/ADT/EquivalenceClasses.h"
#include "llvm/ADT/HashExtras.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/CodeGen/LiveIntervalAnalysis.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/RegAllocRegistry.h"
#include "llvm/CodeGen/SSARegMap.h"
#include "llvm/Target/MRegisterInfo.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Debug.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <memory>

using namespace llvm;

namespace {

  static RegisterRegAlloc
    graphColoringRegAlloc("graphcoloring",
                          "  graph coloring register allocator",
                          createGraphColoringRegisterAllocator);

  ///===------------------------------------------------------------------===///
  ///                             INode Class
  ///===------------------------------------------------------------------===///
  ///
  /// Interference graph node--that pretty much sums it up. If two live
  /// intervals interfere with each other, then they are "neighbors" (what
  /// a nice term!).

  /// FIXME: This class seems to have redundancies (there's a map from
  /// LiveInterval to these things). This should probably be reworked.
  class VISIBILITY_HIDDEN INode {
    typedef std::set<INode*> NeighborList;

    LiveInterval* LI;
    NeighborList  Neighbors;

    INode();                    // DO NOT IMPLEMENT
  public:
    INode(LiveInterval* li) : LI(li) {}

    typedef NeighborList::iterator       iterator;
    typedef NeighborList::const_iterator const_iterator;

    iterator begin()             { return Neighbors.begin(); }
    iterator end()               { return Neighbors.end(); }
    const_iterator begin() const { return Neighbors.begin(); }
    const_iterator end() const   { return Neighbors.end(); }

    /// interferes - Returns "true" if the nodes interfere (overlap).
    bool interferes(const INode* Node) const {
      return LI->overlaps(*Node->LI);
    }
    bool interferes(const LiveInterval& L) const {
      return LI->overlaps(L);
    }

    /// getLiveInterval - Get the live interval associated with this node.
    LiveInterval* getLiveInterval() { return LI; }
    const LiveInterval* getLiveInterval() const { return LI; }

    /// getColor - Return the "color" of this node. It's equivalent to the
    /// register of the interval this node represents.
    unsigned getColor() const {
      return LI->reg;
    }

    /// addNeighbor - Add a node to the set of nodes that interfere with
    /// this node.
    void addNeighbor(INode* Node) {
      Neighbors.insert(Node);
    }

    /// removeNeighbor - Remove a node from the set of nodes that interfere with
    /// this node.
    void removeNeighbor(INode* Node) {
      Neighbors.erase(Node);
    }

    /// hasNeighbors - Returns true if there are nodes which interfere with
    /// this node.
    bool hasNeighbors() const {
      return !Neighbors.empty();
    }

    /// numNeighbors - Returns the number of neighbors this node has in the
    /// graph.
    unsigned numNeighbors() const {
      return unsigned(Neighbors.size());
    }

    /// print - Print the LiveInterval associated with this node.
    void print(std::ostream& Out) const {
      Out << "Node :: " << *LI;
    }
  };

  std::ostream& operator << (std::ostream& Out, const INode& N) {
    N.print(Out);
    return Out;
  }

  ///===------------------------------------------------------------------===///
  ///                          RegAlloc Class
  ///===------------------------------------------------------------------===///

  class VISIBILITY_HIDDEN RegAlloc : public MachineFunctionPass {
    /// RelatedRegClasses - This structure is built the first time a function is
    /// compiled, and keeps track of which register classes have registers that
    /// belong to multiple classes or have aliases that are in other classes.
    EquivalenceClasses<const TargetRegisterClass*> RelatedRegClasses;
    std::map<unsigned, const TargetRegisterClass*> OneClassForEachPhysReg;

    typedef std::list<INode> InterferenceGraph;

    InterferenceGraph    IGraph;
    hash_map<const LiveInterval*, INode*> LINodeMap;

    MachineFunction*        MF;
    const TargetMachine*    TM;
    const MRegisterInfo*    MRI;
    LiveIntervals*          LIs;
    const LoopInfo*         LoopInformation;
    bool*                   PhysRegsUsed;
    std::set<LiveInterval*> UncolorableLIs;

    hash_map<const LiveInterval*, std::set<unsigned> > ForbiddenRegs;

    std::auto_ptr<PhysRegTracker> PRT;
    std::auto_ptr<VirtRegMap>     VRM;
    std::auto_ptr<Spiller>        Spillifier;
  public:
    virtual const char* getPassName() const {
      return "Priority-Based Coloring Approach to Register Allocation";
    }

    /// runOnMachineFunction - Register allocate the whole function.
    bool runOnMachineFunction(MachineFunction& Fn);

    virtual void getAnalysisUsage(AnalysisUsage& AU) const {
      AU.addRequired<LiveIntervals>();
      AU.addRequired<LoopInfo>();
      MachineFunctionPass::getAnalysisUsage(AU);
    }
  private:
    bool PriorityBasedColoring();

    /// BuildGraph - Build the register interference graph.
    void BuildGraph();

    void RemoveNodeFromGraph(INode* Node);

    /// GetFreeReg - Return a free preg for this live interval.
    unsigned GetFreeReg(const LiveInterval* LI);

    /// SeparateUnconstrainedLiveIntervals - Separate the live intervals into
    /// unconstrained (there are physical registers that don't interfere) or
    /// constrained (there are no physical registers available that don't
    /// interfere).
    void SeparateUnconstrainedLiveIntervals(std::set<LiveInterval*>& ULIs,
                                            std::set<LiveInterval*>& CLIs);

    /// CalculateInterference - For the given LiveInterval, calculate all of the
    /// other LiveIntervals that interfere with it.
    template <typename InputIter>
      void CalculateInterference(const LiveInterval* LI,
                                 InputIter Begin, InputIter End);

    /// RegAliasesPhysReg - Return true if the Reg aliases the physical register
    /// assigned to the virtual register.
    bool RegAliasesPhysReg(unsigned Reg, unsigned VReg) const;

    /// ComputeRelatedRegClasses - Create union sets of related register
    /// classes.
    void ComputeRelatedRegClasses();

    /// InitPhysRegsUsed - Mark those physical registers we know are used in the
    /// PhysRegsUsed array.
    void InitPhysRegsUsed() const;

    /// SplitLiveInterval - Split the live interval into intervals in the hopes
    /// that we can eventually allocate a physical register to it.
    void SplitLiveInterval(LiveInterval* LI, std::set<LiveInterval*>& ULIs,
                           std::set<LiveInterval*>& CLIs);

    /// SpillLiveInterval - Assign a live interval to a stack slot.
    void SpillLiveInterval(LiveInterval* LI);

    /// GetNumMappableRegs - Return the number of registers that a virtual
    /// register can be mapped to in the system.
    unsigned GetNumMappableRegs(unsigned VReg);

    /// GetForbiddenRegs - Get the list of registers that are forbidden for
    /// (interfere with) the given live range. Return "true" if the forbidden
    /// set is smaller than the number of registers available for a live range
    /// in the live interval.
    bool GetForbiddenRegs(LiveInterval* LI, LiveRange const *& LR,
                          LiveInterval::const_iterator& Iter,
                          std::set<unsigned>& Forbidden,
                          std::set<const LiveInterval*>& Neighbors);

    /// GetLIWithHighestPriority - Returns the interval with the highest
    /// priority that hasn't been assigned a physical register.
    LiveInterval* GetLIWithHighestPriority(const std::set<LiveInterval*>& CLIs);

    unsigned CalculatePriority(const LiveInterval* LI) const;

    unsigned GetLoopDepth(const MachineBasicBlock* MBB) const {
      return LoopInformation->getLoopDepth(MBB->getBasicBlock());
    }

    bool IsUncolorable(LiveInterval* LI);

    bool IsConstrained(const LiveInterval* LI);

    /// debug - Print out the interference graph if debugging.
    void debug(std::ostream& Out) {
      // For Debugging...
      for (InterferenceGraph::const_iterator
             I = IGraph.begin(), E = IGraph.end(); I != E; ++I) {
        const INode* Node = &*I;

        Out << "Interval " << *Node << "\n";

        if (Node->hasNeighbors()) {
          Out << "  Neighbors:\n";

          for (INode::const_iterator NI = Node->begin(), NE = Node->end();
               NI != NE; ++NI)
            Out << "    " << **NI << "\n";
        } else {
          Out << "    No Neighbors\n";
        }

        Out << "\n  ForbiddenRegs: ";

        const LiveInterval* LI = Node->getLiveInterval();
        unsigned Count = 0;

        if (ForbiddenRegs[LI].size())
          for (std::set<unsigned>::const_iterator
                 II = ForbiddenRegs[LI].begin(), IE = ForbiddenRegs[LI].end();
               II != IE; ++Count) {
            Out << MRI->getName(*II);

            if (++II != IE) {
              Out << ", ";

              if (Count == 10) {
                Out << "\n                 ";
                Count = 0;
              }
            }
          }
        else
          Out << "None";

        Out << "\n";
      }
    }
  };

} // end anonymous namespace


///===--------------------------------------------------------------------===///
/// Graph manipulation methods
///===--------------------------------------------------------------------===///


/// BuildGraph - Build the register interference graph.
///
void RegAlloc::BuildGraph() {
  IGraph.clear();

  std::vector<const LiveInterval*> LIVector;

  /// Fill the interference graph with the nodes: one per live interval.
  for (LiveIntervals::iterator I = LIs->begin(), E = LIs->end(); I != E; ++I) {
    LiveInterval& LI = I->second;

    if (!MRegisterInfo::isVirtualRegister(LI.reg) || VRM->hasPhys(LI.reg))
      continue;

    LIVector.push_back(&LI);
    IGraph.push_back(INode(&LI));
    LINodeMap[&LI] = &IGraph.back();
  }

  /// Go through the graph nodes. If the nodes interfere, then indicate it by
  /// making them neighbors.
  for (std::vector<const LiveInterval*>::iterator
         I = LIVector.begin(), E = LIVector.end(); I != E; ++I)
    CalculateInterference(*I, LIVector.begin(), LIVector.end());

  DEBUG(debug(std::cerr));
}

/// CalculateInterference - For the given LiveInterval, calculate all of the
/// other LiveIntervals that interfere with it.
/// 
template <typename InputIter>
void RegAlloc::CalculateInterference(const LiveInterval* LI, InputIter Begin,
                                     InputIter End) {
  INode* CurNode = LINodeMap[LI];
  unsigned CurReg = CurNode->getColor();

  const TargetRegisterClass* RC = MF->getSSARegMap()->getRegClass(CurReg);
  const TargetRegisterClass* RCLeader = RelatedRegClasses.getLeaderValue(RC);

  for (InputIter I = Begin; I != End; ++I) {
    const LiveInterval* Neighbor = *I;

    if (!MRegisterInfo::isVirtualRegister(Neighbor->reg) || Neighbor == LI)
      continue;

    INode* NeighborNode = LINodeMap[Neighbor];
    const TargetRegisterClass* RegRC =
      MF->getSSARegMap()->getRegClass(NeighborNode->getColor());

    /// Test if this live range interferes with the current live range and that
    /// they're both in the same register class.
    if (CurNode->interferes(NeighborNode) &&
        RelatedRegClasses.getLeaderValue(RegRC) == RCLeader)
      CurNode->addNeighbor(NeighborNode);
    else
      CurNode->removeNeighbor(NeighborNode);
  }

  /// Calculate which physical registers conflict with this live interval and
  /// which don't.
  for (LiveIntervals::iterator I = LIs->begin(); I != LIs->end(); ++I) {
    LiveInterval& LI = I->second;
    unsigned PReg = 0;

    if (!MRegisterInfo::isVirtualRegister(LI.reg))
      PReg = I->second.reg;
    else if (VRM->hasPhys(LI.reg))
      PReg = VRM->getPhys(LI.reg);
    else
      continue;

    if (CurNode->interferes(LI))
      ForbiddenRegs[CurNode->getLiveInterval()].insert(PReg);
    else 
      ForbiddenRegs[CurNode->getLiveInterval()].erase(PReg);
  }
}

/// RemoveNodeFromGraph - Remove a node from the interference graph. Make sure
/// that it's no longer listed as a "neighbor" of its former neighbors.
/// 
void RegAlloc::RemoveNodeFromGraph(INode* Node) {
  for (InterferenceGraph::iterator
         I = IGraph.begin(), E = IGraph.end(); I != E; ++I) {
    const INode* N = &*I;

    if (Node == N) {
      if (Node->hasNeighbors())
        for (INode::iterator
               NI = Node->begin(), NE = Node->end(); NI != NE; ++NI)
          (*NI)->removeNeighbor(Node);

      IGraph.erase(I);
      return;
    }
  }
}


///===--------------------------------------------------------------------===///
/// Register information methods
///===--------------------------------------------------------------------===///


/// InitPhysRegsUsed - Mark those physical registers we know are used in the
/// PhysRegsUsed array.
///
void RegAlloc::InitPhysRegsUsed() const {
  for (LiveIntervals::const_iterator I = LIs->begin(), E = LIs->end();
       I != E; ++I)
    if (MRegisterInfo::isPhysicalRegister(I->second.reg))
      MF->changePhyRegUsed(I->second.reg, true);
}

/// GetNumMappableRegs - Return the number of physical registers available to
/// map to a virtual register.
/// 
unsigned RegAlloc::GetNumMappableRegs(unsigned VReg) {
  assert(MRegisterInfo::isVirtualRegister(VReg));
  const TargetRegisterClass* RC = MF->getSSARegMap()->getRegClass(VReg);
  TargetRegisterClass::const_iterator I = RC->allocation_order_begin(*MF);
  TargetRegisterClass::const_iterator E = RC->allocation_order_end(*MF);
  unsigned Count = 0;
  for (; I != E; ++I) ++Count;
  return Count;
}

/// RegAliasesPhysReg - Return true if the Reg aliases the physical register
/// assigned to the virtual register.
///
bool RegAlloc::RegAliasesPhysReg(unsigned Reg, unsigned VReg) const {
  assert(MRegisterInfo::isVirtualRegister(VReg));

  for (const unsigned* AS = MRI->getAliasSet(VRM->getPhys(VReg)); *AS; ++AS)
    if (*AS == Reg)
      return true;

  return false;
}

/// ComputeRelatedRegClasses - Create union sets of related register classes.
///
void RegAlloc::ComputeRelatedRegClasses() {
  // First pass, add all reg classes to the union, and determine at least one
  // reg class that each register is in.
  bool HasAliases = false;

  for (MRegisterInfo::regclass_iterator RCI = MRI->regclass_begin(),
       E = MRI->regclass_end(); RCI != E; ++RCI) {
    RelatedRegClasses.insert(*RCI);

    for (TargetRegisterClass::const_iterator
           I = (*RCI)->begin(), E = (*RCI)->end(); I != E; ++I) {
      HasAliases |= *MRI->getAliasSet(*I) != 0;

      const TargetRegisterClass *&PRC = OneClassForEachPhysReg[*I];

      if (PRC)
        /// Already processed this register. Just make sure we know that
        /// multiple register classes share a register.
        RelatedRegClasses.unionSets(PRC, *RCI);
      else
        PRC = *RCI;
    }
  }

  /// Second pass, now that we know conservatively what register classes each
  /// reg belongs to, add info about aliases. We don't need to do this for
  /// targets without register aliases.
  if (HasAliases)
    for (std::map<unsigned, const TargetRegisterClass*>::const_iterator
         I = OneClassForEachPhysReg.begin(), E = OneClassForEachPhysReg.end();
         I != E; ++I)
      for (const unsigned *AS = MRI->getAliasSet(I->first); *AS; ++AS)
        RelatedRegClasses.unionSets(I->second, OneClassForEachPhysReg[*AS]);
}

/// GetFreeReg - Return a free preg for this live interval.
///
unsigned RegAlloc::GetFreeReg(const LiveInterval* LI) {
  unsigned Reg = 0; 

  /// Scan for the first available register
  const TargetRegisterClass* RC = MF->getSSARegMap()->getRegClass(LI->reg);

  TargetRegisterClass::const_iterator I = RC->allocation_order_begin(*MF);
  TargetRegisterClass::const_iterator E = RC->allocation_order_end(*MF);

  for (; I != E; ++I) {
    unsigned PReg = *I;

    if (ForbiddenRegs[LI].count(PReg)) continue; // Register conflicts

    const INode* Node = LINodeMap[LI];

    for (INode::const_iterator
           NI = Node->begin(), NE = Node->end(); NI != NE; ++NI) {
      unsigned Color = (*NI)->getColor();

      /// Check to make sure that the aliases of a physical register aren't
      /// conflicting here
      if (MRegisterInfo::isVirtualRegister(Color) && VRM->hasPhys(Color)) {
        if (VRM->getPhys(Color) == PReg || RegAliasesPhysReg(PReg, Color))
          goto TryAgain;
      } else if (Color == PReg) {
        goto TryAgain;
      }
    }

    Reg = PReg;
    break;
  TryAgain:;                    // Gotos aren't always evil...
  }

  DEBUG({
    if (Reg)
      std::cerr << "Assigning VReg " << LI->reg
                << " to PReg " << MRI->getName(Reg) << "\n";
    else
      std::cerr << "Couldn't find PReg for VReg " << LI->reg << "\n";
  });

  return Reg;
}

/// GetForbiddenRegs - Get the list of registers that are forbidden for
/// (interfere with) the given live range. Return "true" if the forbidden set is
/// smaller than the number of registers available for a live range in the live
/// interval.
///
bool RegAlloc::GetForbiddenRegs(LiveInterval* LI, LiveRange const *& LR,
                                LiveInterval::const_iterator& Iter,
                                std::set<unsigned>& Forbidden,
                                std::set<const LiveInterval*>& Neighbors) {
  for (; Iter != LI->end(); ++Iter) {
    LR = &*Iter;

    Forbidden.clear();
    Neighbors.clear();

    std::set<unsigned>::const_iterator I = ForbiddenRegs[LI].begin();
    std::set<unsigned>::const_iterator E = ForbiddenRegs[LI].end();

    for (; I != E; ++I) {
      if (!MRegisterInfo::isVirtualRegister(*I)) continue;

      LiveInterval& L = LIs->getInterval(*I);

      for (LiveInterval::const_iterator
             J = L.begin(); J != L.end(); ++J)
        if (LR->start < J->end && LR->end > J->start) {
          if (!MRegisterInfo::isVirtualRegister(L.reg))
            Forbidden.insert(L.reg);

          Neighbors.insert(&L);
          break;
        }
    }

    if (Forbidden.size() < GetNumMappableRegs(LI->reg))
      return true;
  }

  return false;
}


///===--------------------------------------------------------------------===///
/// LiveInterval manipulation methods
///===--------------------------------------------------------------------===///


bool RegAlloc::IsUncolorable(LiveInterval* LI) {
  LiveInterval::const_iterator Iter = LI->begin();
  std::set<unsigned> Forbidden;
  std::set<const LiveInterval*> Neighbors;
  const LiveRange* LR = &*Iter;

  return !GetForbiddenRegs(LI, LR, Iter, Forbidden, Neighbors) ||
    ForbiddenRegs[LI].size() >= GetNumMappableRegs(LI->reg);
}

bool RegAlloc::IsConstrained(const LiveInterval* LI) {
  const INode* Node = LINodeMap[LI];
  return int(Node->numNeighbors()) >=
    int(GetNumMappableRegs(LI->reg) - ForbiddenRegs[LI].size());
}

/// SeparateUnconstrainedLiveIntervals - Separate the live intervals into
/// unconstrained (there are physical registers that don't interfere) or
/// constrained (there are no physical registers available that don't
/// interfere).
/// 
void
RegAlloc::SeparateUnconstrainedLiveIntervals(std::set<LiveInterval*>& ULIs,
                                             std::set<LiveInterval*>& CLIs) {
  for (LiveIntervals::iterator I = LIs->begin(), E = LIs->end(); I != E; ++I) {
    LiveInterval& LI = I->second;

    if (!MRegisterInfo::isVirtualRegister(LI.reg) || VRM->hasPhys(LI.reg))
      continue;

    if (IsConstrained(&LI)) {
      CLIs.insert(&LI);
      ULIs.erase(&LI);
      DEBUG(std::cerr << "Adding Constrained: " << LI << "\n");
    } else {
      ULIs.insert(&LI);
      CLIs.erase(&LI);
      DEBUG(std::cerr << "Adding Unconstrained: " << LI << "\n");
    }
  }
}

/// SplitLiveInterval - Split the live interval into intervals in the hopes that
/// we can eventually allocate a physical register to it.
/// 
void RegAlloc::SplitLiveInterval(LiveInterval* LI,
                                 std::set<LiveInterval*>& ULIs,
                                 std::set<LiveInterval*>& CLIs) {
  DEBUG(std::cerr << "Splitting live interval: " << *LI << "\n");

  /// In comments below:
  ///
  ///     lr        -- original live interval
  ///     lr'       -- new live interval
  ///     live unit -- equivalent to an llvm LiveRange

  /// (1)
  /// Find a live unit in lr in which the first appearance is a definition,
  /// preferably one at an entry point to lr.  If this cannot be found, then
  /// start with the first live unit that contains a use.  The basic block for
  /// this first live unit of lr' must have unused registers.  This guarantees
  /// that the new live range lr' will be colorable when formed.  Initialize the
  /// "forbidden" set of lr' to be the set of used registers in the lone basic
  /// block.

  /// NewLI       - The live ranges which make up the new live interval we're
  ///               hoping to create.
  /// LINeighbors - The neighbors which interfere with the live ranges we're
  ///               inserting into the new live interval.
  /// LIForbidden - Physical register which are used by neighbors of this live
  ///               range.
  std::vector<LiveRange>        NewLI;
  std::set<const LiveInterval*> LINeighbors;
  std::set<unsigned>            LIForbidden;

  /// We'd *like* to start off at the definition for this live interval. If
  /// that's not possible (InstNum is ~0), then we just go with the first
  /// live range in the live interval.
  unsigned InstNum = LI->getInstForValNum(LI->begin()->ValId);

  if (!InstNum || InstNum == ~0U || InstNum == ~1U)
    InstNum = LI->beginNumber();

  const LiveRange* LR = LI->getLiveRangeContaining(InstNum);
  assert(LR && "Couldn't find live range!!");

  LiveInterval::const_iterator Iter = LI->begin();

  if (!GetForbiddenRegs(LI, LR, Iter, LIForbidden, LINeighbors)) {
    /// This entire live interval is constrained.
    DEBUG(std::cerr << "  Live Interval is constrained. Cannot split!\n");
    ULIs.erase(LI);
    CLIs.insert(LI);
    return;
  }

  assert(LR && "Should have a live range by now!");
  NewLI.push_back(*LR);
  LINeighbors.insert(LI);

  /// (2)
  /// For each successor of the live units in lr' that belongs to lr, check
  /// whether it can be added to lr'.  A live unit can be added if it does not
  /// cause the forbidden set of lr' to be full.  On each addition of a live
  /// unit, this "forbidden" set is updated by computing its union with the set
  /// of used registers in the new basic block.

  std::set<const LiveInterval*> LRNeighbors;

  ++Iter;

  while (GetForbiddenRegs(LI, LR, Iter, LIForbidden, LRNeighbors)) {
    std::set<unsigned> LRForbidden;

    std::set<unsigned> Union;
    std::insert_iterator<std::set<unsigned> > UnionIns(Union, Union.begin());

    std::set_union(LIForbidden.begin(), LIForbidden.end(),
                   LRForbidden.begin(), LRForbidden.end(),
                   UnionIns);

    if (Union.size() < GetNumMappableRegs(LI->reg)) {
      NewLI.push_back(*LR);
      LIForbidden.swap(Union);
      LINeighbors.insert(LRNeighbors.begin(), LRNeighbors.end());
    }

    ++Iter;
  }

  /// If we've decided that all ranges in the original live interval are
  /// unconstrained, then we don't need to split.
  ///
  /// FIXME: This only works if each individual LiveRange is treated as a single
  /// unit and not as something that can be split.
  if (unsigned(LI->end() - LI->begin()) == NewLI.size()) return;

  for (std::vector<LiveRange>::const_iterator
         I = NewLI.begin(), E = NewLI.end(); I != E; ++I) {
    DEBUG(std::cerr << "  Removing live range " << *I
                    << " from old interval\n");
    LI->removeRange(*I);
  }

  DEBUG(std::cerr << "Old live interval (after removal): " << *LI << "\n");

  LiveInterval& NewInterval = LIs->CreateNewLiveInterval(LI, NewLI);

  IGraph.push_back(INode(&NewInterval));
  LINodeMap[&NewInterval] = &IGraph.back();
  LINeighbors.insert(&NewInterval);

  /// (3)
  /// Now that the final shape of lr' (and thus that of lr) has been determined,
  /// update the interferences.  Apart from lr and lr', the live ranges whose
  /// interferences need to be updated are exactly those live ranges that
  /// interfere with the original lr.  After the split, these live ranges may
  /// interfere with lr, lr', or both, depending on their spans.

  for (std::set<const LiveInterval*>::const_iterator
         I = LINeighbors.begin(); I != LINeighbors.end(); ++I)
    CalculateInterference(*I, LINeighbors.begin(), LINeighbors.end());

  /// (4)
  /// Update other information in the live ranges lr' and lr.  For example, the
  /// flags "needrlod" and "needrstr" in the live unit representation are no
  /// longer current, because they are dependent on the actual live range
  /// boundary.

  /// The LLVM spiller will take care of making sure that the values are passed
  /// between the two split live intervals via the same stack slot. It also is
  /// smart and notices when a live range can (actually a BB) can use a register
  /// without needing to be spilt.
  int StackSlot = VRM->hasStackSlot(LI->reg) ?
    VRM->getStackSlot(LI->reg) : VRM->assignVirt2StackSlot(LI->reg);
  VRM->assignVirt2StackSlot(NewInterval.reg, StackSlot);

  /// (5)
  /// If lr, lr', or both become unconstrained after the split, remove them from
  /// the constrained pool and add them to the unconstrained pool of live
  /// ranges.  Because of the introduction of new live ranges, it is also
  /// possible that some unconstrained live range is made constrained.  This
  /// must arise out of the live ranges that interfere with both lr and lr'
  /// after the split, because they are the only ones whose number of
  /// intereferences increase due to the split.  Thus, only these live ranges
  /// need be checked, and the constrained pool and unconstrained pool are
  /// updated.

  for (std::set<const LiveInterval*>::const_iterator
         I = LINeighbors.begin(); I != LINeighbors.end(); ++I) {
    LiveInterval* L = const_cast<LiveInterval*>(*I);

    if (IsConstrained(L)) {
      ULIs.erase(L);
      CLIs.insert(L);
      DEBUG(std::cerr << "Adding Constrained: " << L << "\n");
    } else {
      ULIs.insert(L);
      CLIs.erase(L);
      DEBUG(std::cerr << "Adding Unconstrained: " << L << "\n");
    }
  }
}

/// SpillLiveInterval - Assign a live interval to a stack slot.
/// 
void RegAlloc::SpillLiveInterval(LiveInterval* LI) {
  /// FIXME: Be smarter about finding a preg to spill to. One that's not used so
  /// much during this interval, perhaps?

  if (!MRegisterInfo::isVirtualRegister(LI->reg) || VRM->hasStackSlot(LI->reg))
    return;

  int Slot = VRM->assignVirt2StackSlot(LI->reg);
  DEBUG(std::cerr << "Spilling " << *LI << " into slot " << Slot << "\n");
  std::vector<LiveInterval*> Added =
    LIs->addIntervalsForSpills(*LI, *VRM, Slot);

  static unsigned J = 0;

  for (unsigned I = 0; I < Added.size(); ++I, ++J) {
    unsigned VReg = Added[I]->reg;
    const TargetRegisterClass* RC = MF->getSSARegMap()->getRegClass(VReg);
    TargetRegisterClass::const_iterator Iter = RC->allocation_order_begin(*MF);
    if (Iter + J >= RC->allocation_order_end(*MF)) J = 0;
    unsigned PReg = *(Iter + J);

    VRM->assignVirt2Phys(VReg, PReg);
    PRT->addRegUse(PReg);

    DEBUG(std::cerr << "\tAdded LI: " << *Added[I]
                    << " assigned Reg " << MRI->getName(PReg)
                    << "\n");

    for (LiveIntervals::const_iterator
           K = LIs->begin(); K != LIs->end(); ++K) {
      const LiveInterval& L = K->second;
      unsigned LIReg = L.reg;

      if (!MRegisterInfo::isVirtualRegister(LIReg) || LIReg == VReg) continue;

      if (VRM->hasPhys(LIReg) && VRM->getPhys(LIReg) == PReg &&
          !VRM->hasStackSlot(LIReg) && Added[I]->overlaps(L))
        VRM->assignVirt2StackSlot(LIReg);
    }
  }
}

unsigned RegAlloc::CalculatePriority(const LiveInterval* LI) const {
  /// FIXME: This could be more sophisticated. E.g., it could take into account
  /// how many load/stores are performed in the live interval. The paper
  /// suggests this as the formula:
  ///
  ///    s_i = LoadSave * u + StoreSave * d - MoveCost * n
  ///
  /// where u is the number of uses, d is the number of definitions, and n is
  /// the number of register moves.
  ///
  ///    S(li) = \Sum{s_i * w_i}
  ///
  /// where w_i is execution frequency estimates in the individual live ranges.
  /// Therefore, the priority is:
  ///
  ///    P(li) = S(li) / N
  ///
  /// where N is the number of live ranges within the live interval.

  unsigned InstNum = LI->getInstForValNum(LI->begin()->ValId);

  if (!InstNum || InstNum == ~0U || InstNum == ~1U)
    InstNum = LI->beginNumber();

  const MachineInstr* MI = LIs->getInstructionFromIndex(InstNum);
  return MI ? GetLoopDepth(MI->getParent()) + 1 : 1;
}

/// GetLIWithHighestPriority - Returns the interval with the highest priority
/// that hasn't been assigned a physical register.
/// 
LiveInterval*
RegAlloc::GetLIWithHighestPriority(const std::set<LiveInterval*>& CLIs) {
  LiveInterval* Highest = 0;
  unsigned MaxPriority = 0;

  for (std::set<LiveInterval*>::const_iterator
         I = CLIs.begin(), E = CLIs.end(); I != E; ++I) {
    LiveInterval* LI = *I;

    if (!MRegisterInfo::isVirtualRegister(LI->reg)) continue;

    unsigned Priority = CalculatePriority(LI);

    if (Priority > MaxPriority) {
      Highest = LI;
      MaxPriority = Priority;
    }
  }

  assert(Highest && "Cannot find an interval with the highest priority!");
  return Highest;
}


///===--------------------------------------------------------------------===///
/// Coloring method
///===--------------------------------------------------------------------===///


/// PriorityBasedColoring - Return "true" if we've assigned registers. False
/// otherwise.
/// 
bool RegAlloc::PriorityBasedColoring() {
  std::set<LiveInterval*> ConstrainedLIs;
  std::set<LiveInterval*> UnconstrainedLIs;

  bool AssignedRegs = false;

  /// Initialize the priority of each LI to something ridiculous
  hash_map<const LiveInterval*, unsigned> LIPriority;

  for (std::set<LiveInterval*>::iterator
         I = ConstrainedLIs.begin(), E = ConstrainedLIs.end(); I != E; ++I)
    LIPriority[*I] = ~0U;

  /// 1)
  /// Separate the unconstrained live ranges.
  SeparateUnconstrainedLiveIntervals(UnconstrainedLIs, ConstrainedLIs);

  /// 2)
  /// Repeat steps (a) to (c), each time assigning a color to a live range until
  /// all constrained live ranges have been assigned a color or until there is
  /// no register left that can be assigned to any live range in any basic
  /// block.

  while (!ConstrainedLIs.empty()) {
    /// (a)
    /// Compute the priority function P(lr) for each constrained live range lr
    /// if it has not been computed.  If P(lr) < 0 or if lr is uncolorable,
    /// which occurs when all available registers have been used throughout its
    /// region, mark lr as a noncandidate, and leave it unassigned.  Delete lr
    /// from the interference graph so that live ranges interfering with it will
    /// have one less interference.
    /// 
    /// FIXME: Instead of P(lr) < 0, we should use a low threshold, like 1?

    for (std::set<LiveInterval*>::iterator I = ConstrainedLIs.begin(),
           E = ConstrainedLIs.end(); I != E; ++I) {
      LiveInterval* CLI = *I;

      if (LIPriority[CLI] == ~0U)
        LIPriority[CLI] = CalculatePriority(CLI);

      /// Now we check to see if each live range in the live interval is
      /// "uncolorable", i.e. all pregs are used during its span.
      ///
      /// FIXME: This algorithm is pretty gross and will probably result in
      /// massive performance issues!

      if (IsUncolorable(CLI)) {
        RemoveNodeFromGraph(LINodeMap[CLI]);
        UncolorableLIs.insert(CLI);
        DEBUG(std::cerr << "Removing uncolorable LiveInterval: "
                        << *CLI << "\n");
      }
    }

    if (!UncolorableLIs.empty())
      for (std::set<LiveInterval*>::iterator
             I = UncolorableLIs.begin(), E = UncolorableLIs.end(); I != E; ++I)
        if (*I) ConstrainedLIs.erase(*I);

    if (ConstrainedLIs.empty()) break;

    /// (b)
    /// Find the live range with the highest priority function, lr*, and assign
    /// a color to it. The color assigned must not be in the "forbidden" set for
    /// lr*.  For each live range that lr* interferes with, update the
    /// "forbidden" information.

    LiveInterval* Highest = GetLIWithHighestPriority(ConstrainedLIs);
    unsigned PReg = GetFreeReg(Highest);

    assert(PReg && "Couldn't find a physical register for this LI!");

    VRM->assignVirt2Phys(Highest->reg, PReg);
    PRT->addRegUse(PReg);
    AssignedRegs = true;
    ConstrainedLIs.erase(Highest);

    const INode* Node = LINodeMap[Highest];
    std::vector<INode*> DeadNodes;

    for (INode::iterator
           NI = Node->begin(), NE = Node->end(); NI != NE; ++NI) {
      LiveInterval* L = (*NI)->getLiveInterval();

      ForbiddenRegs[L].insert(PReg);

      if (!VRM->hasPhys(L->reg) && IsConstrained(L)) {
        UnconstrainedLIs.erase(L);

        if (IsUncolorable(L)) {
          DeadNodes.push_back(LINodeMap[L]);
          UncolorableLIs.insert(L);
          ConstrainedLIs.erase(L);
          DEBUG(std::cerr << "+ Removing uncolorable LiveInterval: "
                          << *L << "\n");
        } else {
          ConstrainedLIs.insert(L);
          DEBUG(std::cerr << "+ Adding Constrained: " << *L << "\n");
        }
      }
    }

    /// FIXME: This might make some nodes unconstrained!
    for (std::vector<INode*>::iterator
           NI = DeadNodes.begin(), NE = DeadNodes.end(); NI != NE; ++NI)
      RemoveNodeFromGraph(*NI);

    /// (c)
    /// Check each live range interfering with lr* to see if it needs to be
    /// split.  A live range needs to be split if all of its target registers
    /// have been assigned to one of its neighbors in the interference graph.
    /// This fact is conveyed by its "forbidden" set being equal to the set of
    /// available registers.  If splitting is necessary, apply the splitting
    /// algorithm.

    for (INode::iterator
           NI = Node->begin(), NE = Node->end(); NI != NE; ++NI) {
      LiveInterval* NLI = (*NI)->getLiveInterval();

      if (ForbiddenRegs[NLI].size() >= GetNumMappableRegs(NLI->reg))
        SplitLiveInterval(NLI, UnconstrainedLIs, ConstrainedLIs);
    }
  }

  /// 3)
  /// Assign colors to the unconstrained live ranges, each time choosing a color
  /// not belonging to their "forbidden" set.

  for (std::set<LiveInterval*>::const_iterator I = UnconstrainedLIs.begin(),
         E = UnconstrainedLIs.end(); I != E; ++I) {
    const LiveInterval* LI = *I;
    unsigned PReg = GetFreeReg(LI);

    assert(PReg && "Couldn't find a physical register for this LI!");

    VRM->assignVirt2Phys(LI->reg, PReg);
    PRT->addRegUse(PReg);

    /// Need to update the forbidden information for all neighbor nodes.
    const INode* Node = LINodeMap[const_cast<LiveInterval*>(LI)];

    for (INode::iterator
           NI = Node->begin(), NE = Node->end(); NI != NE; ++NI)
      ForbiddenRegs[(*NI)->getLiveInterval()].insert(PReg);

    AssignedRegs = true;
  }

  return AssignedRegs;
}


///===--------------------------------------------------------------------===///
/// Main method
///===--------------------------------------------------------------------===///


bool RegAlloc::runOnMachineFunction(MachineFunction& Fn) {
  /// Priority-Based Coloring Approach to Register Allocation
  DEBUG(std::cerr << "******** PRIORITY-BASED REGISTER ALLOCATION ********\n");
  DEBUG(std::cerr << "******** Function: "
                  << Fn.getFunction()->getName() << '\n');

  MF = &Fn;
  TM = &MF->getTarget();
  MRI = TM->getRegisterInfo();
  LIs = &getAnalysis<LiveIntervals>();
  LoopInformation = &getAnalysis<LoopInfo>();

  /// If this is the first function compiled, compute the related reg classes.
  if (RelatedRegClasses.empty())
    ComputeRelatedRegClasses();

  PhysRegsUsed = new bool[MRI->getNumRegs()];
  std::fill(PhysRegsUsed, PhysRegsUsed + MRI->getNumRegs(), false);
  Fn.setUsedPhysRegs(PhysRegsUsed);
  InitPhysRegsUsed();

  if (!PRT.get()) PRT.reset(new PhysRegTracker(*MRI));
  VRM.reset(new VirtRegMap(*MF));
  if (!Spillifier.get()) Spillifier.reset(createSpiller());

  bool NotFinished = true;

  while (NotFinished) {
    BuildGraph();
    if (IGraph.empty()) break;
    NotFinished = PriorityBasedColoring();
  }

  DEBUG({
    std::cerr << "=== Register Mapping ===\n";

    for (LiveIntervals::const_iterator
           I = LIs->begin(), E = LIs->end(); I != E; ++I) {
      const LiveInterval& LI = I->second;
      if (!MRegisterInfo::isVirtualRegister(LI.reg)) continue;

      std::cerr << "  reg " << LI.reg << " -> ";

      if (VRM->hasPhys(LI.reg))
        std::cerr << MRI->getName(VRM->getPhys(LI.reg)) << "\n";
      else
        std::cerr << "unassigned!!!\n";
    }
  });

  /// Spill the uncolorable live intervals.
  for (std::set<LiveInterval*>::iterator I = UncolorableLIs.begin(),
         E = UncolorableLIs.end(); I != E; ++I) {
    DEBUG(std::cerr << "Uncolorable LI: " << **I << "\n");
    (*I)->weight = 0.000927;    // Set weight to bogus value
    SpillLiveInterval(*I);
  }

  /// Rewrite spill code and update the PhysRegsUsed set.
  Spillifier->runOnMachineFunction(*MF, *VRM);

  VRM.reset();                  /// Free the VirtRegMap
  IGraph.clear();
  LINodeMap.clear();
  UncolorableLIs.clear();
  ForbiddenRegs.clear();
  return true;
}

FunctionPass* llvm::createGraphColoringRegisterAllocator() {
  return new RegAlloc();
}

// [EOF] RegAllocGraphColoring.cpp
