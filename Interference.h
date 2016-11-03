#ifndef INTERFERENCE_H
#define INTERFERENCE_H

//Includes Missing

using namespace llvm;

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

#endif
