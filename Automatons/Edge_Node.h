#ifndef EDGE_NODE_H
#define EDGE_NODE_H

#include <cstdint>
#include <vector>
#include <string>

// Edge class representing a transition between nodes in the automaton
// For a simple unweighted automaton, we only need the destination node ID.
// For a weighted automaton, we can also include a weight property.
class Edge {
private:
    // destination node ID for this edge
    uint16_t dstId;
    std::string label; //label for the edge
    std::vector<std::vector<uint16_t>> trueAPs; 
    //true atomic proposition ids for the edge. The outer vector represents 
    //the OR clauses and the inner vector represents the AND clauses
    // For weighted edges, we can add a weight property
    uint32_t weight;
public:
    // Constructors
    Edge() = default;
    Edge(uint16_t dstId) : dstId(dstId), weight(1) {}
    Edge(uint16_t dstId, uint32_t weight) : dstId(dstId), weight(weight) {}
    Edge(uint16_t dstId, const std::string& label) : dstId(dstId), label(label), weight(1) {}
    Edge(uint16_t dstId, const std::string& label, uint32_t weight = 1) : dstId(dstId), label(label), weight(weight) {}
    Edge(uint16_t dstId, const std::string& label, bool settrueAPs, uint32_t weight = 1) 
        : dstId(dstId), label(label), weight(weight) {
        if (settrueAPs) {
            this->trueAPs = parseEdgeLabelToVector(label);
        }
    }
    //constructor for subclasses
    virtual ~Edge() = default;
    
    //getters and setters
    uint16_t getDstId() const { return dstId; };
    uint32_t getWeight() const { return weight; };
    std::string getLabel() const { return label; };
    std::vector<std::vector<uint16_t>> getTrueAPs() const { return trueAPs; };
    void setTrueAPs() {
        trueAPs = parseEdgeLabelToVector(label);
    };
    
    std::vector<std::vector<uint16_t>> parseEdgeLabelToVector(const std::string& label) const {
        // Returns outer vector = OR clauses, inner vector = AND clauses with AP IDs
        std::vector<std::vector<uint16_t>> result;
        
        // Split by OR (|)
        size_t orStart = 0;
        size_t orEnd = label.find('|');
        
        while (orStart < label.length()) {
            std::string orClause = (orEnd == std::string::npos) ? 
                                label.substr(orStart) : 
                                label.substr(orStart, orEnd - orStart);
            
            std::vector<uint16_t> andClauseAPs;
            
            // Split by AND (&)
            size_t andStart = 0;
            size_t andEnd = orClause.find('&');
            
            while (andStart < orClause.length()) {
                std::string token = (andEnd == std::string::npos) ? 
                                orClause.substr(andStart) : 
                                orClause.substr(andStart, andEnd - andStart);
                
                // Trim whitespace and quotes
                token.erase(0, token.find_first_not_of(" \t\n\r\""));
                token.erase(token.find_last_not_of(" \t\n\r\"") + 1);
                
                // Skip if empty or negated
                if (!token.empty() && token[0] != '!') {
                    try {
                        // Extract number from "p0", "p1", etc.
                        if (token[0] == 'p' && token.length() > 1) {
                            std::string numStr = token.substr(1);
                            uint16_t apId = static_cast<uint16_t>(std::stoul(numStr));
                            andClauseAPs.push_back(apId);
                        }
                    } catch (const std::exception& e) {
                        // Silent catch
                    }
                }
                
                if (andEnd == std::string::npos) break;
                andStart = andEnd + 1;
                andEnd = orClause.find('&', andStart);
            }
            
            if (!andClauseAPs.empty()) {
                result.push_back(andClauseAPs);
            }
            
            if (orEnd == std::string::npos) break;
            orStart = orEnd + 1;
            orEnd = label.find('|', orStart);
        }
        
        return result;
    }

    
        void setDstId(uint16_t id) { dstId = id; };
        void setWeight(uint32_t w) { weight = w; };
        void setLabel(const std::string& label) { this->label = label; };
    };

    // Node class representing a state in the automaton
    class Node{
    private:
        // For a simple unweighted automaton, we can just store the node ID.
        // For a more complex automaton, we can also include additional properties (e.g., labels, accepting state flag).
        uint16_t id;
        std::string label; //label for the node
        std::pair<uint16_t, std::vector<uint16_t>> productStates; //buchistate, [robot1state, robot2state, ...]
        uint32_t numEdges; //total number of outgoing edges for the node
        std::vector<Edge> edges; // outgoing edges from this node
    public:
        // Constructors
        Node() = default;
        Node(uint16_t id) : id(id), label("Room" + std::to_string(id)), numEdges(0) {}
        Node(uint16_t id, const std::string& label) : id(id), label(label), numEdges(0) {}
        Node(uint16_t id, const std::string& label, bool isProduct): id(id), label(label), numEdges(0) {
            if (isProduct) {
                setProductStates(label);
            }
        }
        //constructor for subclasses
        virtual ~Node() = default;
        //getters and setters
        uint16_t getId() const { return id; };
        void setId(uint16_t id) { this->id = id; };
        std::string getLabel() const { return label; };
        void setLabel(const std::string& label) { this->label = label; };
        uint16_t getidfromlabel() const { return static_cast<uint16_t>(std::stoul(label.substr(4))); }; //convert label to id if label is numeric
        std::vector<Edge> getEdges() const { return edges; };
        uint32_t getNumEdges() const { return numEdges; };
        //get the product states for this node
        std::pair<uint16_t, std::vector<uint16_t>> getProductStates() const { return productStates; };
        void setProductStates(const std::string& label) {
            // Label format is "buchistate,robot1state,robot2state,robot3state,..."
            std::vector<uint16_t> states;
            size_t pos = 0;
            size_t next_pos = label.find(',', pos);
            
            if (next_pos != std::string::npos) {
                // Extract buchistate
                uint16_t buchistate = static_cast<uint16_t>(std::stoul(label.substr(pos, next_pos - pos)));
                pos = next_pos + 1;
                
                // Extract all robot states
                while ((next_pos = label.find(',', pos)) != std::string::npos) {
                    states.push_back(static_cast<uint16_t>(std::stoul(label.substr(pos, next_pos - pos))));
                    pos = next_pos + 1;
                }
                // Get last robot state
                if (pos < label.length()) {
                    states.push_back(static_cast<uint16_t>(std::stoul(label.substr(pos))));
                }
                
                productStates = std::make_pair(buchistate, states);
            }
        }
        //add an edge to this node
        void addEdge(const Edge& edge) { edges.push_back(edge);
            numEdges++;
        };
        //check if there is a direct edge to a given destination node
        bool isAdjacent(uint16_t dstId) const {
            for (const Edge& edge : edges) {
                if (edge.getDstId() == dstId) {
                    return true;
                }
            }
            return false;
        };
        //node label can be used for more complex automata where states have labels
    virtual std::string to_String() { 
        std::string result = "Node[id=" + std::to_string(this->id) + ", label=" + this->label + ", edges=(";
        for (size_t i = 0; i < edges.size(); ++i) {
            result += std::to_string(edges[i].getDstId());
            if (i < edges.size() - 1) result += "|";
        }
        result += ")]";
        return result;
    };
    std::vector<Edge> getEdgestoNode(uint16_t dstId) const { 
        std::vector<Edge> result;
        for (const Edge& edge : edges) {
            if (edge.getDstId() == dstId) {
                result.push_back(edge);
            }
        }
        return result;
    }; //get all edges to a specific destination node ID

};

#endif // EDGE_NODE_H