#ifndef BUCHI_AUTOMATON_H
#define BUCHI_AUTOMATON_H

#include "Automaton.h"
#include <cstdint>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <memory>
#include <algorithm>
#include <fstream>
#include <cstdlib>
#include <map>
#include <set>
#include <spot/twa/twagraph.hh>
#include <spot/twaalgos/dot.hh>
#include <spot/tl/parse.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/degen.hh>
#include <spot/tl/formula.hh>
#include "../MCTB-PDT/include/LTLFormula/LTLFormula.h"
#include "../MCTB-PDT/include/LTLFormula/BatchAtomicProposition.h"

class BuchiAutomaton : public Automaton {
private:
    std::vector<uint16_t> acceptingStates;  // Set of accepting states (Büchi accepting states)
    LTLFormula* ltlFormula;  // LTL formula associated with the Büchi automaton (externally owned)
    spot::twa_graph_ptr spotAutomaton;  // Store the Spot automaton for visualization
    uint16_t initialState;  // Initial state of the Büchi automaton
    bool isInfiniteFlag;  // Flag indicating if the automaton is infinite (GBA)


public:
    // Constructor that takes a pointer to an LTL formula object (externally owned)
    BuchiAutomaton(LTLFormula* formula) : ltlFormula(formula), isInfiniteFlag(false) {
        try {
            // Extract spot formula from LTLFormula and build automaton
            spot::translator trans;
            spot::twa_graph_ptr buchiAut = trans.run(ltlFormula->getSpotFormula());
            
            // Check if it's a GBA (Generalized Büchi Automaton) BEFORE degeneralizing
            std::ostringstream dotStream;
            spot::print_dot(dotStream, buchiAut);
            std::string dotContent = dotStream.str();
            checkIsInfinite(dotContent);  // Check if the automaton is infinite (GBA)
            
            // Degeneralize: convert GBA to standard Büchi automaton
            buchiAut = spot::degeneralize(buchiAut);
            
            // Store the degeneralized spot automaton for visualization
            this->spotAutomaton = buchiAut;
            
            // Clear existing data
            nodeMap.clear();
            acceptingStates.clear();
            numNodes = 0;
            numEdges = 0;
            
            // Generate DOT from degeneralized automaton and parse it
            std::ostringstream dotStream2;
            spot::print_dot(dotStream2, buchiAut);
            std::string dotContent2 = dotStream2.str();
            parseBuchiFromDot(dotContent2);

        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to create Buchi automaton from formula: " + std::string(e.what()));
        }
    }

    ~BuchiAutomaton() override;
    // Override pure virtual methods from Automaton
    void add_Node(Node* node) override;
    bool isAdjacent(uint16_t srcId, uint16_t dstId) const override;

    // Büchi-specific methods
    // Getters for initial and accepting states
    void setInitial(uint16_t stateId) {
        initialState = stateId;
    };
    uint16_t getInitialState() const {
        return initialState;
    };
    // Get the list of accepting states
    void setAccepting(uint16_t stateId) {
        // Add to accepting states if not already present
        auto it = std::find(acceptingStates.begin(), acceptingStates.end(), stateId);
        if (it == acceptingStates.end()) {
            acceptingStates.push_back(stateId);
        }
    };
    // Check if the buchi automaton is finite
    bool isFinite() const {
        return !isInfiniteFlag;
    }
    // Check if the buchi automaton is infinite (GBA)
    bool isInfinite() const {
        return isInfiniteFlag;
    }
    // Check if a state is an accepting state
    bool isAcceptingState(uint16_t stateId) const {
        return std::find(acceptingStates.begin(), acceptingStates.end(), stateId) != acceptingStates.end();
    };
    //get the edge label for a given source and destination
    std::vector<std::string> getEdgeLabels(uint16_t srcId, uint16_t dstId) const;

    //get the true atomic propositions for a given source and destination
    std::vector<std::vector<uint16_t>> getTrueAPs(uint16_t srcId, uint16_t dstId) const;
    // Check if the DOT content indicates an infinite automaton (GBA) by looking for Inf() label
    void checkIsInfinite(const std::string& dotContent) {
        // Look for label="Inf(...) in the DOT header
        size_t label_pos = dotContent.find("label=\"");
        if (label_pos != std::string::npos) {
            size_t end_pos = dotContent.find('\"', label_pos + 7);
            if (end_pos != std::string::npos) {
                std::string label_content = dotContent.substr(label_pos + 7, end_pos - (label_pos + 7));
                // Check if label contains "Inf(" which indicates generalized Büchi automaton
                isInfiniteFlag = (label_content.find("Inf(") != std::string::npos);
            }
        } else {
            isInfiniteFlag = false;
        }
    }
        
    // Comprehensive DOT parser that builds the Büchi automaton
    void parseBuchiFromDot(const std::string& dotContent) {
        std::istringstream stream(dotContent);
        std::string line;
        initialState = 0;  // Default to state 0
        std::map<unsigned, std::vector<std::pair<unsigned, std::string>>> edges;  // src -> [(dst, label)]
        std::set<unsigned> seenNodes;
        std::set<unsigned> acceptingNodeIds;  // Track nodes with peripheries=2
        
        while (std::getline(stream, line)) {
            // Trim line
            line.erase(0, line.find_first_not_of(" \t"));
            line.erase(line.find_last_not_of(" \t") + 1);
            
            if (line.empty() || line[0] == '}' || line[0] == '#') continue;
            
            // Check for initial state indicator: I -> <state> or I-><state>
            if (line.find("I") == 0 && line.find("->") != std::string::npos) {
                size_t arrow_pos = line.find("->");
                std::string stateStr = line.substr(arrow_pos + 2);
                stateStr.erase(0, stateStr.find_first_not_of(" \t"));  // Trim leading whitespace
                stateStr.erase(stateStr.find_last_not_of(" \t\n") + 1);  // Trim trailing whitespace
                
                std::istringstream iss(stateStr);
                if (iss >> initialState) {  // Successfully parsed
                    // initialState is set
                }
                continue;
            }
            
            // Check for node definition: <id> [label="..." peripheries=2 ...]
            size_t bracket_start = line.find('[');
            if (bracket_start != std::string::npos && line.find("->") == std::string::npos) {
                unsigned nodeId;
                std::istringstream iss(line);
                if (iss >> nodeId) {
                    // Check if this is the invisible initial node
                    if (nodeId != UINT_MAX) {  // Skip if parsing failed
                        seenNodes.insert(nodeId);
                        
                        // Check for accepting state marker: peripheries=2 (standard DOT format)
                        if (line.find("peripheries=2") != std::string::npos) {
                            acceptingNodeIds.insert(nodeId);
                        }
                    }
                }
                continue;
            }
            
            // Check for edge definition: <src> -> <dst> [label="..."]
            size_t arrow_pos = line.find("->");
            if (arrow_pos != std::string::npos) {
                // Extract source
                unsigned src;
                std::istringstream src_stream(line.substr(0, arrow_pos));
                if (!(src_stream >> src)) continue;
                
                // Extract destination and bracket content
                size_t after_arrow = arrow_pos + 2;
                size_t bracket_start = line.find('[', after_arrow);
                if (bracket_start == std::string::npos) continue;
                
                std::string dst_str = line.substr(after_arrow, bracket_start - after_arrow);
                dst_str.erase(0, dst_str.find_first_not_of(" \t"));
                dst_str.erase(dst_str.find_last_not_of(" \t") + 1);
                
                unsigned dst;
                if (!(std::istringstream(dst_str) >> dst)) continue;
                
                // Skip initial edge (I -> state)
                if (src == UINT_MAX) continue;
                
                seenNodes.insert(src);
                seenNodes.insert(dst);
                
                // Extract label and acceptance info from brackets
                size_t bracket_end = line.find(']', bracket_start);
                if (bracket_end == std::string::npos) bracket_end = line.length();
                
                std::string bracket_content = line.substr(bracket_start + 1, bracket_end - bracket_start - 1);
                std::string edgeLabel = extractLabelFromDotBrackets(bracket_content);
                
                edges[src].push_back(std::make_pair(dst, edgeLabel));
            }
        }
        
        // Create nodes based on discovered state IDs
        for (unsigned nodeId : seenNodes) {
            Node* node = new Node(nodeId);
            add_Node(node);
        }
        
        // Mark nodes as accepting if they have peripheries=2 (standard DOT format)
        for (unsigned nodeId : acceptingNodeIds) {
            setAccepting(nodeId);
        }
        
        // Add edges
        for (const auto& srcEntry : edges) {
            Node* srcNode = getNode(srcEntry.first);
            if (srcNode != nullptr) {
                for (const auto& dstLabelPair : srcEntry.second) {
                    unsigned dst = dstLabelPair.first;
                    std::string label = dstLabelPair.second;
                    // Create edge with label and add to source node
                    Edge e(dst, label, true, 1);
                    srcNode->addEdge(e);
                    numEdges++;
                }
            }
        }
    }

    // Extract label from DOT bracket content, handling escaped quotes and newlines
    std::string extractLabelFromDotBrackets(const std::string& content) const {
        size_t label_pos = content.find("label=");
        if (label_pos == std::string::npos) return "true";
        
        label_pos += 6;  // strlen("label=")
        
        // Skip whitespace and opening quote
        while (label_pos < content.length() && (content[label_pos] == ' ' || content[label_pos] == '"')) {
            label_pos++;
        }
        
        std::string label;
        while (label_pos < content.length()) {
            char c = content[label_pos];
            
            if (c == '\\' && label_pos + 1 < content.length()) {
                char next = content[label_pos + 1];
                if (next == '"') {
                    label += '"';
                    label_pos += 2;
                } else if (next == 'n') {
                    // Add : then grab acceptance marks {x, y, ...}
                    label += ':';
                    label_pos += 2;  // Skip \n
                    
                    // Skip whitespace after newline
                    while (label_pos < content.length() && (content[label_pos] == ' ' || content[label_pos] == '\t')) {
                        label_pos++;
                    }
                    
                    // Grab acceptance marks: {0,1,2} etc
                    while (label_pos < content.length()) {
                        char c = content[label_pos];
                        if (c == '"') {
                            break;  // End of label
                        }
                        label += c;
                        label_pos++;
                    }
                    break;
                } else {
                    label += c;
                    label_pos++;
                }
            } else if (c == '"') {
                break;
            } else {
                label += c;
                label_pos++;
            }
        }
        
        // Clean up label - remove trailing whitespace
        label.erase(label.find_last_not_of(" \t") + 1);
        return label.empty() ? "true" : label;
    }

    bool isAccepting(uint16_t stateId) const {
        auto it = std::find(acceptingStates.begin(), acceptingStates.end(), stateId);
        return it != acceptingStates.end();
    };
    const std::vector<uint16_t>& getAcceptingStates() const {
        return acceptingStates;
    };

    // Method to get the Spot formula object
    spot::formula get_ltl_formula() const {
        if (!ltlFormula) return spot::formula();
        return ltlFormula->getSpotFormula();
    };

    // Method to get the LTLFormula object
    const LTLFormula* getLTLFormula() const {
        return ltlFormula;
    };
    //method to get a mutable pointer to the LTLFormula object
    LTLFormula* getLTLFormula() {
        return ltlFormula;
    }
    // Method to get the Spot automaton object
    spot::twa_graph_ptr getSpotAutomaton() const {
        return spotAutomaton;
    };

    // Visualization methods
    void saveToDot(const std::string& filename) const {
        if (!spotAutomaton) {
            throw std::runtime_error("Spot automaton not available for visualization");
        }
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }
        spot::print_dot(file, spotAutomaton);
        file.close();
    }

    void convertDotToPng(const std::string& dotFile, const std::string& pngFile) const {
        std::string command = "dot -Tpng \"" + dotFile + "\" -o \"" + pngFile + "\"";
        int result = system(command.c_str());
        if (result != 0) {
            throw std::runtime_error("Failed to convert DOT to PNG. Ensure graphviz is installed.");
        }
    }

    void visualize(const std::string& outputPrefix) const {
        std::string dotFile = outputPrefix + ".dot";
        std::string pngFile = outputPrefix + ".png";
        saveToDot(dotFile);
        convertDotToPng(dotFile, pngFile);
    }

};

#endif
