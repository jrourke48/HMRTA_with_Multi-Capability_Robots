#include "LTLFormula/LTLFormula.h"
#include <spot/tl/parse.hh>
#include <sstream>

// Constructor
LTLFormula::LTLFormula(std::string formulaStr, const std::vector<BatchAtomicProposition>& batchAPsVec) 
    : batchAPs(nullptr) {
    try {
        spot::parsed_formula pf = spot::parse_infix_psl(formulaStr);
        formula = pf.f;
        batchAPs = new std::vector<BatchAtomicProposition>(batchAPsVec);
    } catch (const std::exception& e) {
        delete batchAPs;
        batchAPs = nullptr;
        throw std::runtime_error("Failed to parse LTL formula: " + std::string(e.what()));
    }
}

// Destructor
LTLFormula::~LTLFormula() {
    delete batchAPs;
    batchAPs = nullptr;
}

// Parse the LTL formula
void LTLFormula::parse(const std::string& formulaStr) {
    try {
        spot::parsed_formula pf = spot::parse_infix_psl(formulaStr);
        formula = pf.f;
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to parse LTL formula: " + std::string(e.what()));
    }
}

// Convert to string representation
std::string LTLFormula::toString() const {
    std::ostringstream oss;
    oss << formula;
    return oss.str();
}

// Evaluate the formula
bool LTLFormula::evaluate() {
    // TODO: Implement evaluation logic
    return false;
}

// Setter for formula
void LTLFormula::setFormula(const std::string& formulaStr) {
    try {
        spot::parsed_formula pf = spot::parse_infix_psl(formulaStr);
        formula = pf.f;
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to parse LTL formula: " + std::string(e.what()));
    }
}

// Getter for formula string
std::string LTLFormula::getFormula() const {
    std::ostringstream oss;
    oss << formula;
    return oss.str();
}

// Get the Spot formula object
spot::formula LTLFormula::getSpotFormula() const {
    return formula;
}

// Get all batch atomic propositions
const std::vector<BatchAtomicProposition>& LTLFormula::getBatchAPs() const {
    if (!batchAPs) throw std::runtime_error("BatchAPs vector is null");
    return *batchAPs;
}

// Get a specific batch atomic proposition by ID
BatchAtomicProposition LTLFormula::getBatchAP(uint16_t id) const {
    if (!batchAPs) throw std::runtime_error("BatchAPs vector is null");
    for (const auto& batchAP : *batchAPs) {
        if (batchAP.getAPId() == id) {
            return batchAP;
        }
    }
    throw std::out_of_range("Atomic proposition ID not found");
}

// Get the TS state (AP) for a specific AP ID
uint16_t LTLFormula::getTSState(uint16_t id) const {
    if (!batchAPs || batchAPs->empty()) {
        throw std::runtime_error("BatchAPs vector is null or empty");
    }
    return getBatchAP(id).getAP();
}

// Add a batch atomic proposition
void LTLFormula::addBatchAtomicProposition(const BatchAtomicProposition& ap) {
    if (!batchAPs) batchAPs = new std::vector<BatchAtomicProposition>();
    batchAPs->push_back(ap);
}

// Get batch atomic propositions vector
std::vector<BatchAtomicProposition>& LTLFormula::getBatchAtomicPropositions() {
    if (!batchAPs) batchAPs = new std::vector<BatchAtomicProposition>();
    return *batchAPs;
}

// Get batch atomic propositions vector (const version)
const std::vector<BatchAtomicProposition>& LTLFormula::getBatchAtomicPropositions() const {
    if (!batchAPs) throw std::runtime_error("BatchAPs vector is null");
    return *batchAPs;
}

// Get atomic propositions (alias for getBatchAtomicPropositions)
std::vector<BatchAtomicProposition>& LTLFormula::getAtomicPropositions() {
    if (!batchAPs) batchAPs = new std::vector<BatchAtomicProposition>();
    return *batchAPs;
}

// Get the batch value for a specific AP ID
int8_t LTLFormula::getBatchVal(uint16_t apId) const {
    if (!batchAPs) throw std::runtime_error("BatchAPs vector is null");
    for (const auto& batchAP : *batchAPs) {
        if (batchAP.getAPId() == apId) {
            return batchAP.getBatch();
        }
    }
    throw std::out_of_range("AP ID not found");
}

// Get the required capabilities for a specific AP ID
std::vector<bool> LTLFormula::getRequiredCapabilities(uint16_t apId) const {
    if (!batchAPs) throw std::runtime_error("BatchAPs vector is null");
    for (const auto& batchAP : *batchAPs) {
        if (batchAP.getAPId() == apId) {
            return batchAP.getCapabilities();
        }
    }
    throw std::out_of_range("AP ID not found");
}
// Get the total required capabilities
uint16_t LTLFormula::getTotalRequiredCapabilities() const {
    if (!batchAPs) throw std::runtime_error("BatchAPs vector is null");
    uint16_t total = 0;
    for (const auto& batchAP : *batchAPs) {
        for (bool cap : batchAP.getCapabilities()) {
            if (cap) {
                total += 1;
            }
        }
    }
    return total;
}

// Get the independent required capabilities
uint16_t LTLFormula::getIndependentRequiredCapabilities() const {
    if (!batchAPs) throw std::runtime_error("BatchAPs vector is null");
    
    // Create a zero vector of capabilities
    std::vector<bool> capabilities;
    
    // Build it from independent batch atomic propositions
    for (const auto& batchAP : *batchAPs) {
        if (capabilities.empty()) {
            // Initialize with the first batchAP's capabilities
            capabilities = batchAP.getCapabilities();
        } else {
            // OR the capabilities together
            const auto& batchCaps = batchAP.getCapabilities();
            for (size_t i = 0; i < batchCaps.size() && i < capabilities.size(); ++i) {
                capabilities[i] = capabilities[i] || batchCaps[i];
            }
        }
    }
    
    // Count the total true values
    uint16_t total = 0;
    for (bool cap : capabilities) {
        if (cap) {
            total += 1;
        }
    }
    return total;
}

// Check if formula is valid
bool LTLFormula::isValid() const {
    // TODO: Implement validation logic
    return true;
}

// Build the formula tree
void LTLFormula::buildTree() {
    // TODO: Implement tree building logic
}

