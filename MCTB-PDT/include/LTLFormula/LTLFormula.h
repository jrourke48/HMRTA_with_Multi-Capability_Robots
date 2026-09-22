#ifndef LTL_FORMULA_H
#define LTL_FORMULA_H

#include "BatchAtomicProposition.h"
#include <string>
#include <memory>
#include <vector>
#include <spot/twa/twagraph.hh>
#include <spot/twaalgos/dot.hh>
#include <spot/tl/parse.hh>
#include <spot/twaalgos/translate.hh>

class LTLFormula {
    private:
    spot::formula formula;
    std::vector<BatchAtomicProposition>* batchAPs; // List of atomic propositions in the formula

    // Helper methods
    bool isValid() const;
    void buildTree();

public:
    // Constructor and Destructor
    LTLFormula(std::string formulaStr, const std::vector<BatchAtomicProposition>& batchAPs);
    ~LTLFormula();

    // Core methods
    void parse(const std::string& formula);
    std::string toString() const;
    bool evaluate();

    // Getters and Setters
    void setFormula(const std::string& formula);
    std::string getFormula() const;
    const std::vector<BatchAtomicProposition>& getBatchAPs() const;
    spot::formula getSpotFormula() const;
    BatchAtomicProposition getBatchAP(uint16_t id) const;
    uint16_t getTSState(uint16_t id) const;
    std::vector<BatchAtomicProposition>& getBatchAtomicPropositions();
    const std::vector<BatchAtomicProposition>& getBatchAtomicPropositions() const;
    std::vector<BatchAtomicProposition>& getAtomicPropositions();
    std::vector<bool> getRequiredCapabilities(uint16_t apId) const;
    uint16_t getTotalRequiredCapabilities() const;
    uint16_t getIndependentRequiredCapabilities() const;
    void addBatchAtomicProposition(const BatchAtomicProposition& ap);
    int8_t getBatchVal(uint16_t apId) const;



};

#endif // LTL_FORMULA_H
