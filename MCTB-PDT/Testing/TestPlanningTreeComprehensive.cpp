#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include "../Tree/PlanningDecisionTree.h"
#include "../Tree/Tree_Node.h"

/**
 * COMPREHENSIVE PLANNING TREE TEST SUITE
 * Tests all PlanningDecisionTree methods and Tree_Node operations
 */

// ============================================================================
// TEST 1: Basic Tree Construction
// ============================================================================
void testBasicTreeConstruction() {
    std::cout << "\n=== TEST 1: Basic Tree Construction ===" << std::endl;
    
    // Create simple nodes for testing
    Node* nbaNode = new Node(0);
    Node* tsNode = new Node(0);
    
    std::vector<bool> taskAlloc = {true, false, true};
    std::vector<uint16_t> times = {5, 10, 3};
    
    // Create tree with root
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode, tsNode, taskAlloc, times, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    assert(tree != nullptr);
    assert(!tree->isEmpty());
    assert(tree->getNodeCount() == 1);
    assert(tree->getRoot() != nullptr);
    std::cout << "✓ Tree created successfully" << std::endl;
    std::cout << "✓ Root node ID: " << tree->getRoot()->getId() << std::endl;
    std::cout << "✓ Initial node count: " << tree->getNodeCount() << std::endl;
    
    delete tree;
    std::cout << "✓ Tree destructor called successfully" << std::endl;
}

// ============================================================================
// TEST 2: Insert Node (Two Overloads)
// ============================================================================
void testInsertNode() {
    std::cout << "\n=== TEST 2: Insert Node (Two Overloads) ===" << std::endl;
    
    Node* nbaNode0 = new Node(0);
    Node* tsNode0 = new Node(0);
    
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode0, tsNode0, {true, false}, {5, 10}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    Tree_Node* root = tree->getRoot();
    assert(root != nullptr);
    
    // Test overload 1: insertNode with parameters
    std::cout << "\n--- Testing insertNode(parent, automatonState, tsState, ...) ---" << std::endl;
    Node* nbaNode1 = new Node(1);
    Node* tsNode1 = new Node(1);
    Tree_Node* child1 = tree->insertNode(root, nbaNode1, tsNode1, {false, true}, {8, 7}, 1, Tree_Node::TASK_PROGRESS::PRE);
    
    assert(child1 != nullptr);
    assert(tree->getNodeCount() == 2);
    assert(child1->getParent() == root);
    assert(child1->getId() == 1);
    std::cout << "✓ Child 1 created with ID: " << child1->getId() << std::endl;
    std::cout << "✓ Tree node count: " << tree->getNodeCount() << std::endl;
    
    // Test overload 2: insertNode with pre-constructed node
    std::cout << "\n--- Testing insertNode(Tree_Node*) ---" << std::endl;
    Node* nbaNode2 = new Node(2);
    Node* tsNode2 = new Node(2);
    Tree_Node* child2 = new Tree_Node(0, root, nbaNode2, tsNode2, {true, true}, {6, 9}, 0, Tree_Node::TASK_PROGRESS::PRE);
    Tree_Node* inserted = tree->insertNode(child2);
    
    assert(inserted != nullptr);
    assert(inserted->getId() == 2);  // Should be auto-assigned
    assert(tree->getNodeCount() == 3);
    std::cout << "✓ Pre-constructed node inserted with auto-assigned ID: " << inserted->getId() << std::endl;
    std::cout << "✓ Tree node count: " << tree->getNodeCount() << std::endl;
    
    delete tree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 3: Frontier Management
// ============================================================================
void testFrontierManagement() {
    std::cout << "\n=== TEST 3: Frontier Management ===" << std::endl;
    
    Node* nbaNode = new Node(0);
    Node* tsNode = new Node(0);
    
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode, tsNode, {true}, {5}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    Tree_Node* root = tree->getRoot();
    const std::vector<Tree_Node*>& frontier = tree->getFrontierNodes();
    
    std::cout << "\n--- Initial frontier state ---" << std::endl;
    assert(frontier.size() == 1);
    assert(frontier[0] == root);
    std::cout << "✓ Initial frontier size: " << frontier.size() << std::endl;
    std::cout << "✓ Root is in frontier" << std::endl;
    
    // Add a child - frontier should change
    std::cout << "\n--- After inserting child ---" << std::endl;
    Node* nbaNode1 = new Node(1);
    Node* tsNode1 = new Node(1);
    Tree_Node* child = tree->insertNode(root, nbaNode1, tsNode1, {false}, {7}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    const std::vector<Tree_Node*>& newFrontier = tree->getFrontierNodes();
    assert(newFrontier.size() == 1);
    assert(newFrontier[0] == child);
    assert(std::find(newFrontier.begin(), newFrontier.end(), root) == newFrontier.end());
    std::cout << "✓ Root removed from frontier (no longer leaf)" << std::endl;
    std::cout << "✓ Child added to frontier" << std::endl;
    std::cout << "✓ Frontier size: " << newFrontier.size() << std::endl;
    
    // Add multiple children to same parent
    std::cout << "\n--- After inserting multiple children ---" << std::endl;
    Node* nbaNode2 = new Node(2);
    Node* tsNode2 = new Node(2);
    tree->insertNode(root, nbaNode2, tsNode2, {true}, {6}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    const std::vector<Tree_Node*>& multiFrontier = tree->getFrontierNodes();
    assert(multiFrontier.size() == 2);
    std::cout << "✓ Multiple frontier nodes maintained" << std::endl;
    std::cout << "✓ Frontier size: " << multiFrontier.size() << std::endl;
    
    delete tree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 4: Get All Nodes
// ============================================================================
void testGetAllNodes() {
    std::cout << "\n=== TEST 4: Get All Nodes ===" << std::endl;
    
    Node* nbaNode = new Node(0);
    Node* tsNode = new Node(0);
    
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode, tsNode, {true}, {5}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    Tree_Node* root = tree->getRoot();
    
    // Start with root only
    std::cout << "\n--- Tree with 1 node ---" << std::endl;
    std::vector<Tree_Node*> allNodes = tree->getAllNodes();
    assert(allNodes.size() == 1);
    std::cout << "✓ getAllNodes() returned 1 node" << std::endl;
    
    // Add children
    std::cout << "\n--- Adding nodes to tree ---" << std::endl;
    Node* nbaNode1 = new Node(1);
    Node* tsNode1 = new Node(1);
    tree->insertNode(root, nbaNode1, tsNode1, {false}, {7}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    Node* nbaNode2 = new Node(2);
    Node* tsNode2 = new Node(2);
    tree->insertNode(root, nbaNode2, tsNode2, {true}, {6}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    Node* nbaNode3 = new Node(3);
    Node* tsNode3 = new Node(3);
    tree->insertNode(root, nbaNode3, tsNode3, {false}, {8}, 0, Tree_Node::TASK_PROGRESS::TRA);
    
    allNodes = tree->getAllNodes();
    std::cout << "✓ getAllNodes() returned " << allNodes.size() << " nodes" << std::endl;
    
    // Check that root is reachable
    assert(std::find(allNodes.begin(), allNodes.end(), root) != allNodes.end());
    std::cout << "✓ Root found in all nodes" << std::endl;
    
    delete tree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 5: Delete Subtree
// ============================================================================
void testDeleteSubtree() {
    std::cout << "\n=== TEST 5: Delete Subtree ===" << std::endl;
    
    Node* nbaNode = new Node(0);
    Node* tsNode = new Node(0);
    
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode, tsNode, {true}, {5}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    Tree_Node* root = tree->getRoot();
    
    // Add children
    Node* nbaNode1 = new Node(1);
    Node* tsNode1 = new Node(1);
    Tree_Node* child = tree->insertNode(root, nbaNode1, tsNode1, {false}, {7}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    std::cout << "\n--- Initial state ---" << std::endl;
    std::cout << "✓ Node count before deletion: " << tree->getNodeCount() << std::endl;
    
    // Delete the child
    std::cout << "\n--- After deletion ---" << std::endl;
    tree->deleteSubtree(child);
    std::cout << "✓ Child node deleted" << std::endl;
    std::cout << "✓ Node count after deletion: " << tree->getNodeCount() << std::endl;
    
    delete tree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 6: Insert Subtree
// ============================================================================
void testInsertSubtree() {
    std::cout << "\n=== TEST 6: Insert Subtree ===" << std::endl;
    
    // Create first tree
    Node* nbaNode0 = new Node(0);
    Node* tsNode0 = new Node(0);
    
    PlanningDecisionTree* mainTree = new PlanningDecisionTree(
        nbaNode0, tsNode0, {true}, {5}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    std::cout << "\n--- Main tree created ---" << std::endl;
    std::cout << "✓ Main tree node count: " << mainTree->getNodeCount() << std::endl;
    
    Tree_Node* mainRoot = mainTree->getRoot();
    
    // Create subtree
    Node* nbaSub0 = new Node(1);
    Node* tsSub0 = new Node(1);
    
    PlanningDecisionTree* subtree = new PlanningDecisionTree(
        nbaSub0, tsSub0, {false}, {7}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    std::cout << "\n--- Subtree created ---" << std::endl;
    std::cout << "✓ Subtree node count: " << subtree->getNodeCount() << std::endl;
    
    // Add a child to subtree
    Node* nbaSub1 = new Node(2);
    Node* tsSub1 = new Node(2);
    subtree->insertNode(subtree->getRoot(), nbaSub1, tsSub1, {true}, {3}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    std::cout << "\n--- Subtree expanded ---" << std::endl;
    std::cout << "✓ Subtree node count after adding child: " << subtree->getNodeCount() << std::endl;
    
    // Insert subtree into main tree
    std::cout << "\n--- Inserting subtree into main tree ---" << std::endl;
    mainTree->insertSubtree(mainRoot, subtree);
    
    std::cout << "✓ Subtree inserted successfully" << std::endl;
    std::cout << "✓ Main tree node count after insertion: " << mainTree->getNodeCount() << std::endl;
    assert(mainTree->getNodeCount() == 3);  // Root + 2 from subtree
    
    // Check frontier
    const std::vector<Tree_Node*>& frontier = mainTree->getFrontierNodes();
    std::cout << "✓ Frontier size: " << frontier.size() << std::endl;
    
    delete mainTree;  // This should clean up subtree too
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 7: Empty Tree Operations
// ============================================================================
void testEmptyTreeOperations() {
    std::cout << "\n=== TEST 7: Empty Tree Operations ===" << std::endl;
    
    PlanningDecisionTree* emptyTree = new PlanningDecisionTree();
    
    assert(emptyTree->isEmpty());
    assert(emptyTree->getNodeCount() == 0);
    assert(emptyTree->getRoot() == nullptr);
    std::cout << "✓ Empty tree is empty" << std::endl;
    std::cout << "✓ Root is nullptr" << std::endl;
    std::cout << "✓ Node count is 0" << std::endl;
    
    std::vector<Tree_Node*> allNodes = emptyTree->getAllNodes();
    assert(allNodes.empty());
    std::cout << "✓ getAllNodes() returns empty vector" << std::endl;
    
    delete emptyTree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 8: Clear Tree
// ============================================================================
void testClearTree() {
    std::cout << "\n=== TEST 8: Clear Tree ===" << std::endl;
    
    Node* nbaNode = new Node(0);
    Node* tsNode = new Node(0);
    
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode, tsNode, {true}, {5}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    Tree_Node* root = tree->getRoot();
    
    // Add some nodes
    Node* nbaNode1 = new Node(1);
    Node* tsNode1 = new Node(1);
    tree->insertNode(root, nbaNode1, tsNode1, {false}, {7}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    std::cout << "\n--- Before clear ---" << std::endl;
    std::cout << "✓ Node count: " << tree->getNodeCount() << std::endl;
    assert(tree->getNodeCount() == 2);
    
    // Clear tree
    tree->clearTree();
    
    std::cout << "\n--- After clear ---" << std::endl;
    std::cout << "✓ Node count: " << tree->getNodeCount() << std::endl;
    assert(tree->isEmpty());
    assert(tree->getRoot() == nullptr);
    std::cout << "✓ Tree is empty" << std::endl;
    std::cout << "✓ Root is nullptr" << std::endl;
    
    delete tree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 9: Tree Node Parent-Child Relationships
// ============================================================================
void testNodeParentChildRelationships() {
    std::cout << "\n=== TEST 9: Tree Node Parent-Child Relationships ===" << std::endl;
    
    Node* nbaNode = new Node(0);
    Node* tsNode = new Node(0);
    
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode, tsNode, {true}, {5}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    Tree_Node* root = tree->getRoot();
    assert(root->getParent() == nullptr);
    std::cout << "✓ Root has no parent" << std::endl;
    
    // Add children
    Node* nbaNode1 = new Node(1);
    Node* tsNode1 = new Node(1);
    Tree_Node* child1 = tree->insertNode(root, nbaNode1, tsNode1, {false}, {7}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    Node* nbaNode2 = new Node(2);
    Node* tsNode2 = new Node(2);
    Tree_Node* child2 = tree->insertNode(root, nbaNode2, tsNode2, {true}, {6}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    std::cout << "\n--- After adding children ---" << std::endl;
    assert(child1->getParent() == root);
    assert(child2->getParent() == root);
    std::cout << "✓ Child 1 parent is root" << std::endl;
    std::cout << "✓ Child 2 parent is root" << std::endl;
    std::cout << "✓ Parent-child relationships verified" << std::endl;
    
    delete tree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 10: Node Data Storage and Retrieval
// ============================================================================
void testNodeDataStorage() {
    std::cout << "\n=== TEST 10: Node Data Storage and Retrieval ===" << std::endl;
    
    Node* nbaNode = new Node(5);
    Node* tsNode = new Node(10);
    std::vector<bool> taskAlloc = {true, false, true, true};
    std::vector<uint16_t> times = {12, 25, 8, 15};
    int8_t batchVal = 3;
    Tree_Node::TASK_PROGRESS progress = Tree_Node::TASK_PROGRESS::PRE;
    
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode, tsNode, taskAlloc, times, batchVal, progress
    );
    
    Tree_Node* root = tree->getRoot();
    
    std::cout << "\n--- Checking stored data ---" << std::endl;
    assert(root->getAutomatonState()->getId() == 5);
    assert(root->getTSState()->getId() == 10);
    assert(root->getBatch() == batchVal);
    assert(root->getProgress() == progress);
    
    auto storedTimes = root->getTimes();
    assert(storedTimes == times);
    
    auto storedAlloc = root->getRoboTaskAllocation();
    assert(storedAlloc == taskAlloc);
    
    std::cout << "✓ Automaton state ID: " << root->getAutomatonState()->getId() << std::endl;
    std::cout << "✓ TS state ID: " << root->getTSState()->getId() << std::endl;
    std::cout << "✓ Batch value: " << static_cast<int>(root->getBatch()) << std::endl;
    std::cout << "✓ Progress: " << static_cast<int>(progress) << std::endl;
    std::cout << "✓ Times match stored values" << std::endl;
    std::cout << "✓ Task allocation matches stored values" << std::endl;
    
    delete tree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================
int main() {
    std::cout << "\n";
    std::cout << "========================================" << std::endl;
    std::cout << "PLANNING TREE COMPREHENSIVE TEST SUITE" << std::endl;
    std::cout << "========================================" << std::endl;
    
    try {
        testBasicTreeConstruction();
        testInsertNode();
        testFrontierManagement();
        testGetAllNodes();
        testDeleteSubtree();
        testInsertSubtree();
        testEmptyTreeOperations();
        testClearTree();
        testNodeParentChildRelationships();
        testNodeDataStorage();
        
        std::cout << "\n";
        std::cout << "========================================" << std::endl;
        std::cout << "✓ ALL TESTS PASSED" << std::endl;
        std::cout << "========================================" << std::endl;
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "\n✗ TEST FAILED WITH EXCEPTION: " << e.what() << std::endl;
        return 1;
    }
}
