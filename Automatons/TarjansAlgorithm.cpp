#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
using namespace std;
// Tarjan Algorithm is based on the following facts: 
// 1. DFS search produces a DFS tree/forest 
// 2. Strongly Connected Components form subtrees of the DFS tree. 
// 3. If we can find the head of such subtrees, we can print/store all the nodes in that subtree (including the head) and that will be one SCC. 
// 4. There is no back edge from one SCC to another (There can be cross edges, but cross edges will not be used while processing the graph).

// An iterative DFS based function used by getSCCs()
// Uses an explicit call stack instead of recursion so large graphs with long chains
// cannot overflow the program stack
// u        -> The vertex to be visited next
// disc[]   -> Stores discovery times of visited vertices
// low[]    -> Earliest visited vertex that can be reached
//             from subtree rooted with current vertex
// st       -> Stack to store all active DFS vertices
// inSt[]   -> Boolean array to check whether a node is in stack
// timer    -> Global time counter for discovery times
// allSCCs  -> Stores all strongly connected components
void findSCC(int u, vector<vector<int>> &adj, vector<int> &disc, vector<int> &low,
             vector<bool> &inSt, stack<int> &st, int &timer, vector<vector<int>> &allSCCs) {

    // Each frame is (vertex, index of the next adjacent vertex to look at),
    // replacing the local state a recursive call would keep
    vector<pair<int, size_t>> callStack;

    // Initialize discovery time and low value
    disc[u] = low[u] = ++timer;

    // Push current vertex to stack and mark it as in stack
    st.push(u);
    inSt[u] = true;
    callStack.push_back({u, 0});

    while (!callStack.empty()) {
        int w = callStack.back().first;
        size_t& next = callStack.back().second;

        // Go through the remaining vertices adjacent to w
        if (next < adj[w].size()) {
            int v = adj[w][next];
            next++;

            // If v is not visited yet, then visit it next (in place of a recursive call)
            // Case 1: Tree edge
            if (disc[v] == -1) {
                disc[v] = low[v] = ++timer;
                st.push(v);
                inSt[v] = true;
                callStack.push_back({v, 0});
            }

            // Update low value of w only if v is still in stack
            // Case 2: Back edge (not cross edge)
            else if (inSt[v]) {
                low[w] = min(low[w], disc[v]);
            }
            continue;
        }

        // All neighbors of w are done: this is where the recursive call would return
        callStack.pop_back();

        // If w is head node of SCC, pop the stack and store the SCC
        if (low[w] == disc[w]) {

            vector<int> scc;

            // Pop all vertices from stack till w is found
            while (true) {

                int x = st.top();
                st.pop();
                inSt[x] = false;

                scc.push_back(x);

                if (x == w)
                    break;
            }

            // Store one strongly connected component
            allSCCs.push_back(scc);
        }

        // Check if the subtree rooted with w has a
        // connection to one of the ancestors of its parent
        if (!callStack.empty()) {
            int parent = callStack.back().first;
            low[parent] = min(low[parent], low[w]);
        }
    }
}

// The function to do DFS traversal.
// It uses findSCC() to find all strongly connected components
vector<vector<int>> getSCCs(vector<vector<int>> &adj) {

    int n = adj.size();

    vector<int> disc(n, -1);
    vector<int> low(n, -1);
    vector<bool> inSt(n, false);

    stack<int> st;
    int timer = 0;

    vector<vector<int>> allSCCs;

    // Call the recursive helper function to find SCCs
    // in DFS tree with vertex i
    for (int i = 0; i < n; i++) {

        if (disc[i] == -1) {
            findSCC(i, adj, disc, low, inSt, st, timer, allSCCs);
        }
    }

    return allSCCs;
}
