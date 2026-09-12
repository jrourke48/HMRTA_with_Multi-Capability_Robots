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

// A recursive DFS based function used by getSCCs()
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

    // Initialize discovery time and low value
    disc[u] = low[u] = ++timer;

    // Push current vertex to stack and mark it as in stack
    st.push(u);
    inSt[u] = true;

    // Go through all vertices adjacent to this
    for (int v : adj[u]) {

        // If v is not visited yet, then recur for it
        // Case 1: Tree edge
        if (disc[v] == -1) {

            findSCC(v, adj, disc, low, inSt, st, timer, allSCCs);

            // Check if the subtree rooted with v has a
            // connection to one of the ancestors of u
            low[u] = min(low[u], low[v]);
        }

        // Update low value of u only if v is still in stack
        // Case 2: Back edge (not cross edge)
        else if (inSt[v]) {
            low[u] = min(low[u], disc[v]);
        }
    }

    // If u is head node of SCC, pop the stack and store the SCC
    if (low[u] == disc[u]) {

        vector<int> scc;

        // Pop all vertices from stack till u is found
        while (true) {

            int x = st.top();
            st.pop();
            inSt[x] = false;

            scc.push_back(x);

            if (x == u)
                break;
        }

        // Store one strongly connected component
        allSCCs.push_back(scc);
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