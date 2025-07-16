/******************************************************************************
 * File: Trailblazer.cpp
 *
 * Implementation of the graph algorithms that comprise the Trailblazer
 * assignment. :)
 */

#include "Trailblazer.h"
#include "TrailblazerGraphics.h"
#include "TrailblazerTypes.h"
#include "TrailblazerPQueue.h"

#include <random.h>
using namespace std;

/*
* Helper function for shortestPath, which is for adding neighbours.
* We had to pass almost every element, but i think it is worht it
* cause it makes is better to read and better to debug
*/
void addNeighbours(Loc curr, Loc end, Grid<double>& world, Set<Loc>& green, Map<Loc, double>& dist,
    double costFn(Loc from, Loc to, Grid<double>& world), 
    Map<Loc, Loc>& parents, TrailblazerPQueue<Loc>& pq, 
    double heuristic(Loc start, Loc end, Grid<double>& world)) {

    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            if (i == 0 && j == 0)continue; // we don't stay as same Loc
            int newRow = curr.row + i;
            int newCol = curr.col + j;
            if (!world.inBounds(newRow, newCol))continue;
            Loc newNode = makeLoc(newRow, newCol);
            if (green.contains(newNode)) {
                continue;
            }
            double newCost = dist[curr] + costFn(curr, newNode, world);
            if (!dist.containsKey(newNode)) { // node is gray
                colorCell(world, newNode, YELLOW);
                dist[newNode] = newCost;
                parents[newNode] = curr;
                pq.enqueue(newNode, newCost + heuristic(newNode, end, world));
            }
            else if (dist[newNode] > newCost) {
                dist[newNode] = newCost;
                parents[newNode] = curr;
                pq.decreaseKey(newNode, newCost + heuristic(newNode, end, world));
            }
        }
    }
}

/* Function: shortestPath
 * 
 * Finds the shortest path between the locations given by start and end in the
 * specified world.	 The cost of moving from one edge to the next is specified
 * by the given cost function.	The resulting path is then returned as a
 * Vector<Loc> containing the locations to visit in the order in which they
 * would be visited.	If no path is found, this function should report an
 * error.
 *
 * In Part Two of this assignment, you will need to add an additional parameter
 * to this function that represents the heuristic to use while performing the
 * search.  Make sure to update both this implementation prototype and the
 * function prototype in Trailblazer.h.
 */
Vector<Loc>
shortestPath(Loc start,
    Loc end,
    Grid<double>& world,
    double costFn(Loc from, Loc to, Grid<double>& world),
    double heuristic(Loc start, Loc end, Grid<double>& world)) {

    // check if start or end are invalid nodes 
    if (!world.inBounds(start.row, start.col) ||
        !world.inBounds(end.row, end.col)) {
        error("Start or end out of bounds");
    }

    // our priority queue
    TrailblazerPQueue<Loc> pq;
    pq.enqueue(start, heuristic(start, end, world));

    // will remember shortests routes in dist
    Map<Loc, double> dist;
    dist[start] = 0;

    // will remmeber parents of nodes
    Map<Loc, Loc> parents;

    // will store already shortest pathe nodes
    Set<Loc> green;

    colorCell(world, start, YELLOW);

    while (!pq.isEmpty()) {
        Loc curr = pq.dequeueMin();
        colorCell(world, curr, GREEN);
        green.add(curr);
        if (curr == end) {
            break;
        }
        addNeighbours(curr, end, world, green, dist, costFn, parents, pq, heuristic);
    }

    // if did not reach it
    if (!green.contains(end)) {
        error("Nodes can't be reached");
    }
    // reconstruct path
    Vector<Loc> result;
    Loc current = end;
    while (true) {
        if (current == start)break;
        result.push_back(current);
        current = parents[current];
    }
    result.push_back(start);
    reverse(result.begin(), result.end());
    return result;
}

/*
* This is our createMaze function, which uses Kruskal's Algorithm.
* We first create clasters for each node and enqueue edges with
* random weight as task said, so edge dequeue stays random.
* After this actual algorithm starts, as we choose least
* weighted edges and continue until we get connected MST.
* This implementation is somewhat slow, on large scale it takes
* 30 seconds and don't try running it on huge scale , it is not
* recommended :). Task also said this is fine btw
*/
//Set<Edge> createMaze(int numRows, int numCols) {
//    Map<Loc, Set<Loc>> clasters;
//    TrailblazerPQueue<Edge> pq;
//    for (int r = 0; r < numRows; r++) {
//        for (int c = 0; c < numCols; c++) {
//            Loc node = makeLoc(r, c);
//            Set<Loc> nodeSet;
//            nodeSet.insert(node);
//            clasters[node] = nodeSet;
//            if (r + 1 < numRows) {
//                Loc neigh = makeLoc(r + 1, c);
//                Edge newEdge = makeEdge(node, neigh);
//                pq.enqueue(newEdge, randomReal(0,1));
//            } 
//            if (c + 1 < numCols) {
//                Loc neigh = makeLoc(r, c+1);
//                Edge newEdge = makeEdge(node, neigh);
//                pq.enqueue(newEdge, randomReal(0, 1));
//            }
//        }
//    }
//    Set<Edge> result;
//    while (!pq.isEmpty()) {
//        Edge currEdge = pq.dequeueMin();
//        Loc node1 = currEdge.start;
//        Loc node2 = currEdge.end;
//
//        if (clasters[node1] == clasters[node2]) continue;
//
//        Set<Loc> mergedCluster = clasters[node1] + clasters[node2];
//        for (Loc node : mergedCluster) {
//            clasters[node] = mergedCluster;
//        }
//
//        result.insert(currEdge);
//    }
//    return result;
//}

// we can optimize create Maze and use union find

/*
* This function searches for root, like starting
* parent LUCA :)
*/
Loc find(Map<Loc, Loc>& parent, Loc x) {
    if (parent[x] != x) {
        parent[x] = find(parent, parent[x]);
    }
    return parent[x];
}

/*
* This function merges two clusters if they are not
* in the same one already. Mergins is typically assigning
* one of the roots to another as a parent
*/
void unionSets(Map<Loc, Loc>& parent, Map<Loc, int>& rank, Loc a, Loc b) {
    Loc rootA = find(parent, a);
    Loc rootB = find(parent, b);
    if (rootA != rootB) {
        if (rank[rootA] < rank[rootB]) {
            parent[rootA] = rootB;
        }
        else if (rank[rootA] > rank[rootB]) {
            parent[rootB] = rootA;
        }
        else {
            parent[rootB] = rootA;
            rank[rootA]++;
        }
    }
}

Set<Edge> createMaze(int numRows, int numCols) {
    Map<Loc, Loc> parent;
    Map<Loc, int> rank;
    int totalCells = numRows * numCols;

    TrailblazerPQueue<Edge> pq;
    for (int r = 0; r < numRows; r++) {
        for (int c = 0; c < numCols; c++) {
            Loc curr = makeLoc(r, c);
            parent[curr] = curr;
            rank[curr] = 0;
            if (r + 1 < numRows) {
                pq.enqueue(makeEdge(curr, makeLoc(r + 1, c)), randomReal(0, 1));
            }
            if (c + 1 < numCols) {
                pq.enqueue(makeEdge(curr, makeLoc(r, c + 1)), randomReal(0, 1));
            }
        }
    }

    Set<Edge> result;
    // second check ensures early exit cause tree with n nodes will
    // always have n-1 edges. Once we get that, rest edges are unnecessary
    while (!pq.isEmpty() && result.size() < totalCells - 1) {
        Edge e = pq.dequeueMin();
        if (find(parent, e.start) != find(parent, e.end)) {
            unionSets(parent,rank, e.start, e.end);
            result.add(e);
        }
    }

    return result;
}
