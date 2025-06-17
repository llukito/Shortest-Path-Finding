/******************************************************************************
 * File: Trailblazer.cpp
 *
 * Implementation of the graph algorithms that comprise the Trailblazer
 * assignment.
 */

#include "Trailblazer.h"
#include "TrailblazerGraphics.h"
#include "TrailblazerTypes.h"
#include "TrailblazerPQueue.h"

#include <random.h>
using namespace std;

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
    // our priority queue
    TrailblazerPQueue<Loc> pq;
    pq.enqueue(start, heuristic(start, end, world));
    // will remember shortests routes
    Map<Loc, double> dist;
    dist[start] = 0;
    // will remmeber parents of nodes
    Map<Loc, Loc> parents;
    colorCell(world, start, YELLOW);
    // check if we reached end
    bool reached = false;
    while (!pq.isEmpty()) {
        Loc curr = pq.dequeueMin();
        colorCell(world, curr, GREEN);
        if (curr == end) {
            reached = true;
            break;
        }
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                if (i == 0 && j == 0)continue; // we don't stay as same Loc
                int newRow = curr.row + i;
                int newCol = curr.col + j;
                if (!world.inBounds(newRow, newCol))continue;
                Loc newNode = makeLoc(newRow, newCol);
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
    if (!reached) {
        error("Nodes can't be reached");
    }
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
Set<Edge> createMaze(int numRows, int numCols) {
    Map<Loc, Set<Loc>> clasters;
    TrailblazerPQueue<Edge> pq;
    for (int r = 0; r < numRows; r++) {
        for (int c = 0; c < numCols; c++) {
            Loc node = makeLoc(r, c);
            Set<Loc> nodeSet;
            nodeSet.insert(node);
            clasters[node] = nodeSet;
            if (r + 1 < numRows) {
                Loc neigh = makeLoc(r + 1, c);
                Edge newEdge = makeEdge(node, neigh);
                pq.enqueue(newEdge, randomReal(0,1));
            } 
            if (c + 1 < numCols) {
                Loc neigh = makeLoc(r, c+1);
                Edge newEdge = makeEdge(node, neigh);
                pq.enqueue(newEdge, randomReal(0, 1));
            }
        }
    }
    Set<Edge> result;
    while (!pq.isEmpty()) {
        Edge currEdge = pq.dequeueMin();
        Loc node1 = currEdge.start;
        Loc node2 = currEdge.end;

        if (clasters[node1] == clasters[node2]) continue;

        Set<Loc> mergedCluster = clasters[node1] + clasters[node2];
        for (Loc node : mergedCluster) {
            clasters[node] = mergedCluster;
        }

        result.insert(currEdge);
    }
    return result;
}