/*
 *  File: trailblazer.cpp
 *  - - - - - - - - - - - - - - -
 *  This file implements the Depth First, Binary Search
 *  Dijkstra's, A* and Kruskal algorithms given
 *  a graph passed in by reference.
 */

#include "costs.h"
#include "trailblazer.h"
#include "queue.h"
#include "pqueue.h"
#include "map.h"
#include "set.h"
using namespace std;

//Prototypes
bool depthFirstSearch(Vertex* start, Vertex* end, Vector<Vertex*>& path);
void visitVertex(Vertex* vertex);
void eliminateVertex(Vertex* vertex, Vector<Vertex*>& path);
void enqueueVertex(Vertex* vertex, Vertex* previous, double cost);
void determinePath(Vertex* start, Vertex* end, Vector<Vertex*>& path);
void reverseVector(Vector<Vertex*>& path);
void initializeClusters(BasicGraph& graph, Map<Vertex*, Set<Vertex*> >& clusters);
void enqueueEdges(BasicGraph& graph, PriorityQueue<Edge*>& pqueue);
void mergeClusters(Map<Vertex*, Set<Vertex*> >& clusters, Vertex* start, Vertex* finish);

/*
 *  Method: depthFirstSearch
 *  Parameters: BasicGraph graph by reference
 *              Vertex pointer variable for path start
 *              Vertex pointer variable for path end
 *  - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  Returns a Vector of Vertex pointer variables for which
 *  each index corresponds to a step in the path between
 *  the path's start and end points, if any exists.
 *  This eponymous method is actually a wrapper function
 *  for the recursive backtracking method it calls.
 *  If no path is found, an empty vector is returned.
 */
Vector<Vertex*> depthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path;
    if(depthFirstSearch(start, end, path)) {
    } else {
        path.clear();
    }
    return path;
}

/*
 *  Method: depthFirstSearch
 *  Parameters: Vertex pointer variable for path start
 *              Vertex pointer variable for path end
 *              Vector of Vertex pointers maintaining path
 *  - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  Recursive function that indicates whether the next step
 *  along the path is good or bad. Its base cases are whether
 *  the current location and destination are equivalent,
 *  returning true if so; or whether the current location
 *  has already been visitied, returning false.
 *  The function employs two helper functions to
 *  maintain the path Vector and update the GUI.
 */
bool depthFirstSearch(Vertex* start, Vertex* end, Vector<Vertex*>& path) {
    if(BasicGraph::compare(start, end) == 0) {
        path.add(start);
        visitVertex(start);
        return true;
    }
    if(start->visited) return false;
    path.add(start);
    visitVertex(start);
    for(Edge* edge : start->edges) {
        if(!edge->finish->visited) {
            if(depthFirstSearch(edge->finish, end, path)) return true;
        }
    }
    eliminateVertex(start, path);
    return false;
}

/*
 *  Method: visitVertex
 *  Parameters: Vertex pointer variable for given location
 *  - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  Helper function that carries out the repetitive tasks
 *  of a successful DFS move, including: 
 *    - marking the current Vertex pointer as visited
 *    - updating the GUI to indicate a Vertex visit
 */
void visitVertex(Vertex* vertex) {
    vertex->visited = true;
    vertex->setColor(GREEN);
}

/*
 *  Method: eliminateVertex
 *  Parameters: Vertex pointer variable for given location
 *              Vector of Vertex pointers maintaining path
 *  - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  Helper function that carries out the repetitive tasks
 *  of an unsuccessful DFS move, including: 
 *    - removing the current Vertex pointer varaible from the 
 *      Vector maintaining the path
 *    - updating the GUI to indicate an unsuccessful Vertex visit
 */
void eliminateVertex(Vertex* vertex, Vector<Vertex*>& path) {
    path.remove(path.size()-1);
    vertex->setColor(GRAY);
}

/*
 *  Method: breadthFirstSearch
 *  Parameters: BasicGraph graph by reference
 *              Vertex pointer variable for path start
 *              Vertex pointer variable for path end
 *  - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  Returns a Vector of Vertex pointer variables for which
 *  each index corresponds to a step in the path between
 *  the path's start and end points, if any exists.
 *  This function opts for a breadth first search, which
 *  evaluates all neighbors of the starting vertex as a
 *  potential end point before continuing. Because BFS
 *  orients subsequent vertices to their earlier parent,
 *  this function uses a helper function to rewind the
 *  shortest route from end to start, if applicable,
 *  returning an empty vector if not.
 */
Vector<Vertex*> breadthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path;
    Queue<Vertex*> queue;
    queue.enqueue(start);
    while(!queue.isEmpty()) {
        Vertex* current = queue.dequeue();
        visitVertex(current);
        if(BasicGraph::compare(current, end) == 0) break;
        for(Edge* edge : current->edges) {
            if(!edge->finish->visited &&
               edge->finish->getColor() != YELLOW) {
                enqueueVertex(edge->finish, current, 0);
                queue.enqueue(edge->finish);
            }
        }
    }
    determinePath(start, end, path);
    return path;
}

/*
 *  Method: enqueueVertex
 *  Parameters: Vertex pointer variable for given location
 *              Vertex pointer variable for parent location
 *              Cost of path to that point as double
 *  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  Helper function that carries out the repetitive tasks
 *  of enqueueing a location for processing, including:
 *    - assigning the passed cost to the given location
 *    - aligning the given location with its parent
 *    - updating the GUI to indicate an enqueueing
 */
void enqueueVertex(Vertex* vertex, Vertex* previous, double cost) {
    vertex->cost = cost;
    vertex->previous = previous;
    vertex->setColor(YELLOW);
}

/*
 *  Method: determinePath
 *  Parameters: Vertex pointer variable for path start
 *              Vertex pointer variable for path end
 *              Vector of Vertex pointers maintaining path
 *  - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  Helper function that backtracks from the endpoint vertex
 *  to the starting point, if applicable, using the
 *  'previous' pointer that maps a given vertex to its parent.
 *  If valid, the function adds the vertex to a Vector
 *  to be returned by its calling function, which it does so
 *  after calling a separate helper function to reverse
 *  the Vector's order so to accurately represent the shortest
 *  path taken from start to end.
 */
void determinePath(Vertex* start, Vertex* end, Vector<Vertex*>& path) {
    Vertex* vertex = end;
    while(true) {
        path.add(vertex);
        if(BasicGraph::compare(vertex, start) == 0) break;
        if(!vertex->previous) {
            path.clear();
            break;
        }
        vertex = vertex->previous;
    }
    reverseVector(path);
}

/*
 *  Method: reverseVector
 *  Parameters: Vector of Vertex pointers maintaining path
 *  - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  Helper function that simply reverses the order of
 *  the given Vector's values.
 */
void reverseVector(Vector<Vertex*>& path) {
    for(int i = 0; i < path.size() / 2; i++) {
        int j = path.size() - 1 - i;
        Vertex* tmp = path[i];
        path[i] = path[j];
        path[j] = tmp;
    }
}

/*
 *  Method: dijkstrasAlgorithm
 *  Parameters: BasicGraph graph by reference
 *              Vertex pointer variable for path start
 *              Vertex pointer variable for path end
 *  - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  Returns a Vector of Vertex pointer variables for which
 *  each index corresponds to a step in the path between
 *  the path's start and end points, if any exists.
 *  This function uses Dijkstra's algorithm, which
 *  evaluates the cost of all neighbors of the starting vertex as a
 *  potential end point before continuing. It differs from BFS
 *  in its prioritization of location based on their cost
 *  or distance from the starting point. Because Dijkstra's
 *  orients subsequent vertices to their earlier parent,
 *  this function uses a helper function to rewind the
 *  shortest route from end to start, if applicable,
 *  returning an empty vector if not.
 */
Vector<Vertex*> dijkstrasAlgorithm(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path;
    PriorityQueue<Vertex*> pqueue;
    pqueue.enqueue(start, 0.0);
    while(!pqueue.isEmpty()) {
        Vertex* current = pqueue.dequeue();
        visitVertex(current);
        if(BasicGraph::compare(current, end) == 0) break;
        for(Edge* edge : current->edges) {
            double cost = current->cost + edge->cost;
            if(!edge->finish->visited &&
               edge->finish->getColor() != YELLOW) {
                enqueueVertex(edge->finish, current, cost);
                pqueue.enqueue(edge->finish, cost);
            }
            if(edge->finish->getColor() == YELLOW &&
               cost < edge->finish->cost) {
                enqueueVertex(edge->finish, current, cost);
                pqueue.changePriority(edge->finish, cost);
            }
        }
    }
    determinePath(start, end, path);
    return path;
}

/*
 *  Method: aStar
 *  Parameters: BasicGraph graph by reference
 *              Vertex pointer variable for path start
 *              Vertex pointer variable for path end
 *  - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  Returns a Vector of Vertex pointer variables for which
 *  each index corresponds to a step in the path between
 *  the path's start and end points, if any exists.
 *  This function uses A*'s algorithm, which evaluates
 *  the estimated total cost of all neighbors' paths to the end
 *  from the starting vertex before continuing. It differs from Dijkstra's
 *  in its prioritization of location based on their estimated total cost
 *  or distance from the starting point, as opposed to the current cost
 *  up to that point. Because A* also orients subsequent vertices
 *  to their earlier parent, this function uses a helper function to
 *  rewind the shortest route from end to start, if applicable,
 *  returning an empty vector if not.
 */
Vector<Vertex*> aStar(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path;
    PriorityQueue<Vertex*> pqueue;
    pqueue.enqueue(start, heuristicFunction(start, end));
    while(!pqueue.isEmpty()) {
        Vertex* current = pqueue.dequeue();
        visitVertex(current);
        if(BasicGraph::compare(current, end) == 0) break;
        for(Edge* edge : current->edges) {
            double cost = current->cost + edge->cost;
            if(!edge->finish->visited &&
               edge->finish->getColor() != YELLOW) {
                enqueueVertex(edge->finish, current, cost);
                pqueue.enqueue(edge->finish, cost + 
                               heuristicFunction(edge->finish, end));
            }
            if(edge->finish->getColor() == YELLOW &&
               cost < edge->finish->cost) {
                enqueueVertex(edge->finish, current, cost);
                pqueue.changePriority(edge->finish, cost + 
                                      heuristicFunction(edge->finish, end));
            }
        }
    }
    determinePath(start, end, path);
    return path;
}

/*
 *  Method: kruskal
 *  Parameters: BasicGraph graph by reference
 *  - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  Returns a set of Edges that correspond to the minimum
 *  spanning tree of the graph passed by reference. This Kruskal
 *  algorithm implementation leverages a Priority Queue to
 *  connect each Vertex with its minimum cost Edge without
 *  creating cycles, ensuring its a minimum spanning tree.
 */
Set<Edge*> kruskal(BasicGraph& graph) {
    Set<Edge*> mst;
    Map<Vertex*, Set<Vertex*> > clusters;
    PriorityQueue<Edge*> pqueue;
    initializeClusters(graph, clusters);
    enqueueEdges(graph, pqueue);
    while(!pqueue.isEmpty()) {
        Edge* edge = pqueue.dequeue();
        if(!clusters[edge->start].contains(edge->finish)) {
            mergeClusters(clusters, edge->start, edge->finish);
            mst.add(edge);
        }
    }
    return mst;
}

/*
 *  Method: initializeClusters
 *  Parameters: BasicGraph graph by reference
 *              Map of clusters with keys as Vertices and 
 *              values as connecting endpoints in a set
 *  - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  Returns by reference a map of clusters for the
 *  graph passed by reference. Since this is called at 
 *  initialization, each Vertex is its own cluster 
 *  and its connected endpoints is just itself.
 */
void initializeClusters(BasicGraph& graph, Map<Vertex*, Set<Vertex*> >& clusters) {
    Set<Vertex*> graphVertices = graph.getVertexSet();
    for(Vertex* vertex : graphVertices) {
        Set<Vertex*> otherVertices;  //Does a cluster include itself in connecting Vertices?
        otherVertices.add(vertex);
        clusters[vertex] = otherVertices;
    }
}

/*
 *  Method: enqueueEdges
 *  Parameters: BasicGraph graph by reference
 *              PriorityQueue of Edge pointer variables
 *              that comprise the graph
 *  - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  Returns by reference a Priority Queue of Edge pointer
 *  variables according to their randomly assigned weights.
 *  This PQueue is then used to construct the minimum spanning tree.
 */
void enqueueEdges(BasicGraph& graph, PriorityQueue<Edge*>& pqueue) {
    Set<Edge*> graphEdges = graph.getEdgeSet();
    for(Edge* edge : graphEdges) {
        pqueue.enqueue(edge, edge->cost);
    }
}

/*
 *  Method: mergeClusters
 *  Parameters: Map of clusters with keys as Vertices and 
 *              values as connecting endpoints in a set
 *              Vertex corresponding to edge's starting point
 *              Vertex corresponding to edge's end point
 *  - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *  Merges two clusters given an Edge's starting and end points,
 *  and includes the new clusters and any changes needed to other
 *  Vertices in the map used to track clusters by reference.
 */
void mergeClusters(Map<Vertex*, Set<Vertex*> >& clusters, Vertex* start, Vertex* finish) {
    Set<Vertex*> otherVertices= clusters[start] + clusters[finish];
    for(Vertex* vertex : clusters) {
        if(otherVertices.contains(vertex)) clusters[vertex] = otherVertices;
    }
}
