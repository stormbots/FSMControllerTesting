package frc.robot.DijkstraV2;
// This impliments the basic Dijkstra algorithm as defined on Wikipedia.
//     1  function Dijkstra(Graph, source):
//     2     
//     3      for each vertex v in Graph.Vertices:
//     4          dist[v] ← INFINITY
//     5          prev[v] ← UNDEFINED
//     6          add v to Q
//     7      dist[source] ← 0
//     8     
//     9      while Q is not empty:
//    10          u ← vertex in Q with minimum dist[u]
//    11          remove u from Q
//    12         
//    13          for each neighbor v of u still in Q:
//    14              alt ← dist[u] + Graph.Edges(u, v)
//    15              if alt < dist[v]:
//    16                  dist[v] ← alt
//    17                  prev[v] ← u
//    18
//    19      return dist[], prev[]

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;

/** 
 * A class to compute path traversal across a state graph.
 */
public class DijkstraV2<T extends Enum<T>> {
    /** Holds the traversal costs between any given nodes */
    class Edge<T>{
        public T source;
        public final double cost;
        public final DoubleSupplier dynamicCost;
        public T destination;
        public Edge(T source, T dest, double cost){
            this.source=source; this.cost=cost; this.destination=dest; this.dynamicCost=()->this.cost;
        };
        public Edge(T source, T dest, double fixedCost, DoubleSupplier dynamicCost){
            this.source=source; this.cost=fixedCost; this.destination=dest; this.dynamicCost=dynamicCost;
        };
    }

    /** The Vertex describes the graph nodes, and during traversal holds the costs 
     * related to travelling to any particular node.
      */
    class Vertex<T> implements Comparable<Vertex<T>>{
        public T id;
        public double cost=9999;
        public T prev=null;
        @Override
        public int compareTo(Vertex<T> o) {
            return Double.compare(cost,o.cost);
        }
        public Vertex(T id){this.id=id;}
    }

    //The core datastructures needed for the graph
    HashMap<T,Vertex<T>> nodes = new HashMap<>();
    HashMap<Vertex<T>,ArrayList<Edge<T>>> edges = new HashMap<>();

    /*
     * To minimize java garbage collection issues, these variables 
     * are set once at maximum possible size, then reused on 
     * subsequent operations.
    */
    private ArrayList<T> path;
    private ArrayList<Vertex<T>> unvisited;
    private ArrayList<Edge<T>> emptyEdgeList = new ArrayList<>(0);


    public DijkstraV2(T instance){
        var capacity = instance.getClass().getEnumConstants().length;
        path=new ArrayList<>(capacity);
        unvisited=new ArrayList<>(capacity);
    }
    

    List<T> findPath(T source, T destination){
        //Clean up our storage 
        path.clear();
        unvisited.clear();

        //The algorithm only works if these two exist.
        if(nodes.containsKey(source)==false) return path;
        if(nodes.containsKey(destination)==false) return path;

        //Reset initial parameters
        nodes.forEach((k,v)->{
            v.cost=99999; //suitable uptime
            v.prev=null;
            unvisited.add(v);
        });
        nodes.get(source).cost=0;
        
        //The Dijkstra magic
        while(unvisited.size()>0){
            var u = unvisited.stream().min((a,b)->Double.compare(a.cost, b.cost)).get(); 
            unvisited.remove(u);

            for(var e : edges.getOrDefault(u,emptyEdgeList)){
                var v = nodes.get(e.destination);
                if(unvisited.contains(v)==false)continue;
                var alt = u.cost+e.cost;
                if(alt<v.cost){
                    v.cost=alt;
                    v.prev=u.id;
                }
            }
        }

        var current = destination;
        while(true){
            path.add(current);
            var cv=nodes.get(current);
            if(cv.prev==null)break;
            current=cv.prev;
        }

        //If we were not able to path successfully, return no path
        if( path.get(0)!=destination ) path.clear();
        if( path.get(path.size()-1)!=source ) path.clear();
        //Dijkstra backtracks from dest to src, so reverse it for the correct route
        Collections.reverse(path);

        System.out.printf("Path(%s,%s) -> %s \n",source,destination,path);
        return path;
    }

    /** Add a uni-directional connection with a dynamic initial cost*/
    private DijkstraV2<T> addConnection(T source, T dest, double cost, DoubleSupplier costFunction){
        var va=nodes.getOrDefault(source, new Vertex<T>(source));
        var vb=nodes.getOrDefault(dest, new Vertex<T>(dest));
        //Handled via null checking to minimize scope captures when creating states
        var edge = costFunction==null
            ? new Edge<T>(source, dest, cost)
            : new Edge<T>(source, dest, cost, costFunction);
        nodes.putIfAbsent(source, va );
        nodes.putIfAbsent(dest, vb );
        edges.putIfAbsent(va, new ArrayList<>());
        edges.get(va).add(edge);
        return this;
    }

    /** Return the connection between these points for further modification */
    public Edge<T> getConnection(T source, T dest){
        for(var e : edges.getOrDefault(source,new ArrayList<>())){
            if(e.destination==dest) return e;
        }
        throw(new Error(String.format("Connection (%s->%s) not found",source,dest)));
    }

    /** Add a uni-directional connection from a to b */
    public DijkstraV2<T> addConnection(T source, T dest, double cost){
        return addConnection(source, dest, cost, null);
    }

    /** Add connections between the two nodes*/
    public DijkstraV2<T> addBidirectionalConnection(T a, T b, double cost){
        addConnection(a, b, cost, null);
        addConnection(b, a, cost, null);
        return this;
    }

    /** Create connections between several points in a sequence with identical costs.
     * Syntactic sugar for multiple {@link #addConnection(Enum, Enum, double)} calls.
     */
    public DijkstraV2<T> addDirectionalSequence(double cost, T... states){
        if(states.length<2){
            throw(new Error("Need 2 or more nodes to form connections"));
        }
        for(var i=1;i<states.length; i++){
            addConnection(states[i-1],states[i],cost);
        }
        return this;
    }

    /** Create connections between several points in a sequence with identical costs.
     * Syntactic sugar for multiple {@link #addBidirectionalConnection(Enum, Enum, double)} calls.
     */
    public DijkstraV2<T> addBidirectionalSequence(double cost, T... states){
        if(states.length<2){
            throw(new Error("Need 2 or more nodes to form connections"));
        }
        for(var i=1;i<states.length; i++){
            addConnection(states[i-1],states[i],cost);
            addConnection(states[i],states[i-1],cost);
        }
        return this;
    }

}