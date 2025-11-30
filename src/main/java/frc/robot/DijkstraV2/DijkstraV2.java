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
        public final T source;
        public final T destination;
        public final double cost;
        public final DoubleSupplier dynamicCost;
        public Edge(T source, T dest, double cost){
            this.source=source; this.cost=cost; this.destination=dest; this.dynamicCost=()->this.cost;
        };
        public Edge(T source, T dest, double fixedCost, DoubleSupplier dynamicCost){
            this.source=source; this.cost=fixedCost; this.destination=dest; this.dynamicCost=dynamicCost;
        };
        public String toString(){
            return String.format("Edge[%s->%s](%.2f)",source,destination,cost);
        }
    }

    /** The Vertex describes the graph nodes, and during traversal holds the costs 
     * related to travelling to any particular node.
      */
    class Vertex<T> implements Comparable<Vertex<T>>{
        public final T id;
        public double cost=9999;
        public T prev=null;
        public final DoubleSupplier costfunction;

        @Override
        public int compareTo(Vertex<T> o) {
            return Double.compare(cost,o.cost);
        }
        public Vertex(T id){this.id=id;this.costfunction=()->this.cost;}
        public Vertex(T id, DoubleSupplier costfunction){this.id=id;this.costfunction=costfunction;}
        public String toString(){
            return String.format("Vertex[%s]",id);
        }

    }

    //The core datastructures needed for the graph
    HashMap<T,Vertex<T>> nodes = new HashMap<>();
    HashMap<T,HashMap<T,Edge<T>>> edges = new HashMap<>();

    /*
     * To minimize java garbage collection issues, these variables 
     * are set once at maximum possible size, then reused on 
     * subsequent operations.
    */
    private ArrayList<T> path;
    private ArrayList<Vertex<T>> unvisited;

    public DijkstraV2(T instance){
        var vertexCount = instance.getClass().getEnumConstants().length;
        path=new ArrayList<>(vertexCount+1); //+1 to allow potentially pushing a source node or loop
        unvisited=new ArrayList<>(vertexCount);

        for(var t : instance.getClass().getEnumConstants()){
            nodes.put((T)t, new Vertex<T>((T)t) );
            edges.put((T)t, new HashMap<>());
        }
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
        
        //The Dijkstra magic: Calculate travel costs for nodes
        while(unvisited.size()>0){
            var u = unvisited.stream().min((a,b)->Double.compare(a.cost, b.cost)).get(); 
            unvisited.remove(u);

            for(var e : edges.get(u.id).values()){
                var v = nodes.get(e.destination);
                if(unvisited.contains(v)==false)continue;
                var alt = u.cost+e.cost;
                if(alt<v.cost){
                    v.cost=alt;
                    v.prev=u.id;
                }
            }
        }
        //We now follow the chain of lowest costs back to our source 
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

        // System.out.printf("Path(%s,%s) -> %s \n",source,destination,path);
        return path;
    }

    /** Add a uni-directional connection with a dynamic initial cost. 
     * Will not overwrite existing edges or their costs. <br/>
     * Use {@link #getConnection(Enum, Enum)} to modify existing edges.
    */
    public DijkstraV2<T> addConnection(T source, T dest, double cost){
        edges.get(source).putIfAbsent(dest,new Edge<T>(source, dest, cost));
        return this;
    }

    /** Return the connection between these points for further modification */
    public Edge<T> getConnection(T source, T dest){
        for(var e : edges.getOrDefault(source,new HashMap<>()).values()){
            if(e.destination==dest) return e;
        }
        throw(new Error(String.format("Connection (%s->%s) not found",source,dest)));
    }

    /** Add connections between the two nodes*/
    public DijkstraV2<T> addBidirectionalConnection(T a, T b, double cost){
        addConnection(a, b, cost);
        addConnection(b, a, cost);
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