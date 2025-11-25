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
import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;

public class DijkstraV2<T extends Enum<T>> {
    class Edge<T>{
        public double cost;
        public DoubleSupplier dynamicCost=()->cost;
        public T destination;
        public Edge(T dest, double cost){this.cost=cost; this.destination=dest;};
        public Edge(T dest, double fixedCost, DoubleSupplier dynamicCost){
            this.cost=fixedCost; this.destination=dest; this.dynamicCost=dynamicCost;
        };
    }

    class Vertex<T>{
        public HashMap<T,Edge<T>> list = new HashMap<>();
        public double dist=9999;
        public T prev=null;
    }

    HashMap<T,Vertex<T>> graph = new HashMap<>();

    public DijkstraV2(T instance){
    }
    
    public List computeGross(T source, T destination){
        var Q = new ArrayList<Vertex<T>>(); //unvisited; TODO private global scope to clear faster
        var path = new ArrayList<T>(); //maybe the same? This one outlives the function, replaces the fsm one

        //Reset initial parameters
        graph.values().forEach((v)->{
            v.dist=99999; //suitable uptime
            v.prev=null;
            Q.add(v);
        });
        graph.get(source).dist=0;

        //     9      while Q is not empty:
        while(Q.isEmpty()==false){
            Vertex<T> u = Q.stream().min((a,b)->Double.compare(a.dist, b.dist)).get(); //TODO Unchecked optional unwrap
            Q.remove(u);

            //u is current node. 
            // Q is remaining nodes to evaluate
            // v is next node 
//          for each neighbor v of u still in Q:
//              alt ← dist[u] + Graph.Edges(u, v)
            for(var v : u.list.values()){
                if(Q.contains(v)==false){ continue; }
//    14        alt ← dist[u] + Graph.Edges(u, v)
                var alt = v.cost + u.dist;
//    15        if alt < dist[v]:
                if(alt<v.cost){
                    v.cost = alt;
//    16            dist[v] ← alt
//    17            prev[v] ← u
                    // u.prev = v;
//    18       }
                }
            }

        }


        
        return path;
    }

    void compute(T source, T destination){
        var Q = new ArrayList<Vertex<T>>(); //unvisited; TODO private global scope to clear faster
        var path = new ArrayList<T>(); //maybe the same? This one outlives the function, replaces the fsm one

        //Reset initial parameters
        graph.values().forEach((v)->{
            v.dist=99999; //suitable uptime
            v.prev=null;
            Q.add(v);
        });
        graph.get(source).dist=0;

        //rest of the owl
    }

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
}
