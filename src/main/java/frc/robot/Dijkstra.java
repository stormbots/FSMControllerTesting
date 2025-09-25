package frc.robot;
// https://www.scaler.com/topics/dijkstras-algorithm-java/

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.logging.ConsoleHandler;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.Logger;

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

public class Dijkstra<T extends Enum<T>>{

    public class Vertex implements Comparable<Vertex>{
        T tag;
        double cost;
        Vertex previous=null;
        ArrayList<Edge> edges=new ArrayList<>();
        Vertex(T tag){this.tag=tag;}
        public void connect(Vertex other, double weight){
           edges.add(new Edge(this, other, weight));
        }

        public String toString(){
            return String.format("V(%s [%.2f])",tag.toString(),cost);
        }

        @Override
        public int compareTo(Vertex o) {
            return Double.compare(this.cost,o.cost);
        }

    }

    public class Edge implements Comparable<Edge>{
        Vertex a; Vertex b; Double weight;
        Edge(Vertex a, Vertex b, double weight){this.a=a;this.b=b;this.weight=weight;}

        @Override
        public int compareTo(Edge o) {
            return Double.compare(this.weight,o.weight);
        }
        public String toString(){
            return String.format("E(%s --(%f)-> %s)",a.toString(),weight,b.toString());
        }

    }

    public Logger log = Logger.getAnonymousLogger();
    HashMap<T,Vertex> graph= new HashMap<>();
    ArrayList<Vertex> unvisited = new ArrayList<>();

    public Dijkstra(){
        // log.setLevel(Level.WARNING);
        // log.addHandler(new ConsoleHandler());
        // log.setLevel(Level.ALL);
    }

    public void addNode(T name){
        log.config("Adding node " + name);
        graph.put(name, new Vertex(name));
    }
    public void addConnection(T a, T b, double cost,boolean bidirectional){
        log.config("Adding connection "+a+" "+b);
        graph.get(a).connect(graph.get(b),cost);
        if(bidirectional) addConnection(b,a,cost,false);
    }

    public Deque<T> computeCosts(T start, T end){
        log.fine("Calculating costs for transition "+start+"to"+end);

        //Reset costs
        graph.forEach((t,node)->{
            node.cost=9999;
            node.previous=null;
            unvisited.add(node);
        });
        graph.get(start).cost=0;

        while(unvisited.isEmpty()==false){
            var u = Collections.min(unvisited);
            unvisited.remove(u);
            Collections.sort(u.edges);
            for(Edge uv : u.edges){
                var v=uv.b;

                if(unvisited.contains(v)==false)continue;

                var tempDistance = u.cost+uv.weight;

                if(tempDistance < v.cost){
                    v.cost= tempDistance;
                    v.previous = u;
                }
            }
        }

        return computePath(start, end);
    }

    private Deque<T> computePath(T start, T end){
        var path = new ArrayDeque<T>();
        var last = graph.get(end);
        path.add(last.tag);
        if(last.previous!=null){
            System.out.println(last);
            graph.forEach((k,v)->System.out.println(v));
        }

        while(true){
            last = last.previous;
            if(last==null) break;
            // path.add(last.tag);
            path.addFirst(last.tag);
        }
        
        log.fine("Computed path "+path.toString());

        return path;
    }
}


