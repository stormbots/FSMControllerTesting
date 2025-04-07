package frc.robot;
// https://www.scaler.com/topics/dijkstras-algorithm-java/

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.PriorityQueue;
import java.util.Stack;

import edu.wpi.first.wpilibj2.command.Subsystem;


// public class DJ<T extends Enum<T>> {
public class DJScaler<T extends Enum<T>> {
    public class Vertex implements Comparable<Vertex>{
        T tag;
        double cost;
        Vertex previous=null;
        ArrayList<Pair> edges=new ArrayList<>();
        Vertex(T tag){this.tag=tag;}
        public void connect(Vertex other, double weight){
           edges.add(new Pair(this, other, weight));
        }
        public String toString(){
            return String.format("V(%s [%.2f])",tag.toString(),cost);
        }

        @Override
        public int compareTo(Vertex o) {
            return Double.compare(this.cost,o.cost);
        }

    }

    public class Pair implements Comparable<Pair>{
        Vertex a; Vertex b; Double weight;
        Pair(Vertex a, Vertex b, double weight){this.a=a;this.b=b;this.weight=weight;}

        @Override
        public int compareTo(Pair o) {
            return Double.compare(this.weight,o.weight);
        }
        public String toString(){
            return String.format("E(%s --(%f)-> %s)",a.toString(),weight,b.toString());
        }

    }

    HashMap<T,Vertex> graph= new HashMap<>();

    ArrayList<Vertex> unvisited = new ArrayList<>();

    public void addNode(T name){
        graph.put(name, new Vertex(name));
    }
    public void addConnection(T a, T b, double cost,boolean bidirectional){
        graph.get(a).connect(graph.get(b),cost);
        if(bidirectional) addConnection(b,a,cost,false);
    }

    public Collection<Vertex> computeCosts(T start, T end){
        System.out.println("STARTING PATHS");
        System.out.println(start);
        System.out.println(end);
        System.out.println("---------");

        //Reset costs
        graph.forEach((t,node)->{
            node.cost=999;
            node.previous=null;
            unvisited.add(node);
        });
        graph.get(start).cost=0;
        // var test = graph.get(start);
        // System.out.println("EDGES");
        // graph.values().forEach((v)->v.edges.forEach(System.out::println));
        // System.out.println("---------");
        // System.out.println("UNVISITED");
        // unvisited.forEach(System.out::println);
        // System.out.println("---------");

        // Collections.sort(unvisited);
        // Collections.min(unvisited);
        while(unvisited.isEmpty()==false){
            // var u = unvisited.remove(0);
            var u = Collections.min(unvisited);
            unvisited.remove(u);

            System.out.printf("Active node: %s\n",u.toString());

            Collections.sort(u.edges);
            u.edges.forEach(System.out::println);

            for(Pair uv: u.edges){ //next node, ideally lowest cost
                var v=uv.b;
                if(unvisited.contains(v)==false)continue;
                System.out.println("checking "+uv);

                var tempDistance = u.cost+uv.weight;
                if(tempDistance>=1000){
                    System.out.println("Err, started at wrong element");
                }
                if(tempDistance < v.cost){
                    v.cost= tempDistance;
                    v.previous = u;
                }
            }
        }

        System.out.println("COMPLETED GRAPH: "+ start);
        graph.values().forEach(System.out::println);
        System.out.println("---------");

        //Graph weights are now structured such that they contain the weights from Start to other points
        //Backtrack from End to Start following the lowest cost, and return the list (reversed)
        System.out.println("Building target path");
        var path = new Stack<Vertex>();
        path.add(graph.get(end));

        for(var i=0;i<10;i++){//TODO for loop with proper checking
            var last=path.peek();
            // if(last==null) break;
            if(last==graph.get(start)) break;
            path.add(last.previous);
        }
        path.forEach(System.out::println);
        return path;
    }


// function dijkstra(G, S)
//     //G=graph, S=source 
//     for each vertex v in G
//         //the initial path is set to infinite
//         distance[v] <- infinite
//         previous[v] <- NULL
//         If v != S, add v to Priority Queue PQ
//     distance[S] <- 0
	
//     while PQ IS NOT EMPTY
//         u <- Extract MIN from PQ
//         for each unvisited neighbor v of u
//             tempDistance <- distance[u] + edge_weight(u, v)
//             if tempDistance < distance[v]
//                 distance[v] <- tempDistance
//                 previous[v] <- u
//     return distance[], previous[]

}


