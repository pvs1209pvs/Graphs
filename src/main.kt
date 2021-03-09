fun main(args:Array<String>){

//    listOf(
//        AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-0.txt")),
//        AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-1.txt")),
//        AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-2.txt")),
//        AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-3.txt"))
//    ).forEach{ println(it.articulationPoints(0).contentToString())}

    val g = AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-4.txt"))
    println(g.isEulerian())

}

/*
TODO: connected components using DFS
TODO: shortest path using BFS
TODO: topsort

 */