fun main(args: Array<String>) {


    val g = AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-11.txt"))
    println(g.tarjanDFS(0).contentToString())


    /**
     * a = 0
     * b = 1
     * c = 2
     * d = 3
     * e = 4
     */

}

/*
TODO: connected components using DFS
TODO: shortest path using BFS
TODO: topsort

 */