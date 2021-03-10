fun main(args:Array<String>){


    for (i in 0 until 11){
        val g = AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-$i.txt"))
        println(g.articulationPoints(0).contentToString())

    }

    //AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-2.txt")).articulationPoints(0)

//    val list = ArrayList<Int>()
//    val a = AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-2.txt"))
//    println( a.articulationPoints(0).contentToString())

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