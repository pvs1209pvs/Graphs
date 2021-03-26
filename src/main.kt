fun main(args: Array<String>) {


    val g = AdjacencyMatrix(readGraph("graphs/top-graphs/top-graph-0.txt"))
    g.topsort()

}
