fun main(args: Array<String>) {

    val g = AdjacencyMatrix(readGraph("src/main/resources/graphs/eulerian-path/ep-3.txt"))
    println(g)

    g.epPath()
}
