import java.util.function.Predicate

fun main(args: Array<String>) {


//    for (i in 2..9) {
//        val g = AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-$i.txt"))
//        println(g.eulerPath())
//
//    }

    coolFun { x -> x.length == 5 }


    /**
     * a = 0
     * b = 1
     * c = 2
     * d = 3
     * e = 4
     */

}

fun coolFun(p: Predicate<String>) {
    val l =  listOf("param", "asma", "simar", "chahat").stream().filter(p).toArray()
    println(l.contentToString())
}

/*
TODO: connected components using DFS
TODO: shortest path using BFS
TODO: topsort

 */