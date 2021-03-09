import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

internal class AdjacencyMatrixTest {

    @Test
    fun articulationPointsTest() {

        val results = arrayOf(
            arrayOf(1),
            arrayOf(2),
            arrayOf(3),
            arrayOf(1, 2),
            arrayOf(),
            arrayOf(1),
        )

        for ( i in results.indices){
            println(results[i].contentToString())
        }

        assertArrayEquals(AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-0.txt")).articulationPoints(0), results[0])



        listOf(
            AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-0.txt")),
            AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-1.txt")),
            AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-2.txt")),
            AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-3.txt")),
            AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-4.txt")),
            AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-5.txt")),
        )
            //.forEach { println(it.articulationPoints(0).contentToString()) }
    }

}