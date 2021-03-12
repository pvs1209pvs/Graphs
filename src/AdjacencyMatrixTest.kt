import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

internal class AdjacencyMatrixTest {

    @Test
    fun articulationPoints() {

        val results = arrayOf(
            arrayOf(0, 1),
            arrayOf(2),
            arrayOf(0, 3),
            arrayOf(1, 2),
            arrayOf(0),
            arrayOf(1),
            arrayOf(2),
            arrayOf(2),
            arrayOf(),
            arrayOf(1, 4),
            arrayOf()
        )

        for (i in results.indices) {
            assertArrayEquals(
                AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-$i.txt")).articulationPoints(0),
                results[i]
            )
        }

    }

    @Test
    fun eulerPath() {

        val results = listOf(
            listOf(Pair(0, 1), Pair(1, 2), Pair(2, 0), Pair(0, 3), Pair(3, 4)),
            listOf(Pair(0, 1), Pair(1, 2), Pair(2, 3)),
            listOf(),
            listOf(Pair(1, 0), Pair(0, 2), Pair(2, 1), Pair(1, 3), Pair(3, 5), Pair(5, 4), Pair(4, 1), Pair(1, 6)),
            listOf(Pair(2, 0), Pair(0, 1), Pair(1, 2), Pair(2, 3)),
            listOf(),
            listOf(Pair(3,1), Pair(1,0), Pair(0,2), Pair(2,1), Pair(1,4), Pair(4,2), Pair(2,3), Pair(3,4)),
            listOf(Pair(0,1), Pair(1,4), Pair(4,2), Pair(2,3), Pair(3,4))
        )

        for(i in results.indices){
            assertEquals(AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-${i+2}.txt")).eulerPath(), results[i])
        }

    }

    @Test
    fun bellmanFord() {

        val results = arrayOf(
            arrayOf(0, -1, 2, -2, 1),
            arrayOf(0, 1, 3, 2, 5)
        )

        for (i in results.indices) {
            assertArrayEquals(
                AdjacencyMatrix(readGraph("graphs/weighted-graphs/weighted-graph-$i.txt")).bellmanFord(0),
                results[i]
            )
        }

    }

    @Test
    fun bfs() {

        val results = listOf(
            listOf(0, 1, 5, 2, 3, 4),
            listOf(0, 1, 3, 2, 4, 5),
            listOf(0, 1, 2, 3, 4),
            listOf(0, 1, 2, 3),
            listOf(0, 1, 2, 3, 4),
            listOf(0, 1, 2, 3, 4, 6, 5),
            listOf(0, 1, 2, 3),
            listOf(0, 2, 1, 3, 4),
            listOf(0, 1, 2, 3, 4),
            listOf(0, 1, 4, 2, 3),
            listOf(0, 1, 2, 3)
        )

        for (i in results.indices) {
            assertEquals(results[i], AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-$i.txt")).bfs(0))
        }

    }


    @Test
    fun dfs() {

        val results = listOf(
            listOf(0, 1, 2, 3, 4, 5),
            listOf(0, 1, 2, 3, 4, 5),
            listOf(0, 1, 2, 3, 4),
            listOf(0, 1, 2, 3),
            listOf(0, 1, 2, 3, 4),
            listOf(0, 1, 2, 3, 5, 4, 6),
            listOf(0, 1, 2, 3),
            listOf(0, 2, 1, 3, 4),
            listOf(0, 1, 2, 3, 4),
            listOf(0, 1, 4, 2, 3),
            listOf(0, 1, 2, 3)
        )

        for (i in results.indices) {
            assertEquals(results[i], AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-$i.txt")).dfs(0))
        }

    }

}