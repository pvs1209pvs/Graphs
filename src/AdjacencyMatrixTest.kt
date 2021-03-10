import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

internal class AdjacencyMatrixTest {

    private val NUM_TESTS = 11

    @Test
    fun articulationPoints(){

        val results = arrayOf(
            arrayOf(0,1),
            arrayOf(2),
            arrayOf(0,3),
            arrayOf(1,2),
            arrayOf(0),
            arrayOf(1),
            arrayOf(2),
            arrayOf(2),
            arrayOf(),
            arrayOf(1,4),
            arrayOf()
        )

        for (i in 0 until NUM_TESTS){
            assertArrayEquals( AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-$i.txt")).articulationPoints(0), results[i])
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

        for (i in 0 until NUM_TESTS) {
            val list = ArrayList<Int>()
            AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-$i.txt")).bfs(0, list)
            assertEquals(results[i], list)
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

        for (i in 0 until NUM_TESTS) {
            val list = ArrayList<Int>()
            AdjacencyMatrix(readGraph("graphs/undir-graphs/undir-graph-$i.txt")).dfs(0, list)
            assertEquals(results[i], list)
        }

    }

}