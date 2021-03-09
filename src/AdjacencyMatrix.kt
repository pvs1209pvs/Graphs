import java.lang.StringBuilder
import kotlin.collections.ArrayDeque
import kotlin.collections.ArrayList
import kotlin.math.min

class AdjacencyMatrix(graphSrc: List<List<String>>) {

    private val graph: Array<Array<Int>> = Array(graphSrc.size) { Array(graphSrc.size) { 0 } }
    private val visited = Array(graph.size) { false }
    private var time = 1

    init {
        for (i in graphSrc.indices) {
            for (j in graphSrc.indices) {
                graph[i][j] = graphSrc[i][j].toInt()
            }
        }
    }


    fun isEulerian(): Boolean {



        var oddDegrees = 0

        for(i in graph.indices){
            if(getDegree(i) % 2 != 0){
                ++oddDegrees
            }
        }

        return oddDegrees <= 2

    }



    fun euler(start: Int) {

        val discover = Array(graph.size) { 0 }
        val low = Array(graph.size) { 0 }
        val pred = Array(graph.size) { 0 }
        val parentChild = ArrayList<Pair<Int, Int>>()

        computeLow(0, discover, low, pred, parentChild)

        val edges = getEdges()
        val cards = getCard()

        val tempGraph = Array(graph.size) { Array(graph.size) { 0 } }

        for (i in graph.indices) {
            for (j in graph.indices) {
                tempGraph[i][j] = graph[i][j]
            }
        }

        eulerPath(tempGraph, start, edges, cards, low, discover)

    }

    private fun eulerPath(tempGraph: Array<Array<Int>>, start: Int, numEdges: ArrayList<Pair<Int, Int>>, cards: ArrayList<Int>, low: Array<Int>, discover: Array<Int>) {

        if (numEdges.size == 0) return

        val neighbors = tempGraph[start]

        for (i in neighbors.indices) {
            if (tempGraph[start][i] != 0 && (low[i] <= discover[start] || cards[start]==1)) {
                println("$start,$i")
                numEdges.remove(Pair(start, i))
                numEdges.remove(Pair(i, start))
                tempGraph[start][i] = 0
                tempGraph[i][start] = 0
                --cards[start]
                --cards[i]
                eulerPath(tempGraph, i, numEdges, cards, low, discover)
            }
        }

    }

    /**
     * Returns all the articulation points. Root is not included as one of the articulation points. Todo: look into it.
     * @param start Root vertex
     * @return All the articulation points.
     */
    fun articulationPoints(start: Int): Array<Int> {

        val discover = Array(graph.size) { 0 }
        val low = Array(graph.size) { 0 }
        val pred = Array(graph.size) { 0 }
        val parentChild = ArrayList<Pair<Int, Int>>()

        computeLow(start, discover, low, pred, parentChild)

        return parentChild
            .stream()
            .filter { x -> low[x.second] >= discover[x.first] && x.first != start }
            .map { y -> y.first }
            .distinct()
            .toArray { z -> arrayOfNulls<Int>(z) }

    }

    private fun computeLow(
        v: Int, discover: Array<Int>,
        low: Array<Int>,
        pred: Array<Int>,
        parentChild: ArrayList<Pair<Int, Int>>
    ) {

        visited[v] = true

        low[v] = time
        discover[v] = time

        val neighbors = ArrayList<Int>()
        for (i in graph.indices) {
            if (graph[v][i] == 1) {
                neighbors.add(i)
            }
        }

        for (w in neighbors) {
            if (!visited[w]) {
                parentChild.add(Pair(v, w))
                pred[w] = v
                ++time
                computeLow(w, discover, low, pred, parentChild)
                low[v] = min(low[v], low[w])
            } else if (w != pred[v]) {
                low[v] = min(low[v], discover[w])
            }
        }

    }


    fun bellmanFord(start: Int) {

        val vertices = Array(graph.size) { Int.MAX_VALUE }
        vertices[start] = 0

        for (i in 0 until graph.size - 1) {
            relaxation(vertices)
        }

    }


    private fun relaxation(vertices: Array<Int>) {

        for (i in graph.indices) {
            for (j in graph.indices) {
                if (graph[i][j] != 0) {
                    if (vertices[i] + graph[i][j] < vertices[j]) {
                        vertices[j] = vertices[i] + graph[i][j]
                    }
                }
            }
        }

    }

    /**
     * Only DAGs can have topological ordering.
     * Tarjan's Strongly Connected components can used to find cycles.
     * Trees always have a topological sorts because trees dont have cycles.
     */
    fun topSort() {

        val ordering = ArrayList<Int>(graph.size)

        for (at in graph.indices) {
            if (!visited[at]) {
                val explored = ArrayList<Int>()
                dfs(at, explored)
                for (nodeId in explored) {
                    ordering.add(nodeId)
                }
            }
        }

        println(ordering)

    }


    private fun dfs(at: Int, explored: ArrayList<Int>) {

        visited[at] = true

        val edges = graph[at]

        for (i in edges.indices) {
            if (edges[i] == 1 && !visited[i]) {
                dfs(i, explored)
            }
            explored.add(i)
        }

    }

    /**
     * Useful for finding shortest path in an unweighted graph.
     */
    fun bfs(start: Int) {

        val q = ArrayDeque<Int>()

        q.addLast(start)
        visited[start] = true

        while (q.isNotEmpty()) {
            val neighbors = graph[q.removeFirst()]

            for (i in neighbors.indices) {
                if (neighbors[i] == 1 && !visited[i]) {
                    q.addLast(i)
                    visited[i] = true
                }
            }
        }

    }


    fun dfs(at: Int) {

        if (visited[at]) return
        visited[at] = true

        val neighbours = graph[at]
        for (i in neighbours.indices) {
            if (neighbours[i] == 1 && !visited[i]) {
                dfs(i)
            }
        }

    }


    private fun getDegree(at: Int): Int = graph[at].filter { it == 1 }.count()


    private fun getCard(): ArrayList<Int> {

        val cards = ArrayList<Int>()

        for (i in graph.indices) {
            var card = 0
            for (j in graph.indices) {
                if (graph[i][j] != 0) {
                    ++card
                }
            }
            cards.add(card)
        }

        return cards

    }


    private fun getEdges(): ArrayList<Pair<Int, Int>> {

        val edges = ArrayList<Pair<Int, Int>>()

        for (i in graph.indices) {
            for (j in graph.indices) {
                if (graph[i][j] != 0) {
                    edges.add(Pair(i, j))
                }
            }
        }

        return edges

    }


    override fun toString(): String {

        val s = StringBuilder()

        for (i in graph.indices) {
            s.append("$i -> ")
            for (j in graph.indices) {
                if (graph[i][j] == 1) {
                    s.append("$j, ")
                }
            }
            s.append("\n")
        }

        return s.toString()
    }

}