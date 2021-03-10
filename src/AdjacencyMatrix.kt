import java.lang.StringBuilder
import java.util.stream.Collectors.toList
import kotlin.collections.ArrayDeque
import kotlin.collections.ArrayList
import kotlin.math.min

class AdjacencyMatrix(graphSrc: List<List<String>>) {

    private val graph: Array<Array<Int>> = Array(graphSrc.size) { Array(graphSrc.size) { 0 } }
    private var visited = Array(graph.size) { false }

    private var time = 1
    var callOnRoot = 0

    init {
        for (i in graphSrc.indices) {
            for (j in graphSrc.indices) {
                graph[i][j] = graphSrc[i][j].toInt()
            }
        }
    }


    fun floydWarshall(): Array<Array<Int>> {

        val tempGraph = Array(graph.size) { Array(graph.size) { Int.MAX_VALUE } }

        for (i in graph.indices) {
            for (j in graph[i].indices) {
                if (graph[i][j] != 0) tempGraph[i][j] = graph[i][j]
                if (i == j) {
                    tempGraph[i][j] = 0
                }
            }
        }


        fun floydWarshall(v: Int, graph: Array<Array<Int>>) {
            for (i in graph.indices) {
                for (j in graph.indices) {
                    if (!(i == v || j == v || graph[i][v] == Int.MAX_VALUE || graph[v][j] == Int.MAX_VALUE)) {
                        val temp = graph[i][v] + graph[v][j]
                        if (temp < graph[i][j]) {
                            graph[i][j] = temp
                        }
                    }
                }
            }
        }

        graph.indices.forEach { i -> floydWarshall(i, tempGraph) }

        return tempGraph

    }


    /**
     * Finds the first local sink.
     * @return Id of the first local sink. -1 if no sink exists.
     */
    fun sink(): Int {

        var i = 0
        var j = 1

        while (j < graph.size) {
            if (graph[i][j] == 1) {
                ++i
            } else if (graph[i][j] == 0) {
                ++j
            }
        }

        return if (graph[i].any { it == 1 }) -1 else i

    }


    fun eulerPath(): ArrayList<Pair<Int, Int>> {

        val discover = Array(graph.size) { 0 }
        val low = Array(graph.size) { 0 }
        val pred = Array(graph.size) { 0 }
        val parentChild = ArrayList<Pair<Int, Int>>()
        computeLow(0, discover, low, pred, parentChild)

        val tempGraph = Array(graph.size) { Array(graph.size) { 0 } }
        for (i in graph.indices) {
            for (j in graph.indices) {
                tempGraph[i][j] = graph[i][j]
            }
        }

        val eulerianPath = ArrayList<Pair<Int, Int>>()

        if (isEulerian()) {
            eulerPath(tempGraph, eulerStartVertex(), getEdges(), getCard(), low, discover, eulerianPath)
        }

        return eulerianPath

    }

    private fun eulerStartVertex(): Int = getCard().indexOf(getCard().find { it % 2 != 0 })


    private fun eulerPath(
        tempGraph: Array<Array<Int>>,
        start: Int,
        numEdges: ArrayList<Pair<Int, Int>>,
        cards: ArrayList<Int>,
        low: Array<Int>,
        discover: Array<Int>,
        path: ArrayList<Pair<Int, Int>>
    ) {

        if (numEdges.size == 0) return

        val neighbors = tempGraph[start]

        for (i in neighbors.indices) {
            if (tempGraph[start][i] != 0 && (low[i] <= discover[start] || cards[start] == 1)) {
                path.add(Pair(start, i))
                numEdges.remove(Pair(start, i))
                numEdges.remove(Pair(i, start))
                tempGraph[start][i] = 0
                tempGraph[i][start] = 0
                --cards[start]
                --cards[i]
                eulerPath(tempGraph, i, numEdges, cards, low, discover, path)
            }
        }

    }


    private fun isEulerian(): Boolean = graph.indices.count { getDegree(it) % 2 != 0 } <= 2


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
        visited = Array(graph.size) { false }

        isRootArticulation(start, start)

        val points = parentChild
            .stream()
            .filter { x -> low[x.second] >= discover[x.first] && x.first != start }
            .map { y -> y.first }
            .distinct()
            .collect(toList())

        if (callOnRoot >= 2) {
            points.add(start)
        }

        return points.toTypedArray().sortedArray()

    }

    private fun isRootArticulation(at: Int, root: Int) {

        if (visited[at]) return
        visited[at] = true

        val neighbours = graph[at]
        for (i in neighbours.indices) {
            if (neighbours[i] == 1 && !visited[i]) {
                if (at == root) {
                    ++callOnRoot
                }
                isRootArticulation(i, root)
            }
        }

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
     * Useful for finding shortest path in an unweighted graph.
     */
    fun bfs(start: Int, list: ArrayList<Int>) {

        list.add(start)

        val q = ArrayDeque<Int>()

        q.addLast(start)
        visited[start] = true

        while (q.isNotEmpty()) {
            val neighbors = graph[q.removeFirst()]

            for (i in neighbors.indices) {
                if (neighbors[i] == 1 && !visited[i]) {
                    q.addLast(i)
                    visited[i] = true
                    list.add(i)
                }
            }
        }

    }


    fun dfs(at: Int, list: ArrayList<Int>) {

        if (visited[at]) return
        visited[at] = true

        list.add(at)

        val neighbours = graph[at]
        for (i in neighbours.indices) {
            if (neighbours[i] == 1 && !visited[i]) {
                if (at == 0) {
                    callOnRoot++
                }
                dfs(i, list)
            }
        }

    }


    private fun getDegree(at: Int): Int = graph[at].filter { it == 1 }.count()


    private fun getCard(): ArrayList<Int> {

        val cards = ArrayList<Int>()

        for (i in graph.indices) {
            val card = graph.indices.count { graph[i][it] != 0 }
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

/**
 * DFS
 * BFS
 * Bellman Ford
 * Articulation Points: add root check
 * Eulerian Path: add circuit
 * Sink
 * Floyd Warshall: test
 */