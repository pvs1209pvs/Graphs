import java.lang.StringBuilder
import java.util.*
import java.util.stream.Collectors.toList
import kotlin.collections.ArrayDeque
import kotlin.collections.ArrayList
import kotlin.collections.HashSet
import kotlin.math.min

class AdjacencyMatrix(graphSrc: List<List<String>>) {

    private val graph: Array<Array<Int>> = Array(graphSrc.size) { Array(graphSrc.size) { 0 } }

    private class CallOnRoot(var value: Int)
    private class Time(var value: Int)

    init {
        for (i in graphSrc.indices) {
            for (j in graphSrc.indices) {
                graph[i][j] = graphSrc[i][j].toInt()
            }
        }
    }


    /**
     * Finds the topological ordering of the graph.
     * @return Topological ordering.
     */
    fun topsort(): Array<Int> {

        val topOrder = ArrayList<Int>(graph.size)
        val inDegs = Array(graph.size) { getInDeg(it) }
        val q = ArrayDeque<Int>()

        findZeroInDeg(inDegs, topOrder).forEach { q.addLast(it) }

        while (q.isNotEmpty()) {

            val u = q.removeFirst()
            topOrder.add(u)

            val neighbor = getNeighbor(u)

            neighbor.forEach { --inDegs[it] }

            neighbor.filter { inDegs[it] == 0 }.forEach { q.add(it) }

        }

        return topOrder.toTypedArray()

    }

    /**
     * Returns the vertices with zero in-deg.
     * @param degs In-degs of all the vertices.
     * @param visited Vertices already visited.
     * @return Array of the vertices with zero in-deg.
     */
    private fun findZeroInDeg(degs: Array<Int>, visited: List<Int>) =
        degs.indices.filter { degs[it] == 0 && !visited.contains(it) }.toTypedArray()

    fun isCyclic(graphType: Char): Set<Pair<Int, Int>> {

        fun dfs(at: Int, visited: Array<Boolean>, recStack: Array<Boolean>, cyclicEdges: MutableSet<Pair<Int, Int>>) {

            if (visited[at]) return
            visited[at] = true

            recStack[at] = true

            val neighbours = graph[at]
            for (i in neighbours.indices) {

                if (neighbours[i] == 1) {
                    if (visited[i] && recStack[i]) {
                        cyclicEdges.add(Pair(at, i))
                    }
                    if (!visited[i]) {
                        dfs(i, visited, recStack, cyclicEdges)
                    }
                }
            }

            recStack[at] = false

        }

        fun dfs(at: Int, visited: Array<Boolean>, parent: Int, cyclicEdges: MutableSet<Pair<Int, Int>>) {

            if (visited[at]) return
            visited[at] = true

            val neighbours = graph[at]
            for (i in neighbours.indices) {

                if (neighbours[i] == 1) {

                    if (visited[i] && i != parent) {
                        println("$at $i")
                    }

                    if (!visited[i]) {
                        dfs(i, visited, at, cyclicEdges)
                    }
                }
            }

        }

        val visited = Array(graph.size) { false }
        val recStack = Array(graph.size) { false }
        val cyclicEdges = HashSet<Pair<Int, Int>>()

        when (graphType) {
            'd' -> dfs(0, visited, recStack, cyclicEdges)
            'u' -> dfs(0, visited, -1, cyclicEdges)
        }

        return cyclicEdges

    }

    /**
     * Returns the number of paths you can take from one vertex to the other in p number of hops.
     * @param p Number of hops from one vertex to the other.
     * @return Adjacency matrix containing the number of paths from one vertex to the other.
     */
    fun inMove(p: Int): Array<Array<Int>> {

        val tmp = Array(graph.size) { Array(graph.size) { 0 } }
        val m = Array(graph.size) { Array(graph.size) { 0 } }
        val r = Array(graph.size) { Array(graph.size) { 0 } }
        for (i in graph.indices) {
            for (j in graph.indices) {
                m[i][j] = graph[i][j]
                r[i][j] = graph[i][j]
            }
        }

        var sum = 0
        for (i in 0 until p - 1) {
            for (b in graph.indices) {
                for (d in graph.indices) {
                    for (k in graph.indices) {
                        sum += r[b][k] * m[k][d]
                    }
                    tmp[b][d] = sum
                    sum = 0
                }
            }
            for (b in graph.indices) {
                for (d in graph.indices) {
                    r[b][d] = tmp[b][d]
                }
            }
        }

        return r
    }

    /**
     * Finds the edge that cannot divide the graph in different components.
     * @param at Starting vertex.
     * @return All the critical edges.
     */
    fun tarjan(at: Int): Array<Pair<Int, Int>> {

        val visited = Array(graph.size) { false }
        val time = Time(1)
        val discover = Array(graph.size) { 0 }
        val low = Array(graph.size) { 0 }
        val pred = Array(graph.size) { 0 }
        val parentChild = ArrayList<Pair<Int, Int>>()

        computeLow(at, visited, time, discover, low, pred, parentChild)

        return parentChild.filter { low[it.first] < low[it.second] }.toTypedArray()

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

    /**
     * Returns the Eulerian path.
     * @return Eulerian path.
     */
    fun eulerPath(): List<Pair<Int, Int>> {

        val visited = Array(graph.size) { false }
        val discover = Array(graph.size) { 0 }
        val low = Array(graph.size) { 0 }
        val pred = Array(graph.size) { 0 }
        val parentChild = ArrayList<Pair<Int, Int>>()
        val time = Time(1)
        computeLow(0, visited, time, discover, low, pred, parentChild)

        val tempGraph = Array(graph.size) { Array(graph.size) { 0 } }
        for (i in graph.indices) {
            for (j in graph.indices) {
                tempGraph[i][j] = graph[i][j]
            }
        }

        val eulerianPath = ArrayList<Pair<Int, Int>>()

        val startVertex = eulerStartVertex()
        if (isEulerian() && startVertex >= 0) {
            eulerPath(
                tempGraph, startVertex, getEdges(),
                getDegree(), low, discover, eulerianPath
            )
        }

        return eulerianPath

    }

    /**
     * Returns the starting vertex for euler path.
     * @return Starting vertex for eulerian path.
     */
    private fun eulerStartVertex() = getDegree().indexOf(getDegree().find { it % 2 != 0 })

    /**
     * Finds the eulerian path.
     * @param tempGraph Temporary graph.
     * @param start Starting vertex for eulerian path.
     * @param edges All the edges of the graph.
     * @param degrees Degrees of all the vertices.
     * @param low Low numbers for vertices.
     * @param discover Discovery number for vertices.
     * @param path Eulerian path
     */
    private fun eulerPath(
        tempGraph: Array<Array<Int>>,
        start: Int,
        edges: MutableList<Pair<Int, Int>>,
        degrees: Array<Int>,
        low: Array<Int>,
        discover: Array<Int>,
        path: MutableList<Pair<Int, Int>>
    ) {

        if (edges.size == 0) return

        val neighbors = tempGraph[start]

        for (i in neighbors.indices) {
            if (tempGraph[start][i] != 0 && (low[i] <= discover[start] || degrees[start] == 1)) {
                path.add(Pair(start, i))
                edges.remove(Pair(start, i))
                edges.remove(Pair(i, start))
                tempGraph[start][i] = 0
                tempGraph[i][start] = 0
                --degrees[start]
                --degrees[i]
                eulerPath(tempGraph, i, edges, degrees, low, discover, path)
            }
        }

    }

    /**
     * Checks is the graph is eulerian.
     * @return True if the graph is eulerian else false.
     */
    private fun isEulerian(): Boolean = graph.indices.count { getDegree(it) % 2 != 0 } <= 2

    /**
     * Returns all the articulation points. Root is not included as one of the articulation points. Todo: look into it.
     * @param start Root vertex
     * @return All the articulation points.
     */
    fun articulationPoints(start: Int): Array<Int> {

        var visited = Array(graph.size) { false }
        val time = Time(1)
        val discover = Array(graph.size) { 0 }
        val low = Array(graph.size) { 0 }
        val pred = Array(graph.size) { 0 }
        val parentChild = LinkedList<Pair<Int, Int>>()
        computeLow(start, visited, time, discover, low, pred, parentChild)

        visited = Array(graph.size) { false }
        val callOnRoot = CallOnRoot(0)
        isRootArticulation(start, start, visited, callOnRoot)

        val points = parentChild
            .stream()
            .filter { x -> low[x.second] >= discover[x.first] && x.first != start }
            .map { y -> y.first }
            .distinct()
            .collect(toList())

        if (callOnRoot.value >= 2) {
            points.add(start)
        }

        return points.toTypedArray().sortedArray()

    }

    /**
     * Checks if root is an articulation point.
     * @param at Starting vertex.
     * @param root Root vertex.
     */
    private fun isRootArticulation(at: Int, root: Int, visited: Array<Boolean>, callOnRoot: CallOnRoot) {

        if (visited[at]) return
        visited[at] = true

        val neighbours = graph[at]
        for (i in neighbours.indices) {
            if (neighbours[i] == 1 && !visited[i]) {
                if (at == root) {
                    ++callOnRoot.value
                }
                isRootArticulation(i, root, visited, callOnRoot)
            }
        }

    }

    /**
     * Computes the discovery and low number for the vertices in the graph.
     * @param v Starting node.
     * @param discover Discovery numbers for vertices.
     * @param low Low numbers for vertices.
     * @param pred Predecessor of the current vertex being explored.
     * @param parentChild Parent-child relation of the DFS tree.
     */
    private fun computeLow(
        v: Int,
        visited: Array<Boolean>,
        time: Time,
        discover: Array<Int>,
        low: Array<Int>,
        pred: Array<Int>,
        parentChild: MutableList<Pair<Int, Int>>
    ) {

        visited[v] = true

        low[v] = time.value
        discover[v] = time.value

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
                ++time.value
                computeLow(w, visited, time, discover, low, pred, parentChild)
                low[v] = min(low[v], low[w])
            } else if (w != pred[v]) {
                low[v] = min(low[v], discover[w])
            }
        }

    }

    /**
     * Returns the shortest path form the starting vertex to every other vertex.
     * Also works for negative edges.
     * @param start Starting vertex.
     * @return List containing the shortest path from the starting to other vertices.
     */
    fun bellmanFord(start: Int): Array<Int> {

        val distToVertices = Array(graph.size) { Int.MAX_VALUE }
        distToVertices[start] = 0

        for (i in 0 until graph.size - 1) {
            relaxation(distToVertices)
        }

        return distToVertices

    }

    /**
     * Updates the distance to the vertex if a shorted path is found.
     * @param vertices Shortest path to other vertices.
     */
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
     * Returns the BFS search.
     * @param at Starting vertex.
     * @return BFS search order.
     */
    fun bfs(at: Int): List<Int> {

        /**
         * Returns the BFS path of the graph.
         * Useful for finding shortest path in an unweighted graph.
         * @param at Staring vertex.
         * @param List used to store the BFS path.
         */
        fun bfs(at: Int, visited: Array<Boolean>, list: MutableList<Int>) {

            list.add(at)

            val q = ArrayDeque<Int>()

            q.addLast(at)
            visited[at] = true

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

        val visited = Array(graph.size) { false }
        val path = ArrayList<Int>()

        bfs(at, visited, path)

        return path

    }


    /**
     * Returns the DFS search.
     * @param at Starting vertex.
     * @return DFS search order.
     */
    fun dfs(at: Int): List<Int> {

        /**
         * Returns the depth first path of the tree.
         * @param at Staring vertex for DFS.
         * @param list List used to store the DFS path.
         */
        fun dfs(at: Int, visited: Array<Boolean>, list: MutableList<Int>) {

            if (visited[at]) return
            visited[at] = true

            list.add(at)

            val neighbours = graph[at]
            for (i in neighbours.indices) {
                if (neighbours[i] == 1 && !visited[i]) {
                    dfs(i, visited, list)
                }
            }

        }

        val visited = Array(graph.size) { false }
        val path = ArrayList<Int>()

        dfs(at, visited, path)

        return path

    }

    /**
     * Returns the degree of the vertex in an undirected graph.
     * @param vertex Vertex whose degree needs to be calculated.
     * @return Degree of the vertex.
     */
    private fun getDegree(vertex: Int) = graph[vertex].count { it != 0 }

    /**
     * Returns the degree of all the vertices in an undirected graph.
     * @return ArrayList containing cardinality of each vertex.
     */
    private fun getDegree(): Array<Int> {

        val degrees = Array(graph.size) { 0 }

        for (i in degrees.indices) {
            degrees[i] = getDegree(i)
        }

        return degrees

    }

    /**
     * Out-degree of the vertex.
     * @param vertex Vertex whose out-degree needs to be calculated.
     * @return Out-degree of the vertex.
     */
    private fun getOutDeg(vertex: Int) = graph[vertex].count { it != 0 }

    /**
     * In-degree of the vertex.
     * @param vertex Vertex whose in-degree needs to be calculated.
     * @return In-degree of the vertex.
     */
    private fun getInDeg(vertex: Int) = graph.indices.count { graph[it][vertex] != 0 }

    /**
     * Neighbors of a vertex.
     * @param vertex Vertex whose neighbors needs to be found.
     * @return Array of all the neighbors.
     */
    private fun getNeighbor(vertex: Int) = graph.indices.filter { graph[vertex][it] != 0 }.toIntArray()

    /**
     * Returns all the edges in a graph.
     * @return Edges in a graph as an List of Pair.
     */
    private fun getEdges(): MutableList<Pair<Int, Int>> {

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

    /**
     * Connects two vertices.
     * @param from Source vertex.
     * @param to Destination vertex.
     */
    fun addEdge(from: Int, to: Int) {
        graph[from][to] = 1
    }

    /**
     * Connects two vertices.
     * @param from Source vertex.
     * @param to Destination vertex.
     * @param weight Weight of the edge.
     */
    fun addEdge(from: Int, to: Int, weight: Int) {
        graph[from][to] = weight
    }

    /**
     * Removes all the edges.
     */
    fun reset() {
        for (i in graph.indices) {
            for (j in graph.indices) {
                graph[i][j] = 0
            }
        }
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
            if (i != graph.size - 1) {
                s.append("\n")
            }
        }

        return s.toString()
    }

}

/**
 * DFS
 * BFS
 * Bellman Ford
 * Articulation Points
 * Tarjan's Algorithm
 * Sink
 * InMove
 * TODO: Floyd Warshall: test
 * TODO: Eulerian Path: add circuit, test
 * TODO: IsCyclic: test
 * TODO: Topological Sort: test
 */