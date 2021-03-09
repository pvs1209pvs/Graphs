import java.io.File
import java.util.stream.Collectors.toList

fun readGraph(filename: String): List<List<String>> =
    File(filename).bufferedReader().readLines().stream().map { it.split("\t") }.collect(toList())