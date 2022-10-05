import java.io.File
import java.util.stream.Collectors.toList

fun readGraph(filename: String) = File(filename).bufferedReader().readLines().map { it.split("\t") }

