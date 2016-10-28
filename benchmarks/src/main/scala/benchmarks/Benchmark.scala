package benchmarks

abstract class Benchmark[T] {
  def run(): T
  def check(t: T): Boolean
}
