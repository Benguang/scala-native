package bounce

import som.Random

object Bounce extends benchmarks.Benchmark[Int] {
  private class Ball(random: Random) {
    private var x: Int = random.next() % 500
    private var y: Int = random.next() % 500
    private var xVel: Int = (random.next() % 300) - 150
    private var yVel: Int = (random.next() % 300) - 150

    def bounce(): Boolean = {
      val xLimit: Int = 500
      val yLimit: Int = 500
      var bounced = false

      x += xVel;
      y += yVel;
      if (x > xLimit) { x = xLimit; xVel = 0 - Math.abs(xVel); bounced = true; }
      if (x < 0)      { x = 0;      xVel = Math.abs(xVel);     bounced = true; }
      if (y > yLimit) { y = yLimit; yVel = 0 - Math.abs(yVel); bounced = true; }
      if (y < 0)      { y = 0;      yVel = Math.abs(yVel);     bounced = true; }

      bounced
    }
  }

  def run(): Int = {
    val random = new Random()

    val ballCount = 100
    var bounces   = 0
    val balls     = Array.fill(ballCount)(new Ball(random))

    (0 to 49).foreach { i =>
      balls.foreach { ball =>
        if (ball.bounce()) {
          bounces += 1
        }
      }
    }

    bounces
  }

  def check(result: Int): Boolean =
    result == 1331
}
