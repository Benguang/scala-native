package cd

import java.lang.Boolean.{TRUE, FALSE}
import som.Vector

final class CD extends benchmarks.Benchmark[Nothing] {
  def benchmark(numAircrafts: Int): Int = {
    val numFrames = 200
    val simulator = new Simulator(numAircrafts);
    val detector = new CollisionDetector();
    var actualCollisions = 0

    (0 until numFrames).map { i =>
      val time = i / 10.0
      val collisions = detector.handleNewFrame(simulator.simulate(time))
      actualCollisions += collisions.size()
    }

    actualCollisions
  }

  def loop(iterations: Int): Boolean =
    check(benchmark(iterations), iterations)

  def check(actualCollisions: Int, numAircrafts: Int): Boolean = {
    if (numAircrafts == 1000) { return actualCollisions == 14484 }
    if (numAircrafts ==  500) { return actualCollisions == 14484 }
    if (numAircrafts ==  250) { return actualCollisions == 10830 }
    if (numAircrafts ==  100) { return actualCollisions ==  4305 }
    if (numAircrafts ==   10) { return actualCollisions ==   390 }

    System.out.println("No verification result for " + numAircrafts + " found")
    System.out.println("Result is: " + actualCollisions)

    return false
  }

  def run(): Nothing =
    throw new RuntimeException("Should never be reached");

  def check(result: Nothing): Boolean =
    throw new RuntimeException("Should never be reached");
}

final class Aircraft(val callsign: CallSign, val position: Vector3D)

final class CallSign(val value: Int) extends Comparable[CallSign] {
  override def compareTo(other: CallSign) =
    if (value == other.value) 0
    else if (value < other.value) -1
    else 1
}

final class Collision(val aircraftA: CallSign,
                      val aircraftB: CallSign,
                      val position: Vector3D)

final class CollisionDetector {
  import CollisionDetector._

  private val state = new RedBlackTree[CallSign, Vector3D]()

  def handleNewFrame(frame: Vector[Aircraft]): Vector[Collision] = {
    val motions = new Vector[Motion]()
    val seen = new RedBlackTree[CallSign, Boolean]()

    frame.forEach { aircraft =>
      var oldPosition = state.put(aircraft.callsign, aircraft.position)
      var newPosition = aircraft.position
      seen.put(aircraft.callsign, true)

      if (oldPosition == null) {
        // Treat newly introduced aircraft as if they were stationary.
        oldPosition = newPosition
      }

      motions.append(new Motion(aircraft.callsign, oldPosition, newPosition))
    }

    // Remove aircraft that are no longer present.
    val toRemove = new Vector[CallSign]();
    state.forEach { e =>
      if (!seen.get(e.key)) {
        toRemove.append(e.key);
      }
    }

    toRemove.forEach { e =>
      state.remove(e)
    }

    val allReduced = reduceCollisionSet(motions);
    val collisions = new Vector[Collision]();
    allReduced.forEach { reduced =>
      (0 until reduced.size()).foreach { i =>
        val motion1 = reduced.at(i)
        ((i + 1) until reduced.size()).foreach { j =>
          val motion2 = reduced.at(j)
          val collision = motion1.findIntersection(motion2)
          if (collision != null) {
            collisions.append(new Collision(motion1.callsign, motion2.callsign, collision))
          }
        }
      }
    }

    collisions
  }



}

object CollisionDetector {
  val horizontal = new Vector2D(Constants.GOOD_VOXEL_SIZE, 0.0);
  val vertical   = new Vector2D(0.0, Constants.GOOD_VOXEL_SIZE);

  def isInVoxel(voxel: Vector2D , motion: Motion): Boolean = {
    if (voxel.x > Constants.MAX_X ||
        voxel.x < Constants.MIN_X ||
        voxel.y > Constants.MAX_Y ||
        voxel.y < Constants.MIN_Y) {
      return false;
    }

    val init = motion.posOne
    val fin  = motion.posTwo

    val v_s = Constants.GOOD_VOXEL_SIZE;
    val r   = Constants.PROXIMITY_RADIUS / 2.0;

    val v_x = voxel.x
    val x0 = init.x
    val xv = fin.x - init.x

    val v_y = voxel.y
    val y0 = init.y
    val yv = fin.y - init.y

    var low_x = 0.0d
    var high_x = 0.0d
    low_x = (v_x - r - x0) / xv
    high_x = (v_x + v_s + r - x0) / xv

    if (xv < 0.0) {
      val tmp = low_x
      low_x = high_x
      high_x = tmp
    }

    var low_y = 0.0d
    var high_y = 0.0d
    low_y  = (v_y - r - y0) / yv
    high_y = (v_y + v_s + r - y0) / yv

    if (yv < 0.0) {
      val tmp = low_y
      low_y = high_y
      high_y = tmp
    }

    (((xv == 0.0 && v_x <= x0 + r && x0 - r <= v_x + v_s) /* no motion in x */ ||
      (low_x <= 1.0 && 1.0 <= high_x) || (low_x <= 0.0 && 0.0 <= high_x) ||
       (0.0 <= low_x && high_x <= 1.0)) &&
     ((yv == 0.0 && v_y <= y0 + r && y0 - r <= v_y + v_s) /* no motion in y */ ||
      ((low_y <= 1.0 && 1.0 <= high_y) || (low_y <= 0.0 && 0.0 <= high_y) ||
       (0.0 <= low_y && high_y <= 1.0))) &&
     (xv == 0.0 || yv == 0.0 || /* no motion in x or y or both */
      (low_y <= high_x && high_x <= high_y) ||
      (low_y <= low_x && low_x <= high_y) ||
      (low_x <= low_y && high_y <= high_x)))
  }

  def putIntoMap(
      voxelMap: RedBlackTree[Vector2D, Vector[Motion]],
      voxel: Vector2D,
      motion: Motion): Unit = {
    var array = voxelMap.get(voxel)
    if (array == null) {
      array = new Vector()
      voxelMap.put(voxel, array)
    }
    array.append(motion)
  }

  def recurse(
      voxelMap: RedBlackTree[Vector2D, Vector[Motion]] ,
      seen: RedBlackTree[Vector2D, Boolean] ,
      nextVoxel: Vector2D ,
      motion: Motion): Unit = {
    if (!isInVoxel(nextVoxel, motion)) {
      return
    }

    if (seen.put(nextVoxel, true) == TRUE) {
      return
    }

    putIntoMap(voxelMap, nextVoxel, motion)

    recurse(voxelMap, seen, nextVoxel.minus(horizontal), motion)
    recurse(voxelMap, seen, nextVoxel.plus(horizontal), motion)
    recurse(voxelMap, seen, nextVoxel.minus(vertical), motion)
    recurse(voxelMap, seen, nextVoxel.plus(vertical), motion)
    recurse(voxelMap, seen, nextVoxel.minus(horizontal).minus(vertical), motion)
    recurse(voxelMap, seen, nextVoxel.minus(horizontal).plus(vertical), motion)
    recurse(voxelMap, seen, nextVoxel.plus(horizontal).minus(vertical), motion)
    recurse(voxelMap, seen, nextVoxel.plus(horizontal).plus(vertical), motion)
  }

  def reduceCollisionSet(motions: Vector[Motion]): Vector[Vector[Motion]] = {
    val voxelMap = new RedBlackTree[Vector2D, Vector[Motion]]()
    motions.forEach { motion =>
      drawMotionOnVoxelMap(voxelMap, motion)
    }

    val result = new Vector[Vector[Motion]]();
    voxelMap.forEach { e =>
      if (e.value.size() > 1) {
        result.append(e.value)
      }
    }
    result
  }

  def voxelHash(position: Vector3D): Vector2D = {
    val xDiv = (position.x / Constants.GOOD_VOXEL_SIZE).toInt
    val yDiv = (position.y / Constants.GOOD_VOXEL_SIZE).toInt

    var x = Constants.GOOD_VOXEL_SIZE * xDiv;
    var y = Constants.GOOD_VOXEL_SIZE * yDiv;

    if (position.x < 0) {
      x -= Constants.GOOD_VOXEL_SIZE;
    }
    if (position.y < 0) {
      y -= Constants.GOOD_VOXEL_SIZE;
    }

    new Vector2D(x, y)
  }

  def drawMotionOnVoxelMap(voxelMap: RedBlackTree[Vector2D, Vector[Motion]], motion: Motion): Unit = {
    val seen = new RedBlackTree[Vector2D, Boolean]()
    recurse(voxelMap, seen, voxelHash(motion.posOne), motion)
  }
}

object Constants {
  final val MIN_X = 0.0d
  final val MIN_Y = 0.0d
  final val MAX_X = 1000.0d
  final val MAX_Y = 1000.0d
  final val MIN_Z = 0.0d
  final val MAX_Z = 10.0d
  final val PROXIMITY_RADIUS = 1.0d
  final val GOOD_VOXEL_SIZE  = PROXIMITY_RADIUS * 2.0d
}

class Motion(val callsign: CallSign, val posOne: Vector3D, val posTwo: Vector3D) {
  def delta(): Vector3D =
    posTwo.minus(this.posOne);

  def findIntersection(other: Motion): Vector3D = {
    var init1 = this.posOne
    var init2 = other.posOne
    var vec1 = delta()
    var vec2 = other.delta()
    val radius = Constants.PROXIMITY_RADIUS

    // this test is not geometrical 3-d intersection test, it takes the fact that the aircraft move
    // into account ; so it is more like a 4d test
    // (it assumes that both of the aircraft have a constant speed over the tested interval)

    // we thus have two points, each of them moving on its line segment at constant speed ; we are looking
    // for times when the distance between these two points is smaller than r

    // vec1 is vector of aircraft 1
    // vec2 is vector of aircraft 2

    // a = (V2 - V1)^T * (V2 - V1)
    var a = vec2.minus(vec1).squaredMagnitude()

    if (a != 0.0) {
      // we are first looking for instances of time when the planes are exactly r from each other
      // at least one plane is moving ; if the planes are moving in parallel, they do not have constant speed

      // if the planes are moving in parallel, then
      //   if the faster starts behind the slower, we can have 2, 1, or 0 solutions
      //   if the faster plane starts in front of the slower, we can have 0 or 1 solutions

      // if the planes are not moving in parallel, then

      // point P1 = I1 + vV1
      // point P2 = I2 + vV2
      //   - looking for v, such that dist(P1,P2) = || P1 - P2 || = r

      // it follows that || P1 - P2 || = sqrt( < P1-P2, P1-P2 > )
      //   0 = -r^2 + < P1 - P2, P1 - P2 >
      //  from properties of dot product
      //   0 = -r^2 + <I1-I2,I1-I2> + v * 2<I1-I2, V1-V2> + v^2 *<V1-V2,V1-V2>
      //   so we calculate a, b, c - and solve the quadratic equation
      //   0 = c + bv + av^2

      // b = 2 * <I1-I2, V1-V2>
      var b = 2.0 * init1.minus(init2).dot(vec1.minus(vec2))

      // c = -r^2 + (I2 - I1)^T * (I2 - I1)
      var c = -radius * radius + init2.minus(init1).squaredMagnitude()

      var discr = b * b - 4.0 * a * c
      if (discr < 0.0) {
        return null
      }

      var v1 = (-b - Math.sqrt(discr)) / (2.0 * a)
      var v2 = (-b + Math.sqrt(discr)) / (2.0 * a)

      if (v1 <= v2 && ((v1  <= 1.0 && 1.0 <= v2) ||
                       (v1  <= 0.0 && 0.0 <= v2) ||
                       (0.0 <= v1  && v2  <= 1.0))) {
        // Pick a good "time" at which to report the collision.
        var v = 0.0d
        if (v1 <= 0.0) {
          // The collision started before this frame. Report it at the start of the frame.
          v = 0.0
        } else {
          // The collision started during this frame. Report it at that moment.
          v = v1
        }

        var result1 = init1.plus(vec1.times(v))
        var result2 = init2.plus(vec2.times(v))

        var result = result1.plus(result2).times(0.5)
        if (result.x >= Constants.MIN_X &&
            result.x <= Constants.MAX_X &&
            result.y >= Constants.MIN_Y &&
            result.y <= Constants.MAX_Y &&
            result.z >= Constants.MIN_Z &&
            result.z <= Constants.MAX_Z) {
          return result;
        }
      }

      return null
    }

    // the planes have the same speeds and are moving in parallel (or they are not moving at all)
    // they  thus have the same distance all the time ; we calculate it from the initial point

    // dist = || i2 - i1 || = sqrt(  ( i2 - i1 )^T * ( i2 - i1 ) )
    var dist = init2.minus(init1).magnitude()
    if (dist <= radius) {
      return init1.plus(init2).times(0.5)
    }

    return null;
  }
}



final class RedBlackTree[K <: Comparable[K], V] {
  import RedBlackTree._

  var root: Node[K, V] = null

  def put(key: K, value: V): V = {
    val insertionResult = treeInsert(key, value)
    if (!insertionResult.isNewEntry) {
      return insertionResult.oldValue
    }
    var x = insertionResult.newNode

    while (x != root && x.parent.color == Color.RED) {
      if (x.parent == x.parent.parent.left) {
        val y = x.parent.parent.right
        if (y != null && y.color == Color.RED) {
          // Case 1
          x.parent.color = Color.BLACK
          y.color = Color.BLACK
          x.parent.parent.color = Color.RED
          x = x.parent.parent
        } else {
          if (x == x.parent.right) {
            // Case 2
            x = x.parent
            leftRotate(x)
          }
          // Case 3
          x.parent.color = Color.BLACK
          x.parent.parent.color = Color.RED
          rightRotate(x.parent.parent)
        }
      } else {
        // Same as "then" clause with "right" and "left" exchanged.
        val y = x.parent.parent.left
        if (y != null && y.color == Color.RED) {
          // Case 1
          x.parent.color = Color.BLACK
          y.color = Color.BLACK
          x.parent.parent.color = Color.RED
          x = x.parent.parent
        } else {
          if (x == x.parent.left) {
            // Case 2
            x = x.parent
            rightRotate(x)
          }
          // Case 3
          x.parent.color = Color.BLACK
          x.parent.parent.color = Color.RED
          leftRotate(x.parent.parent)
        }
      }
    }

    root.color = Color.BLACK
    null.asInstanceOf[V]
  }

  def remove(key: K): V = {
    val z = findNode(key)
    if (z == null) {
      return null.asInstanceOf[V]
    }

    // Y is the node to be unlinked from the tree.
    var y: Node[K, V] = null
    if (z.left == null || z.right == null) {
      y = z
    } else {
      y = z.successor()
    }

    // Y is guaranteed to be non-null at this point.
    var x: Node[K, V] = null
    if (y.left != null) {
      x = y.left
    } else {
      x = y.right
    }

    // X is the child of y which might potentially replace y in the tree. X might be null at
    // this point.
    var xParent: Node[K, V] = null
    if (x != null) {
      x.parent = y.parent
      xParent = x.parent
    } else {
      xParent = y.parent
    }
    if (y.parent == null) {
      root = x
    } else {
      if (y == y.parent.left) {
        y.parent.left = x
      } else {
        y.parent.right = x
      }
    }

    if (y != z) {
      if (y.color == Color.BLACK) {
        removeFixup(x, xParent)
      }

      y.parent = z.parent
      y.color = z.color
      y.left = z.left
      y.right = z.right

      if (z.left != null) {
        z.left.parent = y
      }
      if (z.right != null) {
        z.right.parent = y
      }
      if (z.parent != null) {
        if (z.parent.left == z) {
          z.parent.left = y
        } else {
          z.parent.right = y
        }
      } else {
        root = y
      }
    } else if (y.color == Color.BLACK) {
      removeFixup(x, xParent)
    }

    z.value
  }

  def get(key: K): V = {
    val node = findNode(key);
    if (node == null) {
      return null.asInstanceOf[V]
    }
    node.value
  }

  def forEach(fn: Entry[K, V] => Unit): Unit = {
    if (root == null) {
      return;
    }
    var current = treeMinimum(root);
    while (current != null) {
      fn(new Entry(current.key, current.value))
      current = current.successor()
    }
  }

  def findNode(key: K): Node[K, V] = {
    var current = root
    while (current != null) {
      val comparisonResult = key.compareTo(current.key)
      if (comparisonResult == 0) {
        return current
      }
      if (comparisonResult < 0) {
        current = current.left
      } else {
        current = current.right
      }
    }
    null
  }

  def treeInsert(key: K, value: V): InsertResult[K, V] = {
    var y: Node[K, V] = null
    var x: Node[K, V] = root

    while (x != null) {
      y = x
      val comparisonResult = key.compareTo(x.key)
      if (comparisonResult < 0) {
        x = x.left
      } else if (comparisonResult > 0) {
        x = x.right
      } else {
        val oldValue = x.value
        x.value = value
        return new InsertResult(false, null, oldValue)
      }
    }

    var z = new Node[K, V](key, value)
    z.parent = y
    if (y == null) {
      root = z
    } else {
      if (key.compareTo(y.key) < 0) {
        y.left = z
      } else {
        y.right = z
      }
    }
    new InsertResult(true, z, null.asInstanceOf[V]);
  }

  def leftRotate(x: Node[K, V]): Node[K, V] = {
    var y = x.right

    // Turn y's left subtree into x's right subtree.
    x.right = y.left
    if (y.left != null) {
      y.left.parent = x
    }

    // Link x's parent to y.
    y.parent = x.parent
    if (x.parent == null) {
      root = y
    } else {
      if (x == x.parent.left) {
        x.parent.left = y
      } else {
        x.parent.right = y
      }
    }

    // Put x on y's left.
    y.left = x
    x.parent = y

    y
  }

  def rightRotate(y: Node[K, V]): Node[K, V] = {
    var x = y.left

    // Turn x's right subtree into y's left subtree.
    y.left = x.right
    if (x.right != null) {
      x.right.parent = y
    }

    // Link y's parent to x;
    x.parent = y.parent
    if (y.parent == null) {
      root = x
    } else {
      if (y == y.parent.left) {
        y.parent.left = x
      } else {
        y.parent.right = x
      }
    }

    x.right = y
    y.parent = x

    return x
  }

  def removeFixup(_x: Node[K, V], _xParent: Node[K, V]): Unit = {
    var x = _x
    var xParent = _xParent
    while (x != root && (x == null || x.color == Color.BLACK)) {
      if (x == xParent.left) {
        // Note: the text points out that w cannot be null. The reason is not obvious from
        // simply looking at the code; it comes about from the properties of the red-black
        // tree.
        var w = xParent.right
        if (w.color == Color.RED) {
          // Case 1
          w.color = Color.BLACK
          xParent.color = Color.RED
          leftRotate(xParent)
          w = xParent.right
        }
        if ((w.left == null || w.left.color == Color.BLACK)
            && (w.right == null || w.right.color == Color.BLACK)) {
          // Case 2
          w.color = Color.RED
          x = xParent
          xParent = x.parent
        } else {
          if (w.right == null || w.right.color == Color.BLACK) {
            // Case 3
            w.left.color = Color.BLACK
            w.color = Color.RED
            rightRotate(w)
            w = xParent.right
          }
          // Case 4
          w.color = xParent.color
          xParent.color = Color.BLACK
          if (w.right != null) {
            w.right.color = Color.BLACK
          }
          leftRotate(xParent)
          x = root;
          xParent = x.parent
        }
      } else {
        // Same as "then" clause with "right" and "left" exchanged.
        var w = xParent.left
        if (w.color == Color.RED) {
          // Case 1
          w.color = Color.BLACK
          xParent.color = Color.RED
          rightRotate(xParent)
          w = xParent.left
        }
        if ((w.right == null || w.right.color == Color.BLACK)
            && (w.left == null || w.left.color == Color.BLACK)) {
          // Case 2
          w.color = Color.RED
          x = xParent
          xParent = x.parent
        } else {
          if (w.left == null || w.left.color == Color.BLACK) {
            // Case 3
            w.right.color = Color.BLACK
            w.color = Color.RED
            leftRotate(w)
            w = xParent.left
          }
          // Case 4
          w.color = xParent.color
          xParent.color = Color.BLACK
          if (w.left != null) {
            w.left.color = Color.BLACK
          }
          rightRotate(xParent)
          x = root
          xParent = x.parent
        }
      }
    }
    if (x != null) {
      x.color = Color.BLACK
    }
  }
}

object RedBlackTree {
  type Color = Int
  object Color {
    val RED = 1
    val BLACK = 2
  }

  def treeMinimum[K, V](x: Node[K, V]): Node[K, V] = {
    var current = x
    while (current.left != null) {
      current = current.left
    }
    current
  }

  final class Node[K, V](val key: K, var value: V) {
    var left   : Node[K, V] = null
    var right  : Node[K, V] = null
    var parent : Node[K, V] = null
    var color  : Color      = Color.RED

    def successor(): Node[K, V] = {
      var x = this
      if (x.right != null) {
        return treeMinimum(x.right)
      }
      var y = x.parent
      while (y != null && x == y.right) {
        x = y
        y = y.parent
      }
      y
    }
  }

  final class Entry[K, V](val key: K, val value: V)

  final class InsertResult[K, V](val isNewEntry: Boolean, val newNode: Node[K, V], val oldValue: V)
}

final class Simulator(numAircraft: Int) {
  val aircraft = new Vector[CallSign]();
  (0 until numAircraft).foreach { i =>
    aircraft.append(new CallSign(i))
  }

  def simulate(time: Double): Vector[Aircraft] = {
    val frame = new Vector[Aircraft]();
    (0 until aircraft.size()).foreach { i =>
      frame.append(new Aircraft(aircraft.at(i),
          new Vector3D(time, Math.cos(time) * 2 + i * 3, 10)))
      frame.append(new Aircraft(aircraft.at(i + 1),
          new Vector3D(time, Math.sin(time) * 2 + i * 3, 10)))
    }
    frame
  }
}

final class Vector2D(val x: Double, val y: Double) extends Comparable[Vector2D] {
  import Vector2D._

  def plus(other: Vector2D ): Vector2D =
    new Vector2D(x + other.x, y + other.y)

  def minus(other: Vector2D ): Vector2D =
    return new Vector2D(x - other.x, y - other.y)

  @Override
  def compareTo(other: Vector2D): Int = {
    val result = compareNumbers(this.x, other.x)
    if (result != 0) {
      return result
    }
    return compareNumbers(this.y, other.y)
  }
}

object Vector2D {
  def compareNumbers(a: Double, b: Double): Int = {
    if (a == b) {
      return 0;
    }
    if (a < b) {
      return -1;
    }
    if (a > b) {
      return 1;
    }

    // We say that NaN is smaller than non-NaN.
    if (a == a) {
      return 1;
    }
    return -1;
  }
}

final class Vector3D(val x: Double, val y: Double, val z: Double) {
  def plus(other:Vector3D ) =
    new Vector3D(x + other.x, y + other.y, z + other.z)

  def minus(other:Vector3D ) =
    new Vector3D(x - other.x, y - other.y, z - other.z)

  def dot(other: Vector3D ): Double =
    x * other.x + y * other.y + z * other.z

  def squaredMagnitude(): Double =
    this.dot(this)

  def magnitude(): Double =
    Math.sqrt(squaredMagnitude())

  def times(amount:Double ): Vector3D =
    new Vector3D(x * amount, y * amount, z * amount)
}
