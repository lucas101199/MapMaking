
/**
 * This Class resambles a position in the world with x and y coordinate.
 */
public class Point
{
<<<<<<< HEAD
<<<<<<< HEAD
   public double x, y;

   public Point(double pos[])
   {
      this.x = pos[0];
      this.y = pos[1];
   }

   public Point(double x, double y)
   {
      this.x = x;
      this.y = y;
   }

   public double getX() { return x; }
   public double getY() { return y; }

   /**
    * Returns the distance from point <code>p</code> to this point.
    * @param p The other point
    * @return the distance from this point to p
    */
   public double getDistance(Point p) {
      return Math.sqrt(getSquaredDistance(p));
   }

   /**
    * This method returnes the squared distance from another point to this point
    * @param p The other point
    * @return The squared distance between this point and point <code>p</code>
    */
   public double getSquaredDistance(Point p) {
      return Math.pow(p.x-x, 2) + Math.pow(p.y-y, 2);
   }


   public String toString() {
      return "x="+x+", y="+y;
   }
=======
=======
>>>>>>> c21d22ef66e19976f847e9701143c05c9736e3ff
    public double x, y;

    public Point(double pos[])
    {
        this.x = pos[0];
        this.y = pos[1];
    }

    public Point(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public double getX() { return x; }
    public double getY() { return y; }

    public double getDistance(Point p) {
        return Math.sqrt(getSquaredDistance(p));
    }

    public double getSquaredDistance(Point p) {
        return Math.pow(p.x-x, 2) + Math.pow(p.y-y, 2);
    }

    // Bearing to another position, realtive to 'North'
    // Bearing have several meanings, in this case the angle between
    // north and the position p.
    public double getBearingTo(Point p) {
        return Math.atan2(p.y - y, p.x - x);
    }

    public String toString() {
        return "x="+x+", y="+y;
    }
<<<<<<< HEAD
>>>>>>> c21d22ef66e19976f847e9701143c05c9736e3ff
=======
>>>>>>> c21d22ef66e19976f847e9701143c05c9736e3ff
}
