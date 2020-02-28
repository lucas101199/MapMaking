
/**
 * This Class resambles a position in the world with x and y coordinate.
 */
public class Point
{
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
}
