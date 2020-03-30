/**
 * This class contains the methods that are used for following the path created by the <code>Pathfinder</code>.
 */
public class PathFollower {
    public static double FINAL_DISTANCE = 0.2;
    public Point pos;
    public  double speed = 0.75; //0.5 is safe, 1.0 is fine for not overly curvy paths
    public  double lookDistance = 1.0; //1 is safe, 1.5 rather risky, below 0.5 does not work
    public  Path path;
    private  RobotCommunication comm = new RobotCommunication("http://127.0.0.1", 50000);
    private  DifferentialDriveRequest driveRequest = new DifferentialDriveRequest();
    private  LocalizationResponse locResponse = new LocalizationResponse();

    PathFollower(Path path) {
        this.path = path;
    }


    //In the end this code is not necessary hear anymore. Instead it should be in the main method of our robot

    //In the end this code is not necessary hear anymore. Instead it should be in the main method of our robot
    void step(Point pos, LocalizationResponse lr, double angle) throws Exception {
        Point firstPoint = path.path[0];
        Point goalPoint = path.getGoalPoint(pos, lookDistance);
        double[] real_pos = lr.getPosition();

        if (path.first) {
            System.out.println("\nDirection check");
            System.out.println("Angle: " + angle * 180/Math.PI);
            System.out.println("goal y: " + firstPoint.getY());
            System.out.println("pos y: " + real_pos[1]);
            System.out.println("goal x: " + firstPoint.getX());
            System.out.println("pos x: " + real_pos[0]);

            //Check direction of first point
            path.first = false;
            double dx = firstPoint.getX() - real_pos[0];
            double dy = firstPoint.getY() - real_pos[1];

            //Check if xGoal is farther away
            if (Math.abs(dx) > Math.abs(dy)) {
                //Check if goal is in positive x Direction
                if (Math.signum(dx) == 1) {
                    turnInDirection(0, angle);
                } else { //goal is in negative x Direction
                    turnInDirection(Math.PI, angle);
                }
            } else { //yGoal is farther away
                //Check if goal is in positive y Direction
                if (Math.signum(dy) == 1) {
                    turnInDirection(Math.PI / 2, angle);
                } else { //Goal is in negative y Direction
                    turnInDirection(-Math.PI / 2, angle);
                }
            }

        }
        double curvature = getCurvature(goalPoint, pos, lr);
        driveRequest.setLinearSpeed(speed);
        driveRequest.setAngularSpeed(speed * curvature);
        comm.putRequest(driveRequest);
    }


    private void turnInDirection(double goal_angle, double angle) throws Exception{
        if (goal_angle != Math.PI) {
            while ( !((angle < goal_angle + Math.PI / 4) && (angle > goal_angle - Math.PI / 4)) ) {
                turnRobot90D();
                comm.getResponse(locResponse);
                angle = locResponse.getHeadingAngle();
            }
        } else {
            while (angle < Math.PI / 4 && angle > -Math.PI / 4) {
                turnRobot90D();
                comm.getResponse(locResponse);
                angle = locResponse.getHeadingAngle();
            }
        }
    }


    private Point getPosition() {
        double[] position = locResponse.getPosition();
        return new Point(position[0], position[1]);
    }

    /**
     * Takes a point in world coordinates and convertes it into the robots coordinate system.
     * @param p Point that shall be converted into robot coordinates.
     * @param pos The current position of the robot.
     * @return A <code>Point</code> with the x and y value beeing in robot coordinates.
     */
    Point worldToRobot(Point p, Point pos, LocalizationResponse lr) {
        double angle = lr.getHeadingAngle();
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double x = (p.x-pos.x) * cos + (p.y-pos.y) * sin;
        double y = - (p.x-pos.x) * sin + (p.y-pos.y) * cos;
        return new Point(x,y);
    }

    /**
     * Takes a point in robot coordinates and convertes it into world coordinate system.
     * @param p Point that shall be converted.
     * @param pos The current position of the robot.
     * @return A <code>Point</code> with the x and y value beeing in world coordinates.
     */
    Point robotToWorld(Point p, Point pos) {
        double angle = locResponse.getHeadingAngle();
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double x = pos.x + p.x*cos - p.y*sin;
        double y = pos.y + p.x*sin + p.y*cos;
        return new Point(x,y);
    }

    //takes world point

    /**
     * Calculates the curvature of the circle through the point and the robot.
     * @param p Point in world coordinates.
     * @param pos Robot position.
     * @return The curvature
     */
    double getCurvature(Point p, Point pos, LocalizationResponse lr) {
        Point wtR = worldToRobot(p,pos, lr);
        return 2 * wtR.y / p.getSquaredDistance(pos);
    }

    void printPosition() {
        double position[] = locResponse.getPosition();
        System.out.println("position = " + position[0] + ", " + position[1]);
    }

    void printHeading() {
        double angle = locResponse.getHeadingAngle();
        System.out.println("heading = " + angle);
    }

    void stopRobot() throws Exception {
        driveRequest.setLinearSpeed(0);
        driveRequest.setAngularSpeed(0);
        comm.putRequest(driveRequest);
    }

    void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ie) {
            throw new RuntimeException(ie);
        }
    }

    private void turnRobot90D() throws Exception {
        driveRequest.setAngularSpeed(Math.PI * 0.5);
        driveRequest.setLinearSpeed(0);
        comm.putRequest(driveRequest);
        sleep(1000);
        driveRequest.setAngularSpeed(Math.PI * 0);
        comm.putRequest(driveRequest);
    }
}
