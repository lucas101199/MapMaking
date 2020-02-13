import java.awt.*;

/**
 * RobotMain tests the Robot class
 * It uses Java -> JSON -> HttpRequest -> Network -> DssHost32 -> Lokarria(Robulab) -> Core -> MRDS4
 *
 * @author thomasj
 * @date 2019-08-29
 */
/*
public class RobotMain {
    public static double FINAL_DISTANCE = 0.2;
    public static RobotMain main;
    public Robot robot;
    public Point pos;
    public double speed = 1; //0.5 is safe, 1.0 is fine for not overly curvy paths
    public double lookDistance = 1.5; //1 is safe, 1.5 rather risky, below 0.5 does not work
    public Path path;

    public static void main(String[] args) throws Exception {
        main = new RobotMain("http://127.0.0.1", 50000);
        main.run();
    }

    RobotMain(String url, int port) {
        robot = new Robot(url,port);
        path = new Path("Path-around-table-and-back.json");
    }

    void run() {
        while (!path.pathFinished() || path.getNextPoint().getDistance(pos) < FINAL_DISTANCE) {
            step();
            sleep(30);
        }
        stopRobot();
        System.out.println("Path done!");
    }

    void step() {
        pos = getPosition();
        Point goalPoint = path.getGoalPoint(pos,lookDistance);
        double curvature = getCurvature(goalPoint, pos);
        robot.setMotion(speed,speed*curvature);
        //System.out.println("Set new course, curvature:"+curvature);
        //printPosition();
        //System.out.println("Goal:"+goalPoint);
        //printHeading();
    }

    static void normalizeRadian(double r) {
        r += r>0 ? 0 : 2*Math.PI;
    }

    Point getPosition() {
        double[] position = robot.getPosition();
        return new Point(position[0], position[1]);
    }

    Point worldToRobot(Point p, Point pos) {
        double angle = robot.getHeading();
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double x = (p.x-pos.x) * cos + (p.y-pos.y) * sin;
        double y = - (p.x-pos.x) * sin + (p.y-pos.y) * cos;
        return new Point(x,y);
    }

    Point robotToWorld(Point p, Point pos) {
        double angle = robot.getHeading();
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double x = pos.x + p.x*cos - p.y*sin;
        double y = pos.y + p.x*sin + p.y*cos;
        return new Point(x,y);
    }

    //takes world point
    double getCurvature(Point p, Point pos) {
        Point wtR = worldToRobot(p,pos);
        return 2 * wtR.y / p.getSquaredDistance(pos);
    }

    void printPosition() {
        double position[] = robot.getPosition();
        System.out.println("position = " + position[0] + ", " + position[1]);
    }

    void printHeading() {
        double angle = robot.getHeading();
        System.out.println("heading = " + angle);
    }

    void stopRobot() {
        robot.setMotion(0, 0);
    }

    static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ie) {
            throw new RuntimeException(ie);
        }
    }

    void printEchos() {
        double[] echos = robot.getLaser();
        for (int i = 0; i < echos.length; i++) {
            if ((i % 20) == 0) System.out.println();
            System.out.print(echos[i] + " ");
        }
        System.out.println();
    }
}*/