import java.awt.print.Printable;

public class ExampleHowToUseThePlanner {
    public static void example() {
        //Example map and start and goal point
        float[][] map = {
                {0, 0, 0, 0, 0},
                {1, 1, 0, 0, 1},
                {0, 0, 1, 0, 1},
                {0, 0, 0, 0, 1},
                {0, 0, 1, 1, 0},
                {1, 0, 1, 1, 1},
                {0, 0, 0, 0, 0}
        };
        Point start = new Point(0.5, 0.7);
        Point goal = new Point(0.5, 0.7);


        //Creating a new Pathfinder. First parameter is the width of a grid field in meter.
        //Second parameter is the height of a grid field in meter
        //Third parameter is the recent map
        Pathfinder scout = new Pathfinder(1.0, 1.0, map);

        //findPath plans a new path, which then can be followed by the PathFollower.
        //First parameter is the recent position of the robot
        //Second parameter is the point were the path should end in
        //Third parameter is the recent map
        Path way = scout.findPath(start, goal, map);
    }

}
