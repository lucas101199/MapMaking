import java.util.LinkedList;

/**
 * TestRobot interfaces to the (real or virtual) robot over a network
 * connection. It uses Java -> JSON -> HttpRequest -> Network -> DssHost32 ->
 * Lokarria(Robulab) -> Core -> MRDS4
 *
 * @author Thomas Johansson, dept. of Computing Science, Umeå University, Umeå,
 *         Sweden Mail: thomasj@cs.umu.se
 *
 *         Updated by Ola Ringdahl 2015-03-13, 2015-12-16, 2018-11-23
 */
public class TestRobot2 {

    private RobotCommunication robotcomm; // communication drivers
    public static int x_min;
    public static int y_min;
    public static int x_max;
    public static int y_max;
    private double cell_size = 0.2;
    public static int[] coord;
    public boolean first_time = true;
    public float[][] grid;
    public float[][] image_grid;
    public ShowMap map;
    /**
     * Create a robot connected to host "host" at port "port"
     *
     * @param host
     *            normally http://127.0.0.1
     * @param port
     *            normally 50000
     */
    public TestRobot2(String host, int port) {
        robotcomm = new RobotCommunication(host, port);
    }
    /**
     * This simple main program creates a robot, sets up some speed and turning
     * rate and then displays angle and position for 16 seconds.
     *
     * @param args
     *            not used
     * @throws Exception
     *             not caught
     */
    public static void main(String[] args) throws Exception {
        System.out.println("Creating Robot");
        TestRobot2 robot = new TestRobot2("http://127.0.0.1", 50000);
        //TestRobot2 robot = new TestRobot2("http://bratwurst.cs.umu.se", 50000);*
        x_min = -30;
        y_min = -40;
        x_max = 40;
        y_max = 20;
        coord = new int[]{x_min, y_min, x_max, y_max};

        robot.run();
        /*try {
            // Check for connection c
            robot.run();
        }
        catch (Exception e) {
            System.out.println(e);
            System.exit(-1);
        }*/
    }

    private void run() throws Exception {
        System.out.println("Creating response");
        LocalizationResponse lr = new LocalizationResponse();
        LaserEchoesResponse ler = new LaserEchoesResponse();
        LaserPropertiesResponse lpr = new LaserPropertiesResponse();

        System.out.println("Creating request");
        DifferentialDriveRequest dr = new DifferentialDriveRequest();

        System.out.println("Start to move robot");
        int rc = robotcomm.putRequest(dr);
        System.out.println("Response code " + rc);

        // Ask for the laser beam angles
        double angle;
        double[] echoes;
        robotcomm.getResponse(lpr);
        robotcomm.getResponse(lr);
        robotcomm.getResponse(ler);

        double[] angles = getLaserAngles(lpr);
        angle = lr.getHeadingAngle();
        echoes = ler.getEchoes();

        int nCols = (int) (Math.abs(x_max - x_min) / cell_size);
        int nRows = (int) (Math.abs(y_max - y_min) / cell_size);
        boolean showGUI = true; // set this to false if you run in putty
        map = new ShowMap(nRows, nCols, showGUI, coord);

        ObjectAvoider objectAvoider = new ObjectAvoider();

        while (true) {
            robotcomm.getResponse(lpr);
            robotcomm.getResponse(lr);
            robotcomm.getResponse(ler);
            angles = getLaserAngles(lpr);
            angle = lr.getHeadingAngle();
            echoes = ler.getEchoes();

            grid = createMap(lr, echoes, angles); // create an example map
            /*dr.setAngularSpeed(Math.PI * 0.5);
            robotcomm.putRequest(dr);
            sleep(2000);
            dr.setAngularSpeed(Math.PI * 0);
            robotcomm.putRequest(dr);
            sleep(5000);

            robotcomm.getResponse(lpr);
            robotcomm.getResponse(lr);
            robotcomm.getResponse(ler);
            angles = getLaserAngles(lpr);
            angle = lr.getHeadingAngle();
            echoes = ler.getEchoes();

            grid = createMap(lr, echoes, angles); // create an example map*/

            //Compute A Path
            double[] rob_pos = lr.getPosition();
            int[] rob_pos_grid = map.xy_to_rc(rob_pos[0], rob_pos[1]);

            Point start = new Point(rob_pos_grid);

            Pathfinder scout = new Pathfinder(cell_size, cell_size, (double) x_min, (double) y_min, grid);
            Path path = scout.findPath(map, start, grid);
            System.out.println("Pathlength: " + path.path.length);
            /*if (path.path.length == 1) {
                for (int i = 0; i < grid.length; i++) {
                    System.out.println("\n");
                    for (int j = 0; j < grid[0].length; j++) {
                        System.out.print(grid[i][j] +  " ");
                    }
                }
            }*/

            sleep(1000);
            //Follow the Path
            PathFollower follower = new PathFollower(path);
            System.out.print("Pathfollowing begins");

            Point pos = new Point(rob_pos_grid);
            while ( (!path.pathFinished()) ) {
                System.out.println("Step");
                robotcomm.getResponse(lpr);
                robotcomm.getResponse(lr);
                robotcomm.getResponse(ler);
                angles = getLaserAngles(lpr);
                angle = lr.getHeadingAngle();
                echoes = ler.getEchoes();

                follower.step(pos, lr);

                if (objectAvoider.avoidObject(echoes)) { //if obstacle encounter stop the robot
                    System.out.println("Object Avoidance");
                    break;
                }

                grid = createMap(lr, echoes, angles); // create an example map
                map.showGoal(grid, path.path[path.path.length - 1].getX(), path.path[path.path.length - 1].getY());
                pos.setX(lr.getPosition()[0]);
                pos.setY(lr.getPosition()[1]);


                follower.sleep(30);
            }
            System.out.println("Path finished!\n");
            dr.setAngularSpeed(Math.PI * 0);
            dr.setLinearSpeed(0);
            robotcomm.putRequest(dr);
            sleep(5000);
        }

        /*System.out.println("Stop robot");
        rc = robotcomm.putRequest(dr);
        System.out.println("Response code " + rc);*/
    }

    /**
     * A simple example of how to use the ShowMap class that creates a map from
     * your grid, update it and save it to file
     */
    private float[][] createMap(LocalizationResponse localizationResponse,
                           double[] echoes, double[] angles) {
        /* use the same no. of rows and cols in map and grid */
        int nCols = (int) (Math.abs(x_max - x_min) / cell_size);
        int nRows = (int) (Math.abs(y_max - y_min) / cell_size);

        /* Creating a grid with 0.5 */
        if (first_time) {
            first_time = false;
            grid = new float[nRows][nCols];
            image_grid = new float[nRows][nCols];
            for (int i = nRows - 1; i >= 0; i--) {
                for (int j = 0; j < nCols - 1; j++) {
                    grid[i][j] = (float) 0.5;
                    image_grid[i][j] = (float) 0.5;
                }
            }
        }



        // Position of the robot in the grid (red dot)
        double[] position_robot =  localizationResponse.getPosition();
        double robotCol = position_robot[0];
        double robotRow = position_robot[1];

        double tt = localizationResponse.getHeadingAngle(); //angle in radians

        for (int i = 0; i < echoes.length; i++) {

            double y_end_line = robotRow + (echoes[i] * Math.sin(angles[i] + tt)); // y2 = y1 + (lenght * sin(angle)) grid without minus for path planning
            double x_end_line = robotCol + (echoes[i] *  Math.cos(angles[i] + tt)); // x2 = x1 + (lenght * cos(angle)) angle in radians
            double y_end_line_image = robotRow + (echoes[i] * Math.sin(angles[i] + tt)); // y2 = y1 + (lenght * sin(angle)) grid with minus

            int[] obstacle = map.xy_to_rc(x_end_line, y_end_line);
            int[] robot = map.xy_to_rc(robotCol, robotRow);
            int[] obstacle_image = map.xy_to_rc(x_end_line, y_end_line_image);

            colorGrid(grid, echoes[i], obstacle[0], obstacle[1], robot[0], robot[1], map);
            colorGrid(image_grid, echoes[i], obstacle_image[0], obstacle_image[1], robot[0], robot[1], map);

            //growObstacles(cell_size, cell_size);
        }

        // Update the grid
        map.updateMap(grid, robotRow, robotCol);

        return grid;
    }

    //Color the grid using bayes
    public void colorGrid(float[][] grid, double echo, int x, int y, int start_x, int start_y, ShowMap map) {
        LinkedList<Point> visited_points = map.drawBresenhamLine(start_x, start_y, x, y);
        double[] pos_robot_xy = map.rc_to_xy(start_x, start_y);
        Point point_robot = new Point(pos_robot_xy[0], pos_robot_xy[1]);

        for (Point point : visited_points) {
            double[] point_rayon_xy = map.rc_to_xy((int) point.x, (int) point.y);
            float prob_cell = grid[(int)point.y][(int)point.x];
            Point new_point = new Point(point_rayon_xy[0], point_rayon_xy[1]);
            double distance_robot_cell = point_robot.getDistance(new_point);
            float prob_occ;
            float prob_occ_recur = prob_cell;
            //check if the echo is in 10m range
            //System.out.println("Distance to Object: " + echo);
            //System.out.println("Distance to Cell: " + distance_robot_cell);
            if (echo < 10.0) {
                //System.out.println("Within R");
                //Check if cell is in region 2
                if (distance_robot_cell < (echo - 0.11)) {
                    //System.out.println("Cell in Region 2");
                    prob_occ = map.Bayes(distance_robot_cell);
                    prob_occ_recur = map.recursive_bayes(prob_occ, prob_cell);
                } else {
                    if (distance_robot_cell > (echo + 0.11)) {
                        //System.out.println("Cell in Region 3");
                        // cell in region 3
                        prob_occ_recur = prob_cell;
                    } else {
                        //System.out.println("Cell in Region 1");
                        //Check if the cell is in region 1
                        prob_occ = map.Bayes_R1(distance_robot_cell);
                        prob_occ_recur = map.recursive_bayes(prob_occ, prob_cell);
                    }
                }
            } else {
                //The echo is grater than R so every cell within distance R is free (Region 2)
                if (distance_robot_cell < 10) {
                    //System.out.println("Echo grater R");
                    prob_occ = map.Bayes(distance_robot_cell);
                    prob_occ_recur = map.recursive_bayes(prob_occ, prob_cell);
                }
            }

            //float prob_occ_recur = map.recursive_bayes(prob_occ, prob_cell);
            //System.out.println("Occupancy probabillity: " + prob_occ_recur + "\n");
            grid[(int) point.y][(int) point.x] = prob_occ_recur;
        }
    }

    /**
     * Get corresponding angles to each laser beam
     *
     * @param lpr
     * @return laser angles in radians
     */
    double[] getLaserAngles(LaserPropertiesResponse lpr) {
        int beamCount = (int) ((lpr.getEndAngle() - lpr.getStartAngle()) / lpr.getAngleIncrement()) + 1;
        double[] angles = new double[beamCount];
        double a = lpr.getStartAngle();
        for (int i = 0; i < beamCount; i++) {
            angles[i] = a;
            // We get roundoff errors if we use AngleIncrement. Use 1 degree in
            // radians instead
            a += 1 * Math.PI / 180;// lpr.getAngleIncrement();
        }
        return angles;
    }

    void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ie) {
            throw new RuntimeException(ie);
        }
    }

    public boolean Stop_robot(float[][] grid) {
        double percentage_map = 0;
        for (int i = 0; i < grid.length; i++) {
            for (int j = 0; j < grid[0].length; j++) {
                if (grid[i][j] == 0.5) {
                    percentage_map += 1;
                }
            }
        }
        boolean stop = true;
        double percentage = percentage_map / (grid.length * grid[0].length);
        if (percentage < 0.1) {
            stop = false;
        }
        return stop;
    }

    private void growObstacles(double robot_width, double robot_height) {
        for (int i = 0; i < grid.length; i++) {
            //Grow obstacles in negative x direction
            for (int j = 0; j < grid[0].length - 1; j++) { //-1 because the last cell has no cell to look at
                if (grid[i][j + 1] > 0.5) {
                    grid[i][j] = grid[i][j + 1];
                }
            }
            //Grow obstacles in positive x direction
            for (int j = grid[0].length - 1; j >= 1; j--) { //-1 because the last cell has no cell to look at
                if (grid[i][j - 1] > 0.5) {
                    grid[i][j] = grid[i][j - 1];
                }
            }
        }

        for (int i = 0; i < grid[0].length; i++) {
            //Grow obstacles in negative y direction
            for (int j = 0; j < grid.length - 1; j++) { //-1 because the last cell has no cell to look at
                if (grid[j + 1][i] > 0.5) {
                    grid[j][i] = grid[j + 1][i];
                }
            }
            //Grow obstacles in positive y direction
            for (int j = grid.length - 1; j >= 1; j--) { //-1 because the last cell has no cell to look at
                if (grid[j - 1][i] > 0.5) {
                    grid[j][i] = grid[j - 1][i];
                }
            }
        }
    }

}
