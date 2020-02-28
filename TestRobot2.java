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
    public static int[] coord;

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

        // set up the request to move in a circle
        dr.setAngularSpeed(Math.PI * 0.0);
        dr.setLinearSpeed(0.0);
        System.out.println("Start to move robot");
        int rc = robotcomm.putRequest(dr);
        System.out.println("Response code " + rc);
/*
        for (int i = 0; i < 10; i++) {
            robotcomm.getResponse(lr);
            double[] pos = lr.getPosition();
            System.out.println("X Cor: " + pos[0]);
            System.out.println("Y Cor: " + pos[1]);
            try {
                Thread.sleep(500);
            } catch (InterruptedException ex) {
            }
        }
*/

/*
        dr.setAngularSpeed(Math.PI * 0.0);
        dr.setLinearSpeed(0.0);
        robotcomm.putRequest(dr);
*/

/*
        double angle = 0;
        double[] echoes = new double[0];
        int i = 0;
        while (i < 10000) {
            // Ask for the laser beam angles
            robotcomm.getResponse(lpr);
            robotcomm.getResponse(lr);
            robotcomm.getResponse(ler);
            double[] angles = getLaserAngles(lpr);
            angle = lr.getHeadingAngle();
            echoes = ler.getEchoes();
            float[][] grid = createMap(lr, angle, echoes, angles); // create an example map
            try {
                Thread.sleep(500);
            } catch (InterruptedException ex) {
            }
            i++;
        }
*/

        // Ask for the laser beam angles
        double angle = 0;
        double[] echoes = new double[0];
        robotcomm.getResponse(lpr);
        robotcomm.getResponse(lr);
        robotcomm.getResponse(ler);
        double[] angles = getLaserAngles(lpr);
        angle = lr.getHeadingAngle();
        echoes = ler.getEchoes();
        float[][] grid = createMap(lr, angle, echoes, angles); // create an example map

        //for (int i = grid.length - 1; i >= 0; i--) {
        //    System.out.print("Y = " + i + "  ");
        //    for (int j = 0; j < grid[0].length; j++) {
        //        System.out.print("X = " + j + " " + grid[i][j] +  "  ");
        //    }
        //    System.out.print("\n");
        //}

        //Compute A Path
        double [] rob_pos = lr.getPosition();
        double[] rob_pos_grid = new double[2];
        rob_pos_grid[0] = rob_pos[0] + Math.abs(x_min);
        rob_pos_grid[1] = rob_pos[1] + Math.abs(y_min);


        Point start = new Point(rob_pos_grid);

        Pathfinder scout = new Pathfinder(0.2, 0.2, (double)x_min, (double)y_min, grid);
        Path path = scout.findPath(start, grid);
        System.out.println("Pathlength: " + path.path.length);

        //Follow the Path
        PathFollower follower = new PathFollower(path);
        System.out.print("Pathfollowing begins");
        follower.run();



        /*for (int i = 0; i < 10; i++) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
            }

            angle = lr.getHeadingAngle();
            System.out.println("heading = " + angle);

            double[] position = getPosition(lr);

            System.out.println("position = " + position[0] + ", " + position[1]);

            // Ask the robot for laser echoes
            robotcomm.getResponse(ler);
            echoes = ler.getEchoes();
            System.out.println("Object at " + echoes[135] + "m in " + angles[135] * 180.0 / Math.PI + " degrees"); //object in front of the robot
        }*/
        /*System.out.println("Angle at 0: " + angles[0] * 180.0 / Math.PI + " at 45: " + angles[45] * 180.0 / Math.PI
                + " at 90: " + angles[90] * 180.0 / Math.PI + " at 225: " + angles[225] * 180.0 / Math.PI
                + "\nAngle at 268: " + angles[268] * 180.0 / Math.PI + " at 270: " + angles[270] * 180.0 / Math.PI
                + " at 270: " + angles[269] * 180.0 / Math.PI);

        // This is where the laser is mounted on the robot (15cm in front of center)
        double[] lpos = lpr.getPosition();
        System.out.println("Laser position (x,y,z): (" + lpos[0] + ", " + lpos[1] + ", " + lpos[2] + ")");

        // ask the robot about its position and angle

        // set up request to stop the robot
        dr.setLinearSpeed(0);
        dr.setAngularSpeed(0);*/


        System.out.println("Stop robot");
        rc = robotcomm.putRequest(dr);
        System.out.println("Response code " + rc);
    }

    /**
     * A simple example of how to use the ShowMap class that creates a map from
     * your grid, update it and save it to file
     */
    private float[][] createMap(LocalizationResponse localizationResponse, double angle,
                           double[] echoes, double[] angles) {
        /* use the same no. of rows and cols in map and grid */
        int nCols = (int) (Math.abs(x_max - x_min) / 0.2);
        System.out.println("nCols: " + nCols);
        int nRows = (int) (Math.abs(y_max - y_min) / 0.2);

        boolean showGUI = true; // set this to false if you run in putty
        ShowMap map = new ShowMap(nRows, nCols, showGUI, coord);

        /* Creating a grid with 0.5 */
        float[][] grid = new float[nRows][nCols];
        float[][] image_grid = new float[nRows][nCols];
        for (int i = nRows - 1; i >= 0; i--) {
            for (int j = 0; j < nCols; j++) {
                grid[i][j] = (float) 0.4;
                image_grid[i][j] = (float) 0.4;
            }
        }


        // Position of the robot in the grid (red dot)
        double[] position_robot = getPosition(localizationResponse);
        int robotCol = (int) Math.round(position_robot[0]); //y
        int robotRow = (int) Math.round(position_robot[1]); //x

        double tt = localizationResponse.getHeadingAngle(); //angle in radians

        for (int i = 0; i < echoes.length; i++) {
            double y_end_line = robotRow + (echoes[i] * Math.sin(angles[i] + tt)); // y2 = y1 + (lenght * sin(angle))
            double y_end_line_image = robotRow + (echoes[i] * -Math.sin(angles[i] + tt)); // y2 = y1 + (lenght * sin(angle))
            double x_end_line = robotCol + (echoes[i] * Math.cos(angles[i] + tt)); // x2 = x1 + (lenght * cos(angle)) angle in radians

            int[] obstacle = map.xy_to_rc(x_end_line, y_end_line);
            int[] obstacle_image = map.xy_to_rc(x_end_line, y_end_line_image);
            if (x_end_line > x_min && x_end_line < x_max && y_end_line > y_min && y_end_line < y_max) {
                colorGrid(grid, obstacle[0], obstacle[1]);
            }
            if (x_end_line > x_min && x_end_line < x_max && y_end_line_image > y_min && y_end_line_image < y_max) {
                colorGrid(image_grid, obstacle_image[0], obstacle_image[1]);
            }
        }


        System.out.println(" tt = " + tt * (180 / Math.PI));

        // Update the grid
        map.updateMap(image_grid, robotRow, robotCol, echoes, angles);
        System.out.println("grid x length: " + grid[0].length);
        return grid;
    }

    //Color the grid
    public void colorGrid(float[][] grid, int x, int y) {
        int x_cell = x / 5;
        int y_cell = y / 5;

        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                grid[y_cell * 5 + j][x_cell * 5 + i] = (float) 1.0;
            }
        }
    }

    //Determine where the obstacle is in the cell
    public int WhereIsObstacle(int x, int y, int scale) {
        if (x <= (scale / 2) && y <= (scale / 2)) {
            return 1; //obstacle in Upper left area
        }
        if (x >= (scale / 2) && y <= (scale / 2)) {
            return 2; //obstacle in upper right area
        }
        if (x <= (scale / 2) && y >= (scale / 2)) {
            return 3; //obstacle in bottom left area
        }
        return 4; //Otherwise bottom right area
    }

    /**
     * Extract the robot bearing from the response
     *
     * @param lr
     * @return angle in degrees
     */
    double getBearingAngle(LocalizationResponse lr) {
        double angle = lr.getHeadingAngle();
        return angle * 180 / Math.PI;
    }

    /**
     * Extract the position
     *
     * @param lr
     * @return coordinates
     */
    double[] getPosition(LocalizationResponse lr) {
        return lr.getPosition();
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


}
