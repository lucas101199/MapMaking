import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.LinkedList;

/**
 * ShowMap creates a BufferedImage that show the map and saves it to file every
 * 5 second
 *
 * @author Peter Hohnloser
 * Updated by Ola Ringdahl
 */
public class ShowMap extends JPanel {

    private static final long serialVersionUID = 1L;
    private BufferedImage map;
    private Thread task;
    // drawing and saving image size
    private int imageWidth;
    private int imageHeight;
    // scale up the image
    private int scale = 4;
    // Time for saving the BufferedImage as image
    private int saveImageTime = 5000;
    // Robot size in pixels
    private int robotSize = 2;
    // if false, no map will be shown on screen
    private boolean showGUI = true;
    private double cell_size = 0.2; //20cm
    private int x_min, y_min, x_max, y_max;

    /**
     * Constructor for ShowMap
     *
     * @param gridHeight
     *            the height of the grid (no. of rows)
     * @param gridWidth
     *            the width of the grid (no. of columns)
     * @param showGUI
     *            if false, no map will be shown on screen. Good if you are
     *            using Putty for example
     */
    public ShowMap(int gridHeight, int gridWidth, boolean showGUI, int[] coord) {
        super(true);
        this.showGUI = showGUI;
        imageHeight = scale * gridHeight;
        imageWidth = scale * gridWidth;
        if (showGUI) {
            JFrame frame = new JFrame();
            // frame.setResizable(false);
            // make sure the program exits when the frame closes
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setTitle("Showing Map");
            frame.setSize(imageWidth + 17, imageHeight + 39);
            // This will center the JFrame in the middle of the screen
            frame.setLocationRelativeTo(null);
            frame.setLayout(new BorderLayout());
            frame.add(this, BorderLayout.CENTER);

            // updating the Gui
            this.updateUI();
            frame.setVisible(showGUI);
        }
        x_min = coord[0];
        y_min = coord[1];
        x_max = coord[2];
        y_max = coord[3];

        // creating a gray BufferedImage
        map = new BufferedImage(gridWidth, gridHeight, BufferedImage.TYPE_INT_RGB);
        Color c = new Color(127, 127, 127);
        for (int col = 0; col < gridWidth; col++) {
            for (int row = 0; row < gridHeight; row++) {
                map.setRGB(col, row, c.getRGB());
            }
        }
        // creating a thread for saving the map as an image
        task = new Thread() {
            public void run() {
                while (true) {
                    saveMap();
                    System.out.println("Saving map.");
                    try {
                        Thread.sleep(saveImageTime);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        };
        task.start();
    }

    /**
     * Creates a new BufferedImage from a grid with float values between
     * 0.0-1.0, where 0.0 is black and 1.0 is white, with a grey scale in
     * between. Negative values are shown as gray. Call this Method after you
     * have updated the grid.
     *
     * @param grid
     *            is the updated grid
     * @param robotRow
     *            is the current y-position (row) of the robot in the grid.
     * @param robotCol
     *            is the current x-position (column) of the robot translated to
     *            column in the grid.
     */
    public synchronized void updateMap(float[][] grid, int robotRow,
                                       int robotCol, double[] echoes, double[] angles) {
        Color c;
        for (int row = 0; row < grid.length; row++) {
            for (int col = 0; col < grid[0].length; col++) {
                float value = grid[row][col];
                // if value is <0 draw a gray pixel
                // else mapping the value between 0.0 - 1.0 where 1.0 is black
                // and 0.0 is white
                if (value < 0) {
                    c = new Color(0.5f, 0.5f, 0.5f);
                } else {
                    value = Math.abs(value - 1);
                    c = new Color(value, value, value);
                }
                // setting pixel color for pixel col and row
                map.setRGB(col, row, c.getRGB());
            }
        }

        //get the position of the robot in the grid
        int[] position_robot = xy_to_rc(robotCol, robotRow);

        // drawing a filled red Rectangle for the robot. Rectangle size is
        // 5x5
        Graphics g = map.getGraphics();
        g.setColor(Color.RED);
        g.fillRect(position_robot[0], position_robot[1] ,robotSize, robotSize);
        this.updateUI();

        
        LinkedList<Point> visited_point = new LinkedList<>();

        for (int i = 0; i < echoes.length; i++) {
            double y_end_line = robotRow + (echoes[i] * -Math.sin(angles[i])); // y2 = y1 + (lenght * sin(angle))
            double x_end_line = robotCol + (echoes[i] * Math.cos(angles[i])); // x2 = x1 + (lenght * cos(angle)) angle in radians

            int[] obstacle = xy_to_rc(x_end_line, y_end_line);
            g.setColor(Color.BLUE);
            g.drawLine(obstacle[0], obstacle[1], obstacle[0], obstacle[1]);
            g.setColor(Color.CYAN);
            if (x_end_line > x_min && x_end_line < x_max && y_end_line > y_min && y_end_line < y_max) {
                visited_point = drawBresenhamLine(position_robot[0], position_robot[1], obstacle[0], obstacle[1]);
                for (Point point : visited_point) {
                    map.setRGB((int) point.x, (int) point.y, 0);
                }
            }
        }

        // update the gui
        this.updateUI();
    }

    /**
     * Creates a new BufferedImage from a grid with integer values between 0 -
     * maxVal, where 0 is black and maxVal is white, with a grey scale in
     * between. Negative values are shown as gray. Call this Method after you
     * have updated the grid.
     *
     * @param grid
     *            is the updated grid
     * @param maxVal
     *            is the max value that a grid cell can have (e.g. 15 in
     *            standard HIMM)
     * @param robotRow
     *            is the current y-position (row) of the robot in the grid.
     * @param robotCol
     *            is the current x-position (column) of the robot translated to
     *            column in the grid.
     */
    public synchronized void updateMap(int[][] grid, int maxVal, int robotRow,
                                       int robotCol) {
        // mapping the grid to a BufferedImage
        Color c;
        for (int col = 0; col < grid.length; col++) {
            for (int row = 0; row < grid[0].length; row++) {
                int value = grid[col][row];
                // if value is <0 draw a gray pixel
                // else mapping the value between 0 - 255 where 0 is black and
                // 255 is white
                if (value < 0) {
                    c = new Color(127, 127, 127);
                } else {
                    value = Math.abs(value * 255 / maxVal - 255);
                    c = new Color(value, value, value);
                }
                // setting pixel color for pixel col and row
                map.setRGB(row, col, c.getRGB());
            }

        }
        Graphics g = map.getGraphics();
        // drawing a filled red Rectangle for the robot. Rectangle size is
        // 6x6
        g.setColor(Color.RED);
        g.fillRect((int) robotRow - robotSize / 2, (int) robotCol - robotSize
                / 2, robotSize, robotSize);
        // update the gui
        this.updateUI();
    }

    /**
     * Method for saving the BufferedImage as a gif image
     */
    private synchronized void saveMap() {
        try {
            File outputfile = new File("map.gif");
            // Transforming the map BufferedImage to the size of image that is
            // shown
            AffineTransform tx = new AffineTransform();
            tx.scale(scale, scale);
            AffineTransformOp op = new AffineTransformOp(tx,
                    AffineTransformOp.TYPE_BILINEAR);
            BufferedImage tmp = op.filter(map, null);
            ImageIO.write(tmp, "gif", outputfile);
        } catch (IOException e) {
            System.err.println("Couldn't save Map Image");
        }

    }

    /**
     * Method for drawing the image
     */
    @Override
    protected void paintComponent(Graphics g) {
        // Transforming the map BufferedImage to the size of image that is shown
        AffineTransform tx = new AffineTransform();
        tx.scale(scale, scale);
        AffineTransformOp op = new AffineTransformOp(tx,
                AffineTransformOp.TYPE_BILINEAR);
        BufferedImage tmp = op.filter(map, null);
        // drawing the transformed BufferedImage to the screen
        g.drawImage(tmp, 0, 0, this);
        g.dispose();
    }

    public int[] xy_to_rc(double x, double y) {
        int col = (int) ((x - x_min) / cell_size);
        int row = (int) ((y - y_min) / cell_size);
        return new int[]{col,row};
    }

    private int sign (int x) {
        return Integer.compare(x, 0);
    }

    public LinkedList<Point> drawBresenhamLine (int xstart, int ystart, int xend, int yend) {
        int x, y, dx, dy, incx, incy, pdx, pdy, es, el, err;
        LinkedList<Point> visited_point = new LinkedList<>();

        dx = xend - xstart;
        dy = yend - ystart;

        incx = sign(dx);
        incy = sign(dy);

        if (dx < 0) dx = -dx;
        if (dy < 0) dy = -dy;

        if (dx > dy) {
            pdx = incx;     pdy = 0;
            es = dy;        el = dx;
        }
        else {
            pdx = 0;        pdy = incy;
            es = dx;        el = dy;
        }

        x = xstart;
        y = ystart;
        err = el/2;

        for (int t = 0; t < el; t++) {
            err -= es;
            if (err < 0) {
                err += el;
                x += incx;
                y += incy;
            }
            else {
                x += pdx;
                y += pdy;
            }
            visited_point.add(new Point(x,y));
        }
        return visited_point;
    }
}
