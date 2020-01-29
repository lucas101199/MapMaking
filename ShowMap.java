import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

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
    private int robotSize = 4;
    // if false, no map will be shown on screen
    private boolean showGUI = true;

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
    public ShowMap(int gridHeight, int gridWidth, boolean showGUI) {
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
		int robotCol) {
	  Color c;
	  for (int col = 0; col < grid.length; col++) {
		for (int row = 0; row < grid[0].length; row++) {
		    float value = grid[col][row];
		    // if value is <0 draw a gray pixel
		    // else mapping the value between 0.0 - 1.0 where 0.0 is black
		    // and 1.0 is white
		    if (value < 0) {
			  c = new Color(0.5f, 0.5f, 0.5f);
		    } else {
			  value = Math.abs(value - 1);
			  c = new Color(value, value, value);
		    }
		    // setting pixel color for pixel col and row
		    map.setRGB(row, col, c.getRGB());
		}
	  }
	  // drawing a filled red Rectangle for the robot. Rectangle size is
	  // 6x6
	  Graphics g = map.getGraphics();
	  g.setColor(Color.RED);
	  g.fillRect((int) robotRow - robotSize / 2, (int) robotCol - robotSize
		    / 2, robotSize, robotSize);

	  g.setColor(Color.black);
	  //create a grid (draw a line every 1O pixels) 1 meter by 1 meter
		for (int i = 0; i < grid.length; i++){
			if ((i % 10) == 0 && i != 0) {
				g.drawLine(0, i, imageWidth, i);
			}
		}
		for (int i = 0; i < grid[0].length; i++) {
			if ((i % 10) == 0 && i != 0) {
				g.drawLine(i, 0, i, imageHeight);
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

}
