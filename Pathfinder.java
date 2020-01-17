
public class Pathfinder {

    enum Dir {

        NORTH,
        EAST,
        SOUTH,
        WEST
    }

    private Dir[][] heatMap;


    public void findPath(Point start, Point goal, double[][] map) {
        heatMap = initHeatMap(map);
    }



    private Dir[][] initHeatMap(double[][] map) {
        return new Dir[map.length][map[0].length];
    }

    private int x2Grid(int xValue, int gridWidth) {
        return (xValue / gridWidth);
    }

    private int y2Grid(int yValue, int gridHeight) {
        return (yValue / gridHeight);
    }

    private int Grid2x(int column, int gridWidth) {
        return (column * gridWidth + (gridWidth / 2));
    }

    private int Grid2y(int row, int gridHeight) {
        return (row * gridHeight + (gridHeight / 2));
    }
}
