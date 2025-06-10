package Common;

import java.util.ArrayList;

public class Cell {
    public Point topLeft, topRight, bottomLeft, bottomRight;
    public int type; // 0: start cell; 1: BCD cell; 2: dummy obstacle cell

    public ArrayList<Path> paths;

    public Cell(Point topLeft, Point topRight, Point bottomLeft, Point bottomRight, int type) {
        this.topLeft = topLeft;
        this.topRight = topRight;
        this.bottomLeft = bottomLeft;
        this.bottomRight = bottomRight;
        this.type = type;
        this.paths = new ArrayList<>();
    }

    @Override
    public String toString() {
        return "TopLeft: " + topLeft + ", TopRight: " + topRight +
                ", BottomLeft: " + bottomLeft + ", BottomRight: " + bottomRight;
    }
}
