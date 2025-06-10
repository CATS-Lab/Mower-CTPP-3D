package Common;

import java.util.Objects;

public class Point {
    public int x, y, direction;
    public double cost;

    public Point(int x, int y) {
        this.x = x;
        this.y = y;
        this.direction = -1; // No initial direction
        this.cost = 0;
    }

    public Point(int x, int y, int direction, double cost) {
        this.x = x;
        this.y = y;
        this.direction = direction;
        this.cost = cost;
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof Point)) return false;
        Point point = (Point) obj;
        return x == point.x && y == point.y;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }
}
