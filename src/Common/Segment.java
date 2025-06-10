package Common;

import java.util.ArrayList;
import java.util.List;

public class Segment {
    public List<Point> points;
    public int length;
    public double cost;
    public List<Double> speed;
    public List<Double> acceleration;

    public Segment(List<Point> points) {
        this.points = new ArrayList<>(points);
        this.length = points.size();
        this.cost = 0;
        this.speed = new ArrayList<>();
        this.acceleration = new ArrayList<>();
    }

    public Segment(Segment seg) {
        this.points = new ArrayList<>(seg.points);
        this.length = seg.length;
        this.cost = seg.cost;
        this.speed = new ArrayList<>(seg.speed);
        this.acceleration = new ArrayList<>(seg.acceleration);
    }

    @Override
    public String toString() {
        String str = "Segment{" + "cost=" + cost + ", \n";
        for (int i = 0; i < speed.size(); i++) {
            str += "position: " + points.get(i).x + "," + points.get(i).y
                    + ", speed: " + speed.get(i)
                    + ", acceleration: " + acceleration.get(i) + "\n";
        }
        return str;
    }
}
