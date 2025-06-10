package Common;

import Algorithms.SpeedControl;

import java.util.ArrayList;
import java.util.List;


public class Path {
    public List<Point> points;
    ArrayList<Segment> segments;
    public int turns;
    public int length;
    public double cost;

    public Path() {
        points = new ArrayList<>();
        segments = new ArrayList<>();
        turns = 0;
        length = 0;
        cost = 0;
    }

    public Path(Path p) {
        points = new ArrayList<>(p.points);
        segments = new ArrayList<>();
        for (Segment seg : p.segments) {
            segments.add(new Segment(seg));
        }
        turns = p.turns;
        length = p.length;
        cost = p.cost;
    }

    public void addPoint(Point point) {
//        if (!points.isEmpty()) {
//            // Check if there's a turn
//            Point lastPoint = points.get(points.size() - 1);
//            if ((lastPoint.x != point.x) && (lastPoint.y != point.y)) {
//                turns++;
//            }
//            length++;
//        }
        length++;
        points.add(point);
    }

    // Calculate and return the number of turns in the path
    public void getSegmentsAndTurns() {
        segments = new ArrayList<>();
        ArrayList<Point> seg_points = new ArrayList<>();
        seg_points.add(points.get(0));
        for (int i = 2; i < points.size(); i++) {
            Point prev = points.get(i - 2);
            Point curr = points.get(i - 1);
            Point next = points.get(i);
            seg_points.add(curr);
            if ((prev.x != next.x) && (prev.y != next.y)) {
                segments.add(new Segment(seg_points));
                seg_points = new ArrayList<>();
                seg_points.add(curr);
            }
        }
        seg_points.add(points.get(points.size() - 1));
        segments.add(new Segment(seg_points));

        turns = segments.size();
    }


    public void getSpeed(Data data, int[][][] rotated_pixels) {
        cost = 0;
        SpeedControl sc = new SpeedControl(data, rotated_pixels);
        getSegmentsAndTurns();
        for (Segment seg : segments) {
            sc.evaluate(seg);
            cost += seg.cost;
        }
        cost += data.turning_time * turns;
    }

    public Point getEndPoint() {
        return points.get(points.size() - 1);
    }

    public Point getStartPoint() {
        return points.get(0);
    }

    @Override
    public String toString() {
        return "Length: " + length + ", Turns: " + turns + ", Points: " + points;
    }
}
