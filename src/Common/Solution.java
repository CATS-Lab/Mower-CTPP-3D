package Common;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;


public class Solution {
    public Data data;
    public double cost;
    public int turns;
    public double average_speed;
    public double energy;

    public ArrayList<Segment> segSequence;
    public ArrayList<Point> pointSequence;
    public ArrayList<Double> speedSequence;

    public Solution(Data data) {
        this.data = data;
        cost = 0;
        turns = 0;
        segSequence = new ArrayList<>();
        pointSequence = new ArrayList<>();
        speedSequence = new ArrayList<>();
    }

    public Solution(Path path) {
        cost = path.cost;
        turns = path.turns;
        segSequence = new ArrayList<>(path.segments);
        pointSequence = new ArrayList<>(path.points);
        speedSequence = new ArrayList<>();
        for (Segment seg : segSequence) {
            speedSequence.addAll(seg.speed);
            speedSequence.remove(speedSequence.size() - 1);
        }
    }

    public void PrintSolution1(String file_name) {
        try {
            PrintWriter writer = new PrintWriter(file_name + "_solution.txt");
            for (int i = 0; i < pointSequence.size(); i++) {
                writer.println(Integer.toString(pointSequence.get(i).x) + ',' + Integer.toString(pointSequence.get(i).y) + ',' + Math.round(speedSequence.get(i) * 10.0) / 10.0);
            }
            writer.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public void checkAgainstTime(int[][][] best_rotated_pixels) {
        int num = 0;
        for (int i = 0; i < segSequence.size(); i++) {
            Segment seg = segSequence.get(i);
            for (int j = 0; j < seg.points.size() - 1; j++) {
                double slope = data.getHeight(best_rotated_pixels[seg.points.get(j).x][seg.points.get(j).y])
                        - data.getHeight(best_rotated_pixels[seg.points.get(j + 1).x][seg.points.get(j + 1).y]);
                if (slope > 0.31 || slope < -0.31) {
                    num++;
                }
                if ((slope > 0.11 || slope < -0.11) && (seg.acceleration.get(j) > 0.6 || seg.acceleration.get(j) < -1.4)){
                    num++;
                }
            }
        }
        System.out.println(num);
    }

    public void PrintSolution2(String file_name) {
        // calculate the average speed.
        average_speed = 0;
        int num = 0;
        for (int i = 0; i < segSequence.size(); i++) {
            Segment seg = segSequence.get(i);
            for (int j = 0; j < seg.points.size() - 1; j++) {
                average_speed += seg.speed.get(j);
                num++;
            }
            num++;
        }
        average_speed /= num;

        // calculate the total energy consumption
        energy = 0;
        for (int i = 0; i < segSequence.size(); i++) {
            Segment seg = segSequence.get(i);
            for (int j = 0; j < seg.points.size() - 1; j++) {
                energy += data.computeEnergy(seg.speed.get(j), seg.acceleration.get(j), data.timeConsumption(seg.speed.get(j), seg.speed.get(j + 1), 0));
            }
        }

        try {
            PrintWriter writer = new PrintWriter(file_name + "_solution.txt");
            for (int i = 0; i < segSequence.size(); i++) {
                Segment seg = segSequence.get(i);
                for (int j = 0; j < seg.points.size() - 1; j++) {
                    writer.println(Integer.toString(seg.points.get(j).x) + ',' + Integer.toString(seg.points.get(j).y) + ',' + Math.round(seg.speed.get(j) * 10.0) / 10.0);
                }
            }
            writer.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public void MergePath(ArrayList<Path> paths) {
        for (Path path : paths) {
            cost += path.cost;
            turns += path.turns;
            segSequence.addAll(path.segments);
        }
    }
}