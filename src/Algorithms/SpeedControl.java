package Algorithms;

import Common.Data;
import Common.Point;
import Common.Segment;

public class SpeedControl {
    static class State {
        double time;
        double energy;
        int prevIndex;
        double accelerationUsed;

        public State(double time, double energy, int prevIndex, double accelerationUsed) {
            this.time = time;
            this.energy = energy;
            this.prevIndex = prevIndex;
            this.accelerationUsed = accelerationUsed;
        }
    }


    public Data data;
    public int[][][] rotated_pixels;

    public SpeedControl(Data data, int[][][] rotated_pixels) {
        this.data = data;
        this.rotated_pixels = rotated_pixels;
    }

    public void evaluate(Segment seg) {
        if (seg.length == 2) {
            Point start = seg.points.get(0);
            Point end = seg.points.get(1);
            double slope = data.getHeight(rotated_pixels[end.x][end.y]) - data.getHeight(rotated_pixels[start.x][start.y]);
            double[] acc_range = data.acceleration_range(slope);
            double max_speed = Math.sqrt(2 * data.cell_len * acc_range[0] * (-acc_range[1]) / (acc_range[0] + (-acc_range[1])));
            seg.cost = max_speed / acc_range[0] + max_speed / (-acc_range[1]);
            seg.speed.add(0.0);
            seg.speed.add(0.0);
            seg.acceleration.add(0.0);
            seg.acceleration.add(0.0);
        } else {
            State[][] S = new State[seg.length][data.speed_levels.length];
            for (int i = 0; i < S.length; i++) {
                for (int j = 0; j < S[i].length; j++) {
                    S[i][j] = new State(Double.MAX_VALUE, Double.MAX_VALUE, -1, 0);
                }
            }
            S[0][0] = new State(0, 0, -1, 0);

            for (int i = 1; i < seg.length; i++) {
                for (int uIndex = 0; uIndex < data.speed_levels.length; uIndex++) {
                    double u = data.speed_levels[uIndex];
                    if (S[i - 1][uIndex].time == Double.MAX_VALUE) continue;  // Skip unreachable states
                    for (int vIndex = 0; vIndex < data.speed_levels.length; vIndex++) {
                        double v = data.speed_levels[vIndex];
                        double slope = data.getHeight(rotated_pixels[seg.points.get(i - 1).x][seg.points.get(i - 1).y])
                                - data.getHeight(rotated_pixels[seg.points.get(i).x][seg.points.get(i).y]);
                        double a = (Math.pow(v, 2) - Math.pow(u, 2)) / 2 * data.cell_len;
                        double[] a_range = data.acceleration_range(slope);
                        double[] v_range = data.speed_range(slope);
                        if (v <= v_range[0] && v >= v_range[1] && a <= a_range[0] && a >= a_range[1]) {
                            double t = data.timeConsumption(u, v, slope);
                            double newTime = S[i - 1][uIndex].time + t;
                            double newEnergy = S[i - 1][uIndex].energy + data.computeEnergy(u, a, t);

                            // energy-aware
//                            if (newEnergy < S[i][vIndex].energy) {
                            // time-aware
                            if (newTime < S[i][vIndex].time) {
                                // canDecelerateToZero
                                if (v * v / (2 * data.acceleration_range(0)[1]) <= (seg.length - i) * data.cell_len) {
                                    S[i][vIndex] = new State(newTime, newEnergy, uIndex, a);
                                }
                            }
                        }
                    }
                }
            }

            if (S[seg.length - 1][0].time < Double.MAX_VALUE) {
                seg.cost = S[seg.length - 1][0].time;
                int currentIndex = 0;
                seg.acceleration.add(0.0);
                for (int i = seg.length - 1; i >= 0; i--) {
                    seg.speed.add(0, data.speed_levels[currentIndex]);
                    if (i > 0)
                        seg.acceleration.add(0, S[i][currentIndex].accelerationUsed);
                    currentIndex = S[i][currentIndex].prevIndex;
                }
//                System.out.println(seg);
            } else {
                System.out.println("It is not possible to traverse the track within the given constraints.");
                System.exit(0);
            }
        }
    }

}