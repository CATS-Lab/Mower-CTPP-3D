package Common;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.*;

public class Data {
    //instance message
    public int rows, cols, num_node, start_node;
    public int cell_len = 1;
    public double turning_time = 2.0;
    public double[] speed_levels;
    public boolean[][] binary_graph; // true:grass; false:obstacle
    public double[][] height_graph;
    public double highest;
    public double lowest;

    // floyd algorithm
    public int maxSteps = 30; // 20, 30

    // Simulated annealing
    public double initialTemperature = 100;
    public double coolingRate = 0.99;
    public int maxIterations = 100;

    // directed graph
    public int num_dummy_node; // top: 0; left: 1; right: 2; bottom: 3
    public boolean[][] arc_matrix;
    public double[][] cost_matrix;

    // system parameters
    public Random ran;
    public int seed = 2;

    public Data() {
        ran = new Random();
        speed_levels = new double[8];
        for (int i = 0; i < speed_levels.length; i++)
            speed_levels[i] = 0.5 * i;
    }

    public int[][][] getPixelMatrix() {
        highest = Double.NEGATIVE_INFINITY;
        lowest = Double.POSITIVE_INFINITY;

        for (double[] doubles : height_graph) {
            for (double aDouble : doubles) {
                if (aDouble > highest) {
                    highest = aDouble;
                }
                if (aDouble < lowest) {
                    lowest = aDouble;
                }
            }
        }

        int[][][] pixels = new int[binary_graph.length][binary_graph[0].length][3];
        for (int i = 0; i < binary_graph.length; i++) {
            for (int j = 0; j < binary_graph[0].length; j++) {
                if (binary_graph[i][j]) {
                    double ratio = (height_graph[i][j] - lowest) / (highest - lowest);
                    pixels[i][j][0] = (int) (255 - 255 * ratio);
                    pixels[i][j][1] = 255;
                    pixels[i][j][2] = (int) (255 - 255 * ratio);
                } else {
                    pixels[i][j][0] = 0;
                    pixels[i][j][1] = 0;
                    pixels[i][j][2] = 0;
                }
            }
        }

        return pixels;
    }

    public double getHeight(int[] pixel) {
        double ratio = 1 - pixel[0] / 255.0;
        return ratio * (highest - lowest) + lowest;
    }

    public Point id2point(int currentNode) {
        if (currentNode < num_node)
            return new Point(currentNode / (cols), currentNode % (cols));
        else
            return new Point(0, 0);
    }

    public int point2id(Point p) {
        return p.x * cols + p.y;
    }

    public void obtainDirectedGraph(String inst_name) {
        int[][][] pixels = getPixelMatrix();
        PrintMap(pixels, inst_name);

        num_dummy_node = 4 * (num_node + 1);
        arc_matrix = new boolean[num_dummy_node][num_dummy_node];
        cost_matrix = new double[num_dummy_node][num_dummy_node];
        for (int i = 0; i < num_dummy_node; i++) {
            Arrays.fill(arc_matrix[i], false);
            Arrays.fill(cost_matrix[i], 0);
        }
        for (int i = 0; i < num_dummy_node - 4; i++) {
            int id_i = (int) (i / 4);
            int pos = i % 4;
            Point p_i = id2point(id_i);

            if (!binary_graph[p_i.x][p_i.y])
                continue;

            if (p_i.x - 1 >= 0 && binary_graph[p_i.x - 1][p_i.y]) {
                Point p_top = new Point(p_i.x - 1, p_i.y);
                int node_to_top = 4 * point2id(p_top) + 3;
                arc_matrix[i][node_to_top] = true;
                if (pos != 3 && pos != 0)
                    cost_matrix[i][node_to_top] = turning_time;
            }
            if (p_i.y + 1 < cols && binary_graph[p_i.x][p_i.y + 1]) {
                Point p_right = new Point(p_i.x, p_i.y + 1);
                int node_to_right = 4 * point2id(p_right) + 1;
                arc_matrix[i][node_to_right] = true;
                if (pos != 1 && pos != 2)
                    cost_matrix[i][node_to_right] = turning_time;
            }
            if (p_i.y - 1 >= 0 && binary_graph[p_i.x][p_i.y - 1]) {
                Point p_left = new Point(p_i.x, p_i.y - 1);
                int node_to_left = 4 * point2id(p_left) + 2;
                arc_matrix[i][node_to_left] = true;
                if (pos != 2 && pos != 1)
                    cost_matrix[i][node_to_left] = turning_time;
            }
            if (p_i.x + 1 < rows && binary_graph[p_i.x + 1][p_i.y]) {
                Point p_bottom = new Point(p_i.x + 1, p_i.y);
                int node_to_bottom = 4 * point2id(p_bottom);
                arc_matrix[i][node_to_bottom] = true;
                if (pos != 0 && pos != 3)
                    cost_matrix[i][node_to_bottom] = turning_time;
            }
        }

        for (int i = 0; i < num_dummy_node - 4; i++) {
            for (int j = 0; j < 4; j++) {
                arc_matrix[i][num_dummy_node - (4 - j)] = arc_matrix[i][4 * start_node + j];
                cost_matrix[i][num_dummy_node - (4 - j)] = cost_matrix[i][4 * start_node + j];
                arc_matrix[i][4 * start_node + j] = false;
                cost_matrix[i][4 * start_node + j] = 0;
            }
        }

    }

    public void PrintMap(int[][][] output_img, String file_name) {
        try {
            PrintWriter writer = new PrintWriter("./result/" + file_name + "_map.txt");
            for (int i = 0; i < output_img.length; i++) {
                for (int j = 0; j < output_img[i].length; j++) {
                    writer.print(output_img[i][j][0] + "," + output_img[i][j][1] + "," + output_img[i][j][2] + "\t");
                }
                writer.println();
            }
            writer.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public boolean is_feasible_slope(double slope) {
        if (slope <= 0.31 && slope >= -0.31) {
            return true;
        } else {
            return false;
        }
    }

    public double[] acceleration_range(double slope) {
        double[] array = new double[2];
        if (Math.abs(slope - 0) <= 0.11) {
            array[0] = 1.25;
            array[1] = -2.5;
        } else if (slope <= 0.31 && slope >= -0.31) {
            array[0] = 0.6;
            array[1] = -1.4;
        }

        return array;
    }

    public double[] speed_range(double slope) {
        double[] array = new double[2];
        if (slope <= 0.31) {
            array[0] = 3.5;
            array[1] = 0;
        }

        return array;
    }

    public double timeConsumption(double v1, double v2, double slope) {
        if (v1 == 0 && v2 == 0) {
            double[] acc_range = acceleration_range(slope);
            double max_speed = Math.sqrt(2 * cell_len * acc_range[0] * (-acc_range[1]) / (acc_range[0] + (-acc_range[1])));
            return max_speed / acc_range[0] + max_speed / (-acc_range[1]);
        } else {
            return cell_len / ((v1 + v2) / 2);
        }
    }

    // Constants from the equation
    static final double p1 = -0.2683;
    static final double p2 = 0.06775;
    static final double p3 = 4.55752;
    static final double p4 = 0.03141;
    static final double g = 9.81; // m/s^2
    static final double w = 1; // working width in meters
    static final double d = 0; // working depth in cm
    static final double M = 250; // total mass including tank in kg
    static final double m = 250; // vehicle and implement mass in kg
    static final double r_rc = 0.06;   // rolling resistance coefficient
    static final double P_air = 1;   // air conditioning power in kW

    public double computeEnergy(
            double v,      // speed in km/h
            double a,      // terrain inclination in %
            double t      // time duration in seconds
    ) {
        // Compute each term of power (in kW)
        double term1 = (p1 + v * w * p2);
        double term2 = (p3 + d * v * v * p4) * w;
        double term3 = (0.115 * M * v * a) / 3600.0;
        double term4 = g * m * v * r_rc / 1800.0;

        // Total power required in kW
        double P = term1 + term2 + term3 + P_air + term4;

        // Convert power (kW) and time (s) to energy (kWh)
        double energy_kWh = P * (t / 3600.0);

        return energy_kWh;
    }
}