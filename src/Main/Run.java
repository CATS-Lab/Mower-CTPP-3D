package Main;

import Algorithms.DFSHeuristic;
import Algorithms.Heuristic;
import Algorithms.MILP;
import Common.*;

import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

public class Run {
    public static void main(String[] args) throws IOException {
        String[] inst_name_list = {
                "3_3_0.3_1.0_0", "4_4_0.3_1.0_0", "5_5_0.3_1.0_0",
                "6_6_0.3_1.0_0", "7_7_0.3_1.0_0", "8_8_0.3_1.0_0",
                "9_9_0.3_1.0_0", "10_10_0.3_1.0_0", "11_11_0.3_1.0_0",
                "12_12_0.3_1.0_0", "13_13_0.3_1.0_0",

//                "50_50_0.32_1.0_0", "50_50_0.32_1.0_1", "50_50_0.32_1.0_2",
//                "50_50_0.32_1.2_0", "50_50_0.32_1.2_1", "50_50_0.32_1.2_2",
//                "50_50_0.35_1.0_0", "50_50_0.35_1.0_1", "50_50_0.35_1.0_2",
//                "50_50_0.35_1.2_0", "50_50_0.35_1.2_1", "50_50_0.35_1.2_2",

//                "50_50_0.3_1.3_0", "50_50_0.3_1.3_1", "50_50_0.3_1.3_2",
//                "50_50_0.3_1.3_3", "50_50_0.3_1.3_4",

//                "100_100_0.32_1.0_0", "100_100_0.32_1.0_1", "100_100_0.32_1.0_2",
//                "100_100_0.32_1.2_0", "100_100_0.32_1.2_1", "100_100_0.32_1.2_2",
//                "100_100_0.35_1.0_0", "100_100_0.35_1.0_1", "100_100_0.35_1.0_2",
//                "100_100_0.35_1.2_0", "100_100_0.35_1.2_1", "100_100_0.35_1.2_2"

//                "125_125_0.32_1.0_0", "125_125_0.32_1.0_1", "125_125_0.32_1.0_2",
        };
        for (String inst_name : inst_name_list) {
            String data_path = "./data/" + inst_name + ".txt";
            Input input = new Input();
            Data data = input.read_data(data_path);
            System.out.println("hello! " + inst_name);

            double t1 = System.nanoTime();


            // exact MILP slover
            data.obtainDirectedGraph(inst_name);
            MILP model = new MILP(data);
            Solution solution = model.evaluate(inst_name);
            solution.PrintSolution1("./result/" + inst_name);


            // heuristic algorithm
            Heuristic decomposition = new Heuristic(data);
            decomposition.searchDegrees(inst_name);
            decomposition.findAreas();
            decomposition.generatePaths();
            decomposition.obtainDirectedGraph();
            Solution solution = decomposition.simulatedAnnealing();
            solution.PrintSolution2("./result/" + inst_name);

            // DFS STC heuristic
//            DFSHeuristic explorer = new DFSHeuristic(data);
//            Solution solution = explorer.run();
//            solution.PrintSolution2("./result/" + inst_name);

            double t2 = System.nanoTime();

            String sum_file = "result/algorithm.csv";
            BufferedWriter sum_out = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(sum_file, true)));
            sum_out.write(inst_name + "," + solution.cost + "," + solution.turns + "," + solution.average_speed + "," + solution.energy + "," + (t2 - t1) / 1e9);
            sum_out.newLine();
            sum_out.close();
        }
    }
}