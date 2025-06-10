package Algorithms;

import Common.Data;
import Common.Point;
import Common.Solution;
import ilog.concert.IloException;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

import java.io.FileNotFoundException;
import java.io.PrintStream;

public class MILP {
    public Data data;

    public MILP(Data data) {
        this.data = data;
    }

    public Solution evaluate(String inst_name) {
        try {
            PrintStream fileOut = new PrintStream("./result/" + inst_name + "_model.txt");
            PrintStream console = System.out;
//            System.setOut(fileOut);

            IloCplex cplex = new IloCplex();
            cplex.setParam(IloCplex.Param.TimeLimit, 3600);

            IloNumVar[][] x = new IloNumVar[data.num_dummy_node][data.num_dummy_node];
            IloNumVar[][] y = new IloNumVar[data.num_dummy_node][data.speed_levels.length];
            IloNumVar[][] t = new IloNumVar[data.num_dummy_node][data.num_dummy_node];
            IloNumVar[] u = new IloNumVar[data.num_dummy_node];


            for (int i = 0; i < data.num_dummy_node; i++) {
                x[i] = new IloNumVar[data.num_dummy_node];
                y[i] = new IloNumVar[data.speed_levels.length];
                t[i] = new IloNumVar[data.num_dummy_node];
                for (int j = 0; j < data.num_dummy_node; j++) {
                    x[i][j] = cplex.boolVar("x_" + i + "_" + j);
                    if (j < data.speed_levels.length) {
                        y[i][j] = cplex.boolVar("y_" + i + "_" + j);
                    }
                    t[i][j] = cplex.numVar(0, Double.MAX_VALUE, "t_" + i + "_" + j);
                }
            }
            for (int i = 0; i < data.num_dummy_node; i++) {
                u[i] = cplex.numVar(0, Double.MAX_VALUE, "u_" + i);
            }

            // objective 1
            IloLinearNumExpr objective = cplex.linearNumExpr();
            for (int i = 0; i < data.num_dummy_node; i++) {
                for (int j = 0; j < data.num_dummy_node; j++) {
                    if (data.arc_matrix[i][j])
                        objective.addTerm(1, t[i][j]);
                }
            }
            cplex.addMinimize(objective);

            // constraints 2
            for (int i = 0; i < data.num_dummy_node - 4; i++) {
                if ((int) (i / 4) == data.start_node) continue;
                IloLinearNumExpr expr1 = cplex.linearNumExpr();
                IloLinearNumExpr expr2 = cplex.linearNumExpr();
                boolean is_empty = true;
                for (int j = 0; j < data.num_dummy_node; j++) {
                    if (data.arc_matrix[i][j]) {
                        expr1.addTerm(1, x[i][j]);
                        is_empty = false;
                    }
                    if (data.arc_matrix[j][i]) {
                        expr2.addTerm(1, x[j][i]);
                        is_empty = false;
                    }
                }
                if (!is_empty)
                    cplex.addEq(expr1, expr2);
            }

            // constraints 3
            for (int i = 0; i < (int) (data.num_dummy_node / 4) - 1; i++) {
                if (i == data.start_node) continue;
                IloLinearNumExpr expr3 = cplex.linearNumExpr();
                boolean is_empty = true;
                for (int j = 0; j < 4; j++)
                    for (int k = 0; k < data.num_dummy_node; k++) {
                        if (data.arc_matrix[4 * i + j][k]) {
                            expr3.addTerm(1.0, x[4 * i + j][k]);
                            is_empty = false;
                        }
                    }
                if (!is_empty)
                    cplex.addGe(expr3, 1);
            }

            // constraints 4
            IloLinearNumExpr expr4 = cplex.linearNumExpr();
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < data.num_dummy_node; j++) {
                    if (data.arc_matrix[4 * data.start_node + i][j]) {
                        expr4.addTerm(1.0, x[4 * data.start_node + i][j]);
                    }
                }
            }
            cplex.addEq(expr4, 1);

            IloLinearNumExpr expr5 = cplex.linearNumExpr();
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < data.num_dummy_node; j++) {
                    if (data.arc_matrix[j][data.num_dummy_node - 4 + i]) {
                        expr5.addTerm(1.0, x[j][data.num_dummy_node - 4 + i]);
                    }
                }
            cplex.addEq(expr5, 1);

            // constraints 5
            for (int i = 0; i < data.num_dummy_node; i++) {
                for (int j = 0; j < data.num_dummy_node; j++) {
                    if (data.arc_matrix[i][j]) {
                        IloLinearNumExpr expr6 = cplex.linearNumExpr();
                        expr6.addTerm(1.0, u[i]);
                        expr6.addTerm(-1.0, u[j]);
                        expr6.addTerm(1e5, x[i][j]);
                        cplex.addLe(expr6, 1e5 - 1);
                    }
                }
            }


            // constraints 6
            for (int i = 0; i < data.num_dummy_node; i++) {
                IloLinearNumExpr expr3 = cplex.linearNumExpr();
                for (int k = 0; k < data.speed_levels.length; k++) {
                    expr3.addTerm(1.0, y[i][k]);
                }
                cplex.addLe(expr3, 1);
            }


            // constraints 7
            IloLinearNumExpr expr9 = cplex.linearNumExpr();
            for (int i = 0; i < 4; i++) {
                expr9.addTerm(1, y[4 * data.start_node + i][0]);
            }
            cplex.addEq(expr9, 1);

            IloLinearNumExpr expr10 = cplex.linearNumExpr();
            for (int i = 0; i < 4; i++) {
                expr10.addTerm(1, y[data.num_dummy_node - (4 - i)][0]);
            }
            cplex.addEq(expr10, 1);


            // constraints 8
            for (int i = 0; i < data.num_dummy_node; i++) {
                IloLinearNumExpr expr11 = cplex.linearNumExpr();
                for (int n = 0; n < data.speed_levels.length; n++) {
                    expr11.addTerm(1, y[i][n]);
                }
                for (int j = 0; j < data.num_dummy_node; j++) {
                    if (data.arc_matrix[i][j]) {
                        expr11.addTerm(-1e5, x[i][j]);
                    }
                }
                cplex.addGe(expr11, 1 - 1e5);
            }


            // constraints 9
            for (int i = data.num_dummy_node - 4; i < data.num_dummy_node; i++) {
                IloLinearNumExpr expr11 = cplex.linearNumExpr();
                expr11.addTerm(1, y[i][0]);
                for (int j = 0; j < data.num_dummy_node; j++) {
                    if (data.arc_matrix[j][i]) {
                        expr11.addTerm(-1e5, x[j][i]);
                    }
                }
                cplex.addGe(expr11, 1 - 1e5);
            }


            // constraints 10
            for (int i = 0; i < data.num_dummy_node; i++) {
                IloLinearNumExpr expr11 = cplex.linearNumExpr();
                expr11.addTerm(1, y[i][0]);
                for (int j = 0; j < data.num_dummy_node; j++) {
                    if (data.arc_matrix[i][j]) {
                        if (i % 4 != j % 4)
                            expr11.addTerm(-1e5, x[i][j]);
                    }
                }
                cplex.addGe(expr11, 1 - 1e5);
            }


            // constraints 11
            for (int i = 0; i < data.num_dummy_node; i++) {
                for (int j = 0; j < data.num_dummy_node; j++) {
                    if (data.arc_matrix[i][j]) {
                        for (int n = 0; n < data.speed_levels.length; n++) {
                            for (int m = 0; m < data.speed_levels.length; m++) {
                                double a = (Math.pow(data.speed_levels[m], 2) - Math.pow(data.speed_levels[n], 2)) / 2 * data.cell_len;
                                Point start = data.id2point((int) (i / 4));
                                Point end = data.id2point((int) (j / 4));
                                double slope = data.height_graph[end.x][end.y] - data.height_graph[start.x][start.y];
                                double[] acc_range = data.acceleration_range(slope);
                                double[] v_range = data.speed_range(slope);
                                if (a >= acc_range[0] && a <= acc_range[1] && data.speed_levels[n] <= v_range[0] && data.speed_levels[m] <= v_range[0]) {
                                    IloLinearNumExpr expr8 = cplex.linearNumExpr();
                                    expr8.addTerm(1, y[j][m]);
                                    expr8.addTerm(1e5, x[i][j]);
                                    expr8.addTerm(1e5, y[i][n]);
                                    cplex.addLe(expr8, 2 * 1e5);
                                }
                            }
                        }
                    }
                }
            }


            // constraints 12
            for (int i = 0; i < data.num_dummy_node; i++) {
                for (int j = 0; j < data.num_dummy_node; j++) {
                    if (data.arc_matrix[i][j]) {
                        for (int n = 0; n < data.speed_levels.length; n++) {
                            for (int m = 0; m < data.speed_levels.length; m++) {
                                double time = (2 * data.cell_len) / (data.speed_levels[n] + data.speed_levels[m]);
                                if (data.speed_levels[n] + data.speed_levels[m] < 0.1) {
                                    // cell_len
                                    Point start = data.id2point((int) (i / 4));
                                    Point end = data.id2point((int) (j / 4));
                                    double slope = data.height_graph[end.x][end.y] - data.height_graph[start.x][start.y];
                                    double[] acc_range = data.acceleration_range(slope);
                                    double max_speed = Math.sqrt(2 * data.cell_len * acc_range[0] * (-acc_range[1]) / (acc_range[0] + (-acc_range[1])));
                                    time = max_speed / acc_range[0] + max_speed / (-acc_range[1]);
                                }
                                IloLinearNumExpr expr13 = cplex.linearNumExpr();
                                expr13.addTerm(1, t[i][j]);
                                expr13.addTerm(-1e5, x[i][j]);
                                expr13.addTerm(-1e5, y[i][n]);
                                expr13.addTerm(-1e5, y[j][m]);
                                cplex.addGe(expr13, time + data.cost_matrix[i][j] - 3 * 1e5);
                            }
                        }
                    }
                }
            }

            cplex.exportModel("model.lp");


            if (cplex.solve()) {
                System.out.println("Solution status: " + cplex.getStatus());
                System.out.println("Optimal value: " + cplex.getObjValue());

                for (int i = 0; i < data.num_dummy_node; i++) {
                    for (int j = 0; j < data.num_dummy_node; j++) {
                        if (data.arc_matrix[i][j] && cplex.getValue(x[i][j]) > 0.9)
                            System.out.println("x[" + i + "][" + j + "] = " + cplex.getValue(x[i][j]));
                        try {
                            if (j < data.speed_levels.length && cplex.getValue(y[i][j]) > 0.9) {
                                System.out.println("y[" + i + "][" + j + "] = " + cplex.getValue(y[i][j]));
                            }
                        } catch (IloException ignored) {

                        }
                        if (data.arc_matrix[i][j] && cplex.getValue(t[i][j]) > 0.9)
                            System.out.println("t[" + i + "][" + j + "] = " + cplex.getValue(t[i][j]));
                    }
                }

                for (int i = 0; i < data.num_dummy_node; i++) {
                    try {
                        if (cplex.getValue(u[i]) > 0.9)
                            System.out.println("u[" + i + "] = " + cplex.getValue(u[i]));
                    } catch (IloCplex.UnknownObjectException e) {

                    }
                }

                Solution sol = new Solution(data);
                sol.pointSequence.add(data.id2point(data.start_node));
                sol.speedSequence.add(0.0);
                int before = 0;
                for (int i = 0; i < 4; i++) {
                    if (cplex.getValue(y[data.start_node * 4 + i][0]) > 0.1) {
                        before = data.start_node * 4 + i;
                    }
                }

                int dead = 0;
                loopOut:
                while (true) {
                    for (int i = 0; i < data.num_dummy_node; i++) {
                        if (data.arc_matrix[before][i] && cplex.getValue(x[before][i]) >= 0.9) {
                            sol.pointSequence.add(data.id2point((int) (i / 4)));
                            for (int j = 0; j < data.speed_levels.length; j++) {
                                if (cplex.getValue(y[i][j]) > 0.1) {
                                    sol.speedSequence.add(data.speed_levels[j]);
                                    break;
                                }
                            }
                            before = i;
                            if (i >= data.num_dummy_node - 4)
                                break loopOut;
                            else
                                continue loopOut;
                        }
                    }
                    dead++;
                    if (dead >= 10000) {
                        System.out.println("bad");
                        break loopOut;
                    }
                }

                cplex.end();
                fileOut.close();
                System.setOut(console);
                return sol;

            } else {
                System.out.println("Solution not found.");
            }
        } catch (IloException e) {
            e.printStackTrace();
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        return null;
    }
}
