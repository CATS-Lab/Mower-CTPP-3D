package Algorithms;

import Common.*;
import ImageProcess.MatrixRotator;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.*;

public class Heuristic {
    public Data data;
    public int best_degree;
    public int[][][] best_rotated_pixels;
    public int[][] BCD_map;
    public int cell_num;
    public ArrayList<Cell> cells;
    public boolean[][] arc_matrix;
    public Path[][] path_matrix;
    public int node_num;
    public int[][] shortestPaths;

    public Heuristic(Data data) {
        this.data = data;
    }

    public void searchDegrees(String inst_name) {
        MatrixRotator rotator = new MatrixRotator();
        best_degree = 0;
        int best_num_segments = Integer.MAX_VALUE;
        for (int degree = 0; degree < 1; degree += 30) {
            int[][][] pixels = data.getPixelMatrix();
            int[][][] rotated_pixels = rotator.rotate(pixels, degree);
            int[][] BCD_map = new int[rotated_pixels.length][rotated_pixels[0].length];
            addDummyObstacle(rotated_pixels, BCD_map);
            int[] results = decompose(rotated_pixels, BCD_map);
            if (results[0] < best_num_segments) {
                best_degree = degree;
                best_num_segments = results[0];
            }
        }

        int[][][] pixels = data.getPixelMatrix();
        best_rotated_pixels = rotator.rotate(pixels, best_degree);
        BCD_map = new int[best_rotated_pixels.length][best_rotated_pixels[0].length];
        addDummyObstacle(best_rotated_pixels, BCD_map);
        int[] results = decompose(best_rotated_pixels, BCD_map);
        cell_num = results[1];
    }

    public int[][] trimEdges(int[][] BCD_map, int[] retract) {
        int top = 0, bottom = 0;
        int left = 0, right = 0;

        // Trim top
        while (isAllMinusOneRow(BCD_map, top)) {
            top++;
        }

        // Trim bottom
        while (isAllMinusOneRow(BCD_map, BCD_map.length - bottom - 1)) {
            bottom++;
        }

        // Trim left
        while (isAllMinusOneColumn(BCD_map, left, 0, BCD_map.length - 1)) {
            left++;
        }

        // Trim right
        while (isAllMinusOneColumn(BCD_map, right, 0, BCD_map.length - 1)) {
            right++;
        }

        top = Math.min(top, left);
        top = Math.max(top - 1, 0);
        bottom = Math.min(bottom, right);
        bottom = Math.max(bottom - 1, 0);

        int[][] trimmedMap = new int[BCD_map.length - bottom - top + 1][BCD_map.length - bottom - top + 1];
        for (int i = top; i <= BCD_map.length - bottom; i++) {
            for (int j = top; j <= BCD_map[0].length - bottom; j++) {
                trimmedMap[i - top][j - top] = BCD_map[i][j];
            }
        }

        retract[0] = top;
        retract[1] = bottom;

        return trimmedMap;
    }

    public boolean isAllMinusOneRow(int[][] BCD_map, int row) {
        for (int j = 0; j < BCD_map[0].length; j++) {
            if (BCD_map[row][j] != -1) {
                return false;
            }
        }
        return true;
    }

    public boolean isAllMinusOneColumn(int[][] BCD_map, int col, int top, int bottom) {
        for (int i = top; i <= bottom; i++) {
            if (BCD_map[i][col] != -1) {
                return false;
            }
        }
        return true;
    }

    public int[][][] trimPixels(int[][][] pixels, int[][] trimmedMap, int retract) {
        int depth = pixels[0][0].length;
        int[][][] trimmedPixels = new int[trimmedMap.length][trimmedMap[0].length][depth];

        for (int i = 0; i < trimmedMap.length; i++) {
            for (int j = 0; j < trimmedMap[0].length; j++) {
                for (int k = 0; k < depth; k++) {
                    trimmedPixels[i][j][k] = pixels[retract + i][retract + j][k];
                }
            }
        }

        return trimmedPixels;
    }

    public void addDummyObstacle(int[][][] pixels, int[][] output_img) {
        int rows = pixels.length;
        int cols = pixels[0].length;

        for (int i = 0; i < rows; i++) {
            Arrays.fill(output_img[i], -1);
        }

        for (int i = 0; i < cols; i++) {
            for (int j = 0; j < rows - 1; j++) {
                if (!isObstacle(pixels[j][i]) && !isObstacle(pixels[j + 1][i])) {
                    if (!data.is_feasible_slope(data.getHeight(pixels[j + 1][i]) - data.getHeight(pixels[j][i]))) {
                        output_img[j][i] = 0; // denote dummy obstacle
                        output_img[j + 1][i] = 0;
                    }
                }
            }
        }
    }

    public int calcConnectivity(boolean[] slice, ArrayList<Integer> connectivePartsStart, ArrayList<Integer> connectivePartsEnd) {
        int connectivity = 0;
        boolean openPart = false;
        int startPoint = 0;

        for (int i = 0; i < slice.length; i++) {
            if (!openPart && slice[i]) {
                openPart = true;
                startPoint = i;
            } else if (openPart && !slice[i]) {
                openPart = false;
                connectivity++;
                connectivePartsStart.add(startPoint);
                connectivePartsEnd.add(i);
            }
        }
        // Handle case where the last segment ends at the last point
        if (openPart) {
            connectivity++;
            connectivePartsStart.add(startPoint);
            connectivePartsEnd.add(slice.length);
        }

        return connectivity;
    }

    public int[][] getAdjacencyMatrix(ArrayList<Integer> partsLeftStart, ArrayList<Integer> partsLeftEnd,
                                      ArrayList<Integer> partsRightStart, ArrayList<Integer> partsRightEnd) {
        int[][] adjacencyMatrix = new int[partsLeftStart.size()][partsRightStart.size()];
        for (int[] matrix : adjacencyMatrix) Arrays.fill(matrix, 0);

        for (int i = 0; i < partsLeftStart.size(); i++) {
            for (int j = 0; j < partsRightStart.size(); j++) {
                int leftPartStart = partsLeftStart.get(i);
                int leftPartEnd = partsLeftEnd.get(i);
                int rightPartStart = partsRightStart.get(j);
                int rightPartEnd = partsRightEnd.get(j);
                if (Math.min(leftPartEnd, rightPartEnd) - Math.max(leftPartStart, rightPartStart) > 0) {
                    adjacencyMatrix[i][j] = 1;
                }
            }
        }
        return adjacencyMatrix;
    }

    public boolean isObstacle(int[] pixels) {
        return pixels[0] == 0 && pixels[1] == 0 && pixels[2] == 0;
    }

    public int[] decompose(int[][][] pixels, int[][] output_img) {
        int rows = pixels.length;
        int cols = pixels[0].length;
        int last_connectivity = 0;
        ArrayList<Integer> last_connectivity_parts_start = new ArrayList<>();
        ArrayList<Integer> last_connectivity_parts_end = new ArrayList<>();
        int current_cell = 0;
        int current_line = 0;
        ArrayList<Integer> current_cells = new ArrayList<>();

        for (int i = 0; i < cols; i++) {
            boolean[] slice = new boolean[rows];
            Arrays.fill(slice, true);
            for (int j = 0; j < rows; j++) {
                if (isObstacle(pixels[j][i]) || output_img[j][i] == 0) { // obstacle or dummy obstacle
                    slice[j] = false;
                }
            }

            ArrayList<Integer> connective_parts_start = new ArrayList<>();
            ArrayList<Integer> connective_parts_end = new ArrayList<>();
            int connectivity = calcConnectivity(slice, connective_parts_start, connective_parts_end);
            current_line += connectivity;

            if (last_connectivity == 0) {
                current_cells = new ArrayList<>();
                for (int j = 0; j < connectivity; j++) {
                    current_cells.add(current_cell);
                    current_cell++;
                }
            } else if (connectivity == 0) {
                current_cells = new ArrayList<>();
                continue;
            } else {
                int[][] adj_matrix = getAdjacencyMatrix(last_connectivity_parts_start, last_connectivity_parts_end, connective_parts_start, connective_parts_end);
                ArrayList<Integer> new_cells = new ArrayList<>();
                ArrayList<Boolean> labels = new ArrayList<>();
                for (int j = 0; j < connective_parts_start.size(); j++) {
                    new_cells.add(0);
                    labels.add(false);
                }
                for (int j = 0; j < adj_matrix.length; j++) {
                    int row_sum = 0;
                    ArrayList<Integer> pos_x = new ArrayList<>();
                    ArrayList<Integer> pos_y = new ArrayList<>();
                    for (int k = 0; k < adj_matrix[j].length; k++) {
                        if (adj_matrix[j][k] == 1) {
                            pos_x.add(j);
                            pos_y.add(k);
                            row_sum++;
                        }
                    }
                    if (row_sum == 1 && !labels.get(pos_y.get(0))) {
                        new_cells.set(pos_y.get(0), current_cells.get(j));
                    } else if (row_sum > 1) {
                        for (int k = 0; k < pos_x.size(); k++) {
                            new_cells.set(pos_y.get(k), current_cell);
                            labels.set(pos_y.get(k), true);
                            current_cell = current_cell + 1;
                        }
                    }
                }

                for (int j = 0; j < adj_matrix[0].length; j++) {
                    int column_sum = 0;
                    ArrayList<Integer> pos_x = new ArrayList<>();
                    ArrayList<Integer> pos_y = new ArrayList<>();
                    for (int k = 0; k < adj_matrix.length; k++) {
                        if (adj_matrix[k][j] == 1) {
                            pos_x.add(k);
                            pos_y.add(j);
                            column_sum++;
                        }
                    }
                    if (column_sum > 1 || column_sum == 0) {
                        if (!labels.get(j)) {
                            new_cells.set(j, current_cell);
                            current_cell = current_cell + 1;
                        }
                    }
                }
                current_cells = new ArrayList<>(new_cells);
            }
            for (int j = 0; j < current_cells.size(); j++) {
                for (int k = connective_parts_start.get(j); k < connective_parts_end.get(j); k++) {
                    output_img[k][i] = current_cells.get(j) + 1; // area begin from 1
                }
            }

            last_connectivity = connectivity;
            last_connectivity_parts_start = new ArrayList<>(connective_parts_start);
            last_connectivity_parts_end = new ArrayList<>(connective_parts_end);
        }

        int[] results = new int[2];
        results[0] = current_line;
        results[1] = current_cell;

        return results;
    }


    public void findAreas() {
        cells = new ArrayList<>();
        // BCD cells
        for (int i = 0; i <= cell_num; i++) {
            cells.add(null);
        }
        for (int i = 0; i < BCD_map.length; i++) {
            for (int j = 0; j < BCD_map[i].length; j++) {
                int num = BCD_map[i][j];
                if (num != -1 && num != 0) {
                    if (cells.get(num) == null)
                        cells.set(num, new Cell(new Point(i, j), new Point(i, j), new Point(i, j), new Point(i, j), 1));
                    else {
                        // Update the corners
                        if (j < cells.get(num).topLeft.y || (i < cells.get(num).topLeft.x && j == cells.get(num).topLeft.y))
                            cells.get(num).topLeft = new Point(i, j);
                        if (j > cells.get(num).bottomRight.y || (i > cells.get(num).bottomRight.x && j == cells.get(num).bottomRight.y))
                            cells.get(num).bottomRight = new Point(i, j);
                        if (j < cells.get(num).bottomLeft.y || (i > cells.get(num).bottomLeft.x && j == cells.get(num).bottomLeft.y))
                            cells.get(num).bottomLeft = new Point(i, j);
                        if (j > cells.get(num).topRight.y || (i < cells.get(num).topRight.x && j == cells.get(num).topRight.y))
                            cells.get(num).topRight = new Point(i, j);
                    }
                }
            }
        }
        // dummy obstacle cells
        for (int row = 0; row < BCD_map.length; row++) {
            int start = -1;
            for (int col = 0; col < BCD_map[row].length; col++) {
                if (BCD_map[row][col] == 0) {
                    if (start == -1) {
                        start = col;
                    }
                } else {
                    if (start != -1) {
                        for (int k = start; k < col; k++) {
                            BCD_map[row][k] = cells.size();
                        }
                        cells.add(new Cell(new Point(row, start), new Point(row, col - 1), new Point(row, start), new Point(row, col - 1), 2));
                        start = -1;
                    }
                }
            }
            // end of the row
            if (start != -1) {
                cells.add(new Cell(new Point(row, start), new Point(row, BCD_map[row].length - 1), new Point(row, start), new Point(row, BCD_map[row].length - 1), 2));
            }
        }
        // start cell
        Point start_node = cells.get(1).topLeft;
        cells.set(0, new Cell(start_node, start_node, start_node, start_node, 0));

        cell_num = cells.size();
    }

    public boolean isInArea(int x, int y, int area_id) {
        if (x < 0 || x >= BCD_map.length)
            return false;
        if (y < 0 || y >= BCD_map[x].length)
            return false;
        if (BCD_map[x][y] != area_id)
            return false;
        return true;
    }

    public int checkNearestRow(int direction, int pos, int column, int[][] map) {
        if (map[pos][column] != -1) {
            if (direction == -1) {
                while (pos - 1 >= 0 && map[pos - 1][column] != -1) {
                    pos = pos - 1;
                }
            } else {
                while (pos + 1 < map.length && map[pos + 1][column] != -1) {
                    pos = pos + 1;
                }
            }
        } else if (map[pos][column] == -1) {
            if (direction == -1) {
                while (pos + 1 < map.length && map[pos][column] == -1) {
                    pos = pos + 1;
                }
            } else {
                while (pos - 1 >= 0 && map[pos][column] == -1) {
                    pos = pos - 1;
                }
            }
        }
        return pos;
    }

    public Path generateOnePath(int area_id, Point start, int mode) {
        Path path = new Path();
        int dx, dy;
        if (mode == 0) {
            dx = 1;
            dy = 1;
        } else if (mode == 1) {
            dx = -1;
            dy = 1;
        } else if (mode == 2) {
            dx = 1;
            dy = -1;
        } else {
            dx = -1;
            dy = -1;
        }

        Point current = new Point(start.x, start.y);
        while (isInArea(current.x, current.y, area_id)) {
            path.addPoint(new Point(current.x, current.y));
            if (isInArea(current.x + dx, current.y, area_id)) {
                current.x += dx;
            } else {
                double height_before = data.getHeight(best_rotated_pixels[current.x][current.y]);
                current.y += dy;
                if (current.y >= 0 && current.y < BCD_map[current.x].length) {
                    current.x = checkNearestRow(dx, current.x, current.y, BCD_map);
                    double height_after = data.getHeight(best_rotated_pixels[current.x][current.y]);
                    double slope = height_after - height_before;
                    if (!data.is_feasible_slope(slope))
                        return null;
                    dx = -dx;
                }
            }
        }

        return path;
    }

    public void generatePaths() {
        cells.get(0).paths.add(new Path());
        for (int i = 1; i < cell_num; i++) {
            cells.get(i).paths.add(generateOnePath(i, cells.get(i).topLeft, 0));
            cells.get(i).paths.add(generateOnePath(i, cells.get(i).topRight, 2));
            if (cells.get(i).type == 1) {
                cells.get(i).paths.add(generateOnePath(i, cells.get(i).bottomLeft, 1));
                cells.get(i).paths.add(generateOnePath(i, cells.get(i).bottomRight, 3));
            }

            for (Path p : cells.get(i).paths) {
                if (p != null)
                    p.getSpeed(data, best_rotated_pixels);
            }
        }
    }

    public final int[][] directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};


    public Path buildPath(Map<Point, Point> prev, Point end) {
        Path path = new Path();
        for (Point at = end; at != null; at = prev.get(at)) {
            path.addPoint(at);
        }
        Collections.reverse(path.points); // Reverse the path to start from the beginning
        return path;
    }

    public Path findShortestPath(int[][] grid, Point start, Point end) {
        int m = grid.length;
        int n = grid[0].length;

        Map<Point, Point> prev = new HashMap<>(); // Store the previous point for each visited point
        PriorityQueue<Point> queue = new PriorityQueue<>(Comparator.comparingDouble(p -> p.cost));
        queue.offer(new Point(start.x, start.y, -1, 0));
        Map<Point, Integer> costSoFar = new HashMap<>();
        costSoFar.put(start, 0);

        while (!queue.isEmpty()) {
            Point current = queue.poll();

            if (current.equals(end)) {
                return buildPath(prev, end); // Found the end point, build and return the path
            }

            for (int i = 0; i < directions.length; i++) {
                int newX = current.x + directions[i][0];
                int newY = current.y + directions[i][1];
                Point next = new Point(newX, newY, i, current.cost + 1);

                if (newX >= 0 && newX < m && newY >= 0 && newY < n && grid[newX][newY] != -1) {
                    double slope = data.getHeight(best_rotated_pixels[newX][newY]) - data.getHeight(best_rotated_pixels[current.x][current.y]);
                    if (data.is_feasible_slope(slope)) {
                        int newCost = costSoFar.get(current) + 1;
                        if (current.direction != -1 && current.direction != i) {
                            newCost += data.turning_time; // Penalty for changing direction
                        }

                        if (!costSoFar.containsKey(next) || newCost < costSoFar.get(next)) {
                            costSoFar.put(next, newCost);
                            next.cost = newCost + heuristic(next, end);
                            queue.offer(next);
                            prev.put(next, current);
//                            System.out.println(prev.size());
                        }
                    }
                }
            }
        }

        return null; // No path found
    }

    public double heuristic(Point a, Point b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

    public boolean[][] getAdjacencyMap() {
        int rows = BCD_map.length;
        int cols = BCD_map[0].length;
        boolean[][] adjacency = new boolean[cell_num][cell_num];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (BCD_map[i][j] != -1) {

                    int current = BCD_map[i][j];

                    if (i > 0 && BCD_map[i - 1][j] != -1 && BCD_map[i - 1][j] != current) {
                        adjacency[current][BCD_map[i - 1][j]] = true;
                        adjacency[BCD_map[i - 1][j]][current] = true;
                    }

                    if (i < rows - 1 && BCD_map[i + 1][j] != -1 && BCD_map[i + 1][j] != current) {
                        adjacency[current][BCD_map[i + 1][j]] = true;
                        adjacency[BCD_map[i + 1][j]][current] = true;
                    }

                    if (j > 0 && BCD_map[i][j - 1] != -1 && BCD_map[i][j - 1] != current) {
                        adjacency[current][BCD_map[i][j - 1]] = true;
                        adjacency[BCD_map[i][j - 1]][current] = true;
                    }

                    if (j < cols - 1 && BCD_map[i][j + 1] != -1 && BCD_map[i][j + 1] != current) {
                        adjacency[current][BCD_map[i][j + 1]] = true;
                        adjacency[BCD_map[i][j + 1]][current] = true;
                    }
                }
            }
        }

        return adjacency;
    }

    public int[][] floydWarshall(boolean[][] adjacency) {
        int INF = 1000000;
        int[][] dist = new int[cell_num][cell_num];

        for (int i = 0; i < cell_num; i++) {
            for (int j = 0; j < cell_num; j++) {
                if (i == j) {
                    dist[i][j] = 0;
                } else if (adjacency[i][j]) {
                    dist[i][j] = 1;
                } else {
                    dist[i][j] = INF;
                }
            }
        }

        // Floyd-Warshall 
        for (int k = 0; k < cell_num; k++) {
            for (int i = 0; i < cell_num; i++) {
                for (int j = 0; j < cell_num; j++) {
                    if (dist[i][j] > dist[i][k] + dist[k][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                    }
                }
            }
        }

        return dist;
    }

    public boolean[][] getReachableMatrix(int[][] shortestPaths) {
        boolean[][] reachable = new boolean[cell_num][cell_num];

        for (int i = 0; i < cell_num; i++) {
            for (int j = 0; j < cell_num; j++) {
                if (shortestPaths[i][j] <= data.maxSteps) {
                    reachable[i][j] = true;
                } else {
                    reachable[i][j] = false;
                }
            }
        }

        return reachable;
    }

    public void obtainDirectedGraph() {
        node_num = 4 * cell_num; // top left, top right, bottom left, bottom right
        arc_matrix = new boolean[node_num][node_num];
        path_matrix = new Path[node_num][node_num];
        for (int i = 0; i < node_num; i++) {
            Arrays.fill(arc_matrix[i], false);
            Arrays.fill(path_matrix[i], null);
        }

        boolean[][] adjacency = getAdjacencyMap();
        Point start_node = cells.get(1).topLeft;

        if (start_node.x < BCD_map.length - 1 && BCD_map[start_node.x + 1][start_node.y] != -1) {
            adjacency[0][BCD_map[start_node.x + 1][start_node.y]] = true;
            adjacency[BCD_map[start_node.x + 1][start_node.y]][0] = true;
        }
        if (start_node.y < BCD_map[0].length - 1 && BCD_map[start_node.x][start_node.y + 1] != -1) {
            adjacency[0][BCD_map[start_node.x][start_node.y + 1]] = true;
            adjacency[BCD_map[start_node.x][start_node.y + 1]][0] = true;
        }
        shortestPaths = floydWarshall(adjacency);
        boolean[][] reachableMatrix = getReachableMatrix(shortestPaths);


        // arc cost within cells
        for (int i = 1; i < cell_num; i++) {
            Cell c1 = cells.get(i);
            if (c1.type == 2) {
                arc_matrix[i * 4][i * 4 + 1] = true;
                path_matrix[i * 4][i * 4 + 1] = c1.paths.get(0);
                arc_matrix[i * 4 + 1][i * 4] = true;
                path_matrix[i * 4 + 1][i * 4] = c1.paths.get(1);
            } else {
                if (c1.paths.get(0) != null && c1.topRight.equals(c1.paths.get(0).getEndPoint())) {
                    arc_matrix[i * 4][i * 4 + 1] = true;
                    path_matrix[i * 4][i * 4 + 1] = c1.paths.get(0);
                } else if (c1.paths.get(0) != null) {
                    arc_matrix[i * 4][i * 4 + 3] = true;
                    path_matrix[i * 4][i * 4 + 3] = c1.paths.get(0);
                }
                if (c1.paths.get(1) != null && c1.topLeft.equals(c1.paths.get(1).getEndPoint())) {
                    arc_matrix[i * 4 + 1][i * 4] = true;
                    path_matrix[i * 4 + 1][i * 4] = c1.paths.get(1);
                } else if (c1.paths.get(1) != null) {
                    arc_matrix[i * 4 + 1][i * 4 + 2] = true;
                    path_matrix[i * 4 + 1][i * 4 + 2] = c1.paths.get(1);
                }
                if (c1.paths.get(2) != null && c1.topRight.equals(c1.paths.get(2).getEndPoint())) {
                    arc_matrix[i * 4 + 2][i * 4 + 1] = true;
                    path_matrix[i * 4 + 2][i * 4 + 1] = c1.paths.get(2);
                } else if (c1.paths.get(2) != null) {
                    arc_matrix[i * 4 + 2][i * 4 + 3] = true;
                    path_matrix[i * 4 + 2][i * 4 + 3] = c1.paths.get(2);
                }
                if (c1.paths.get(3) != null && c1.topLeft.equals(c1.paths.get(3).getEndPoint())) {
                    arc_matrix[i * 4 + 3][i * 4] = true;
                    path_matrix[i * 4 + 3][i * 4] = c1.paths.get(3);
                } else if (c1.paths.get(3) != null) {
                    arc_matrix[i * 4 + 3][i * 4 + 2] = true;
                    path_matrix[i * 4 + 3][i * 4 + 2] = c1.paths.get(3);
                }
            }
        }

        for (int i = 0; i < node_num; i++) {
            for (int j = 0; j < node_num; j++) {
                if (path_matrix[i][j] == null)
                    arc_matrix[i][j] = false;
            }
        }

        // arc cost between cells
        for (int i = 0; i < cell_num; i++) {
            for (int j = 0; j < cell_num; j++) {
                if (i == j) continue;
                if (!reachableMatrix[i][j]) continue;
                Cell c1 = cells.get(i);
                Cell c2 = cells.get(j);
                ArrayList<Point> points1 = new ArrayList<>();
                ArrayList<Point> points2 = new ArrayList<>();
                points1.add(c1.topLeft);
                if (c1.type > 0) {
                    points1.add(c1.topRight);
                }
                if (c1.type == 1) {
                    points1.add(c1.bottomLeft);
                    points1.add(c1.bottomRight);
                }
                points2.add(c2.topLeft);
                if (c2.type > 0) {
                    points2.add(c2.topRight);
                }
                if (c2.type == 1) {
                    points2.add(c2.bottomLeft);
                    points2.add(c2.bottomRight);
                }
                for (int n = 0; n < points1.size(); n++) {
                    for (int m = 0; m < points2.size(); m++) {
                        Path shortestPath = findShortestPath(BCD_map, points1.get(n), points2.get(m));
                        shortestPath.getSpeed(data, best_rotated_pixels);
                        arc_matrix[i * 4 + n][j * 4 + m] = true;
                        path_matrix[i * 4 + n][j * 4 + m] = shortestPath;
                    }
                }
            }
        }
    }


    public double greedySelectPath(ArrayList<Integer> paths) {
        boolean[] isVisited = new boolean[cell_num];
        Arrays.fill(isVisited, false);

        double totalCost = 0;
        int currentCell = 0;
        int currentPoint = 0;
        isVisited[currentCell] = true;
        paths.add(currentPoint);
        while (true) {
            boolean visit_all = true;
            double min_cost = Double.POSITIVE_INFINITY;
            int min_cell_id = 0;
            int min_middle_point_id = 0;
            int min_end_point_id = 0;

            for (int i = 0; i < cell_num; i++) {
                if (isVisited[i])
                    continue; // Skip visited cell

                visit_all = false;

                for (int j = 0; j < 4; j++) {
                    if (arc_matrix[currentPoint][4 * i + j]) {

                        for (int k = 0; k < 4; k++) {
                            if (arc_matrix[4 * i + j][4 * i + k]) {

                                double cost = path_matrix[currentPoint][4 * i + j].cost; // + path_matrix[4 * i + j][4 * i + k].cost;
                                if (cost < min_cost) {
                                    min_cell_id = i;
                                    min_middle_point_id = j;
                                    min_end_point_id = k;
                                    min_cost = cost;
                                }
                            }
                        }
                    }
                }
            }

            if (visit_all)
                break;

            if (min_cost > 1e5) {
                System.out.println("greedy algorithm fails");
                System.exit(0);
            }

            totalCost += path_matrix[currentPoint][4 * min_cell_id + min_middle_point_id].cost
                    + path_matrix[4 * min_cell_id + min_middle_point_id][4 * min_cell_id + min_end_point_id].cost;
            paths.add(4 * min_cell_id + min_middle_point_id);
            paths.add(4 * min_cell_id + min_end_point_id);
            currentPoint = 4 * min_cell_id + min_end_point_id;
            currentCell = min_cell_id;
            isVisited[currentCell] = true;
        }

        // check cost
        double c = 0;
        for (int i = 0; i < paths.size() - 1; i++) {
            c += path_matrix[paths.get(i)][paths.get(i + 1)].cost;
        }
        if (Math.abs(c - totalCost) > 0.0001) {
            System.out.println("GREEDY ERROR");
            System.exit(0);
        }

        return totalCost;
    }


    public Solution simulatedAnnealing() {
        ArrayList<Integer> currentSolution = new ArrayList<>();
        double currentCost = greedySelectPath(currentSolution);
        ArrayList<Integer> bestSolution = new ArrayList<>(currentSolution);
        double bestCost = currentCost;

        double temperature = data.initialTemperature;

        int iteration = 0;
        while (temperature > 1 && iteration < data.maxIterations) {
            ArrayList<Integer> newSolution = new ArrayList<>(currentSolution);
            double newCost = generateNeighbor(currentSolution, newSolution, currentCost);

            if (acceptanceProbability(currentCost, newCost, temperature) > Math.random()) {
                currentSolution = new ArrayList<>(newSolution);
                currentCost = newCost;
            }

            if (currentCost < bestCost) {
                bestSolution = currentSolution;
                bestCost = currentCost;
                System.out.println("Temperature: " + temperature + ", iteration: " + iteration + ", best cost: " + bestCost);
            }
            temperature *= data.coolingRate;
            iteration += 1;
        }

        // check cost
        double c = 0;
        for (int i = 0; i < bestSolution.size() - 1; i++) {
            c += path_matrix[bestSolution.get(i)][bestSolution.get(i + 1)].cost;
        }

        ArrayList<Path> paths = new ArrayList<>();
        for (int i = 0; i < bestSolution.size() - 1; i++) {
            paths.add(new Path(path_matrix[bestSolution.get(i)][bestSolution.get(i + 1)]));
        }
        Solution solution = new Solution(data);
        solution.MergePath(paths);

        solution.checkAgainstTime(best_rotated_pixels);

        return solution;
    }

    private double generateNeighbor(ArrayList<Integer> solution, ArrayList<Integer> newSolution, double cost) {
        double newCost = cost;

        // swap cell
        double min_cost_1 = Double.POSITIVE_INFINITY;
        int index_11 = 0;
        int index_12 = 0;
        for (int i = 1; i < solution.size() - 2; i += 2) {
            for (int j = i + 2; j < solution.size(); j += 2) {
                if (i == j) continue;
                if (!(arc_matrix[solution.get(i - 1)][solution.get(j)]
                        && arc_matrix[solution.get(j + 1)][solution.get(i + 2)]
                        && arc_matrix[solution.get(j - 1)][solution.get(i)]))
                    continue;
                double swapCost = 0;
                if (j == i + 2) {
                    if (j == solution.size() - 2) {
                        swapCost = -path_matrix[solution.get(i - 1)][solution.get(i)].cost
                                - path_matrix[solution.get(i + 1)][solution.get(j)].cost
                                + path_matrix[solution.get(i - 1)][solution.get(j)].cost
                                + path_matrix[solution.get(j + 1)][solution.get(i)].cost;
                    } else if (arc_matrix[solution.get(i + 1)][solution.get(j + 2)]) {
                        swapCost = -path_matrix[solution.get(i - 1)][solution.get(i)].cost
                                - path_matrix[solution.get(i + 1)][solution.get(j)].cost
                                - path_matrix[solution.get(j + 1)][solution.get(j + 2)].cost
                                + path_matrix[solution.get(i - 1)][solution.get(j)].cost
                                + path_matrix[solution.get(j + 1)][solution.get(i)].cost
                                + path_matrix[solution.get(i + 1)][solution.get(j + 2)].cost;
                    }
                } else {
                    if (j == solution.size() - 2) {
                        swapCost = -path_matrix[solution.get(i - 1)][solution.get(i)].cost
                                - path_matrix[solution.get(i + 1)][solution.get(i + 2)].cost
                                - path_matrix[solution.get(j - 1)][solution.get(j)].cost
                                + path_matrix[solution.get(i - 1)][solution.get(j)].cost
                                + path_matrix[solution.get(j + 1)][solution.get(i + 2)].cost
                                + path_matrix[solution.get(j - 1)][solution.get(i)].cost;
                    } else if (arc_matrix[solution.get(i + 1)][solution.get(j + 2)]) {
                        swapCost = -path_matrix[solution.get(i - 1)][solution.get(i)].cost
                                - path_matrix[solution.get(i + 1)][solution.get(i + 2)].cost
                                - path_matrix[solution.get(j - 1)][solution.get(j)].cost
                                - path_matrix[solution.get(j + 1)][solution.get(j + 2)].cost
                                + path_matrix[solution.get(i - 1)][solution.get(j)].cost
                                + path_matrix[solution.get(j + 1)][solution.get(i + 2)].cost
                                + path_matrix[solution.get(j - 1)][solution.get(i)].cost
                                + path_matrix[solution.get(i + 1)][solution.get(j + 2)].cost;
                    }
                }

                if (swapCost < min_cost_1) {
                    min_cost_1 = swapCost;
                    index_11 = i;
                    index_12 = j;
                }
            }
        }

        // change cell
        double min_cost_2 = Double.POSITIVE_INFINITY;
        int index_21 = 0;
        int index_22 = 0;
        int index_23 = 0;
        for (int i = 1; i < solution.size(); i += 2) {
            Cell cell = cells.get(solution.get(i) / 4);
            int cell_id = solution.get(i) - solution.get(i) % 4;
            for (int j = 0; j < cell.paths.size(); j++) {
                Path path = cell.paths.get(j);

                if (path != null && j != solution.get(i) % 4) {
                    if (!arc_matrix[solution.get(i - 1)][cell_id + j]) continue;

                    for (int k = 0; k < cell.paths.size(); k++) {
                        if (arc_matrix[cell_id + j][cell_id + k]) {

                            double changeCost = 0;
                            if (i == solution.size() - 2) {
                                changeCost += -path_matrix[solution.get(i - 1)][solution.get(i)].cost
                                        - path_matrix[solution.get(i)][solution.get(i + 1)].cost
                                        + path_matrix[solution.get(i - 1)][cell_id + j].cost
                                        + path_matrix[cell_id + j][cell_id + k].cost;
                            } else if (arc_matrix[cell_id + k][solution.get(i + 2)]) {
                                changeCost += -path_matrix[solution.get(i - 1)][solution.get(i)].cost
                                        - path_matrix[solution.get(i)][solution.get(i + 1)].cost
                                        - path_matrix[solution.get(i + 1)][solution.get(i + 2)].cost
                                        + path_matrix[solution.get(i - 1)][cell_id + j].cost
                                        + path_matrix[cell_id + j][cell_id + k].cost
                                        + path_matrix[cell_id + k][solution.get(i + 2)].cost;
                            }
                            if (changeCost < min_cost_2) {
                                min_cost_2 = changeCost;
                                index_21 = i;
                                index_22 = cell_id + j;
                                index_23 = cell_id + k;
                            }
                        }
                    }
                }
            }
        }


        if (min_cost_1 < min_cost_2) {
            newCost = cost + min_cost_1;
            Collections.swap(newSolution, index_11, index_12);
            Collections.swap(newSolution, index_11 + 1, index_12 + 1);
        } else {
            newCost = cost + min_cost_2;
            newSolution.set(index_21, index_22);
            newSolution.set(index_21 + 1, index_23);
        }

        return newCost;
    }

    public double acceptanceProbability(double currentCost, double newCost, double temperature) {
        if (newCost < currentCost) {
            return 1.0;
        }
        return Math.exp((currentCost - newCost) / temperature);
    }


    public void PrintRotatedMap(int[][][] output_img, String file_name) {
        try {
            PrintWriter writer = new PrintWriter("./result/" +file_name + "_map.txt");
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
}