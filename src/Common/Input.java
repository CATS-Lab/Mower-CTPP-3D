package Common;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class Input {
    public Data read_data(String file_name) throws IOException {
        Data data = new Data();
        File file = new File(file_name);
        FileReader fr = new FileReader(file);
        BufferedReader br = new BufferedReader(fr);

        String line;
        line = br.readLine();
        data.rows = Integer.valueOf(line);
        line = br.readLine();
        data.cols = Integer.valueOf(line);
        data.num_node =data.rows * data.cols;
        line = br.readLine();

        data.binary_graph = new boolean[data.rows][data.cols];
        for (int i = 0; i <  data.rows; i++) {
            if ((line = br.readLine()) == null) {
                System.out.println("input error");
                System.exit(0);
            }
            String[] sp = line.split("\\s+");
            for (int j = 0; j < sp.length; j++) {
                if (sp[j].equals("1")) {
                    data.binary_graph[i][j] = false;
                } else {
                    data.binary_graph[i][j] = true;
                }
                if (sp[j].equals("2")) {
                    data.start_node = i * data.rows + j;
                }
            }
        }
        line = br.readLine();

        data.height_graph = new double[data.rows][data.cols];
        for (int i = 0; i < data.rows; i++) {
            if ((line = br.readLine()) == null) {
                System.out.println("input error");
                System.exit(0);
            }
            String[] sp = line.split("\\s+");
            for (int j = 0; j < data.cols; j++) {
                data.height_graph[i][j] = Double.parseDouble(sp[j]);
            }
        }

        return data;
    }
}