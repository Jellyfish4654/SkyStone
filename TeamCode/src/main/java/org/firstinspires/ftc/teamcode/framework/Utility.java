package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.util.Calendar;
import java.io.File;

public class Utility {

    static Calendar now = Calendar.getInstance();

    public static double roundTwoDec(double x) {
        x *= 100;
        x = Math.round(x);
        return x / 100;
    }

    public static String getTime() {
        return String.format("%2d:%2d:%2d ", now.get(Calendar.HOUR_OF_DAY), now.get(Calendar.MINUTE),
                now.get(Calendar.SECOND));
    }

    public static double[][] getMovementPositions(File fileName) {
        String fileText = ReadWriteFile.readFile(fileName);
        String inputs[] = fileText.split("~");
        double[][] coordinates = new double[inputs.length][5];
        for (int i = 0; i < inputs.length; i++) {
            String[] params = inputs[i].split(",");
            for (int j = 0; j < params.length; j++) {
                coordinates[i][j] = Double.parseDouble(params[j]);
            }
        }
        return coordinates;
    }
}
