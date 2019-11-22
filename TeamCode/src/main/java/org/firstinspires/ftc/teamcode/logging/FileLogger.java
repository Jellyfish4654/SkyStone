package org.firstinspires.ftc.teamcode.logging;

import android.os.Environment;
import java.util.Random;
import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStream;
import java.util.Calendar;

public class FileLogger {
    private static OutputStream file;
    static {
        try {
            Calendar now = Calendar.getInstance();
            String dateString = String.format("%4d-%2d-%2d", now.get(Calendar.YEAR), now.get(Calendar.MONTH), now.get(Calendar.DAY_OF_MONTH));
            String random = Long.toString(new Random().nextLong(), 36);
            File dir = new File(Environment.getExternalStorageDirectory(), ".skystone-logs/" + dateString);
            dir.mkdirs();
            file = new FileOutputStream(new File(dir, random + ".txt"));
        } catch(Exception e) {
            throw new RuntimeException(e.getMessage());
        }
    }

    public static boolean addLine(String line) {
        try {
            file.write(line.getBytes());
            return true;
        } catch(Exception e) {
            return false;
        }
    }

    public static boolean addData(String label, Object object) {
        Calendar now = Calendar.getInstance();
        String line = String.format("%2d:%2d:%2d ", now.get(Calendar.HOUR_OF_DAY), now.get(Calendar.MINUTE), now.get(Calendar.SECOND));
        if (label.isEmpty()) {
            line += object.toString() + "\n";
        } else {
            line += label + ": " + object.toString() + "\n";
        }

        return addLine(line);
    }
}