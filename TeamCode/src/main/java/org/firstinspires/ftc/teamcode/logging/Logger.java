package org.firstinspires.ftc.teamcode.logging;

import android.os.Environment;
import java.util.Random;
import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStream;
import java.util.Date;

public class Logger {
    private static OutputStream file;
    static {
        try {
            Date now = new Date();
            String dateString = String.format("%4d-%2d-%2d", now.getYear() + 1900, now.getMonth() + 1, now.getDate());
            String random = Long.toString(new Random().nextLong(), 36);
            File dir = new File(Environment.getExternalStorageDirectory(), ".skystone-logs/" + dateString);
            dir.mkdirs();
            file = new FileOutputStream(new File(dir, random + ".txt"));
        } catch(Exception e) {
            throw new RuntimeException(e.getMessage());
        }
    }

    public static boolean log(String label, Object object) {
        Date now = new Date();
        String line = String.format("%2d:%2d:%2d ", now.getHours(), now.getMinutes(), now.getSeconds());
        if (label.isEmpty()) {
            line += object.toString() + "\n";
        } else {
            line += label + ": " + object.toString() + "\n";
        }

        try {
            file.write(line.getBytes());
            return true;
        } catch(Exception e) {
            return false;
        }
    }
}