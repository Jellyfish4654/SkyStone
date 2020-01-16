package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Calendar;

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
}
