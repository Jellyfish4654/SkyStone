package org.firstinspires.ftc.teamcode.logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Calendar;

/** Logs both to Telemetry and to the filesystem. */
public class DoubleLogger {
    Telemetry telemetry;

    public DoubleLogger(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void addData(String label, Object o) {
        telemetry.addData(label, o);
        FileLogger.addData(label, o);
    }

    public void addData(String label, String format, Object... args) {
        this.addData(label, String.format(format, args));
    }

    public void addDataUpdate(String label, Object o) {
        telemetry.addData(label, o);
        FileLogger.addData(label, o);
        telemetry.update();
    }

    public void addDataUpdate(String label, String format, Object... args) {
        this.addData(label, String.format(format, args));
    }

    public void update() {
        telemetry.update();

        Calendar now = Calendar.getInstance();
        String timestamp = String.format("%2d:%2d:%2d", now.get(Calendar.HOUR_OF_DAY), now.get(Calendar.MINUTE),
                now.get(Calendar.SECOND));
        FileLogger.addLine("=== " + timestamp + " UPDATE ===\n");

    }
}