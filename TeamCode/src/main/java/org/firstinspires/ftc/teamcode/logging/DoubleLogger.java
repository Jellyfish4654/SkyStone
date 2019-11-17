package org.firstinspires.ftc.teamcode.logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Date;

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

    public void update() {
        Date now = new Date();
        String timestamp = String.format("%2d:%2d:%2d", now.getHours(), now.getMinutes(), now.getSeconds());
        telemetry.update();
        FileLogger.addLine("=== " + timestamp + " UPDATE ===");
    }
}