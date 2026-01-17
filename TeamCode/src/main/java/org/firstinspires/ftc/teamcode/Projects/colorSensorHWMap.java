package org.firstinspires.ftc.teamcode.Projects;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Config
@TeleOp(name = "ColorSensorTest / Auto Color Detect")
public class colorSensorHWMap extends LinearOpMode {

    colorSensorHWMap robot = new colorSensorHWMap();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    // Tune these if needed
    public static float MIN_SATURATION = 0.3f;
    public static float MIN_VALUE = 0.2f;

    @Override
    public void runOpMode() {

        dashboard.setTelemetryTransmissionInterval(25);

        waitForStart();

        while (opModeIsActive()) {

//            processSensor(
//                    "Sensor 1",
//                    robot.sensor1.red(),
//                    robot.sensor1.green(),
//                    robot.sensor1.blue()
//            );
//
//            processSensor2(
//                    "Sensor 2",
//                    robot.sensor2.red(),
//                    robot.sensor2.green(),
//                    robot.sensor2.blue()
//            );
//
//            processSensor3(
//                    "Sensor 3",
//                    robot.sensor3.red(),
//                    robot.sensor3.green(),
//                    robot.sensor3.blue()
//            );

            telemetry.update();
        }
    }

    /**
     * Processes one color sensor and outputs telemetry
     */
    private String processSensor(String name, int r, int g, int b) {
        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        String detectedColor = classifyColor(hsv);

        telemetry.addLine("---- " + name + " ----");
        telemetry.addData("Red", r);
        telemetry.addData("Green", g);
        telemetry.addData("Blue", b);
        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Sat", hsv[1]);
        telemetry.addData("Val", hsv[2]);
        telemetry.addData("Detected", detectedColor);

        return detectedColor;
    }
    private String processSensor2(String name, int r, int g, int b) {
        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        String detectedColor = classifyColor2(hsv);

        telemetry.addLine("---- " + name + " ----");
        telemetry.addData("Red", r);
        telemetry.addData("Green", g);
        telemetry.addData("Blue", b);
        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Sat", hsv[1]);
        telemetry.addData("Val", hsv[2]);
        telemetry.addData("Detected", detectedColor);

        return detectedColor;
    }
    private String processSensor3(String name, int r, int g, int b) {
        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        String detectedColor = classifyColor3(hsv);

        telemetry.addLine("---- " + name + " ----");
        telemetry.addData("Red", r);
        telemetry.addData("Green", g);
        telemetry.addData("Blue", b);
        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Sat", hsv[1]);
        telemetry.addData("Val", hsv[2]);
        telemetry.addData("Detected", detectedColor);

        return detectedColor;
    }

    /**
     * Classifies color using HSV values
     */
    private String classifyColor(float[] hsv) {

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        if (sat < MIN_SATURATION || val < MIN_VALUE) {
            return "UNKNOWN";
        }

        if (hue >= 110 && hue <= 170) {
            return "GREEN";
        }

// PURPLE / MAGENTA
// Typical purple is ~260–320
        if (hue >= 250 && hue <= 320) {
            return "PURPLE";
        }

        return "UNKNOWN";
    }
    private String classifyColor2(float[] hsv) {

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        if (sat < MIN_SATURATION || val < MIN_VALUE) {
            return "UNKNOWN";
        }

        if (hue >= 110 && hue <= 170) {
            return "GREEN";
        }

// PURPLE / MAGENTA
// Typical purple is ~260–320
        if (hue >= 250 && hue <= 320) {
            return "PURPLE";
        }

        return "UNKNOWN";
    }
    private String classifyColor3(float[] hsv) {

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        if (sat < MIN_SATURATION || val < MIN_VALUE) {
            return "UNKNOWN";
        }

        if (hue >= 110 && hue <= 170) {
            return "GREEN";
        }

// PURPLE / MAGENTA
// Typical purple is ~260–320
        if (hue >= 250 && hue <= 320) {
            return "PURPLE";
        }

        return "UNKNOWN";
    }
}

