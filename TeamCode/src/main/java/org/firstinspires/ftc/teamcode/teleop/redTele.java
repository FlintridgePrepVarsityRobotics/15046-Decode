package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;

import android.graphics.Color;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Config
@TeleOp(name = "red")
public class redTele extends LinearOpMode {

    public HWMap robot = new HWMap();
    public ElapsedTime buttonTimer = new ElapsedTime();
    public ElapsedTime colorTimer = new ElapsedTime();
    ElapsedTime reverseTimer = new ElapsedTime();
    boolean reversingLauncher = false;
    boolean reversedBefore = false;

    // PIDF + Feedforward constants (starting values — tune these)
    // These gains are chosen so PIDF+FF outputs a motor power in [-1,1].
    public static double kP = 0.001;
    public static double kI = 0.0006;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // Feedforward: kS (static), kV (velocity), kA (acceleration)
    // kV roughly ~ 1 / (max_ticks_per_sec) as a starting point

    public static double kS = 0.0;
    public static double kV = 0.00042;
    public static double kA = 0.0;

    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        // --- Vision setup ---
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480))
                .build();

        int frameWidth = 640;
        int centerX = frameWidth / 2;
        int tolerance = 50; // pixels within which the tag is centered

        double speed = 1;
        boolean lastUp = false;
        boolean lastMid = false;
        boolean lastDown = false;
        boolean lastX = false;
        boolean bWasPressed = false;
        boolean isIntakeRunning = false;

        boolean intakeFull = false;
        double tagDist = 0;

        boolean color1 = false;
        boolean color2 = false;

        // For A-button toggle
        boolean lastAState = false;

        int ticksPerRev = 28;
        double setpointRPM = 0;
        double targetRPM = 0;

        // Ensure launcher has encoder mode set if you want velocity feedback
        robot.launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ColorSensor sensor1;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        sensor1 = hardwareMap.get(ColorSensor.class, "sensor1");

        // get a reference to the distance sensor that shares the same name

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsv1[] = {0F, 0F, 0F};
        float hsv2[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        waitForStart();

        while (opModeIsActive()) {
            // Creating obj for PID Tuning
            TelemetryPacket packet = new TelemetryPacket();
            pidf.setPIDF(kP, kI, kD, kF);
            feedforward = new SimpleMotorFeedforward(kS, kV, kA);

            // color scale factor init
            Color.RGBToHSV(
                    (int) (robot.sensor1.red() * SCALE_FACTOR),
                    (int) (robot.sensor1.green() * SCALE_FACTOR),
                    (int) (robot.sensor1.blue() * SCALE_FACTOR),
                    hsv1
            );

            Color.RGBToHSV(
                    (int) (robot.sensor2.red() * SCALE_FACTOR),
                    (int) (robot.sensor2.green() * SCALE_FACTOR),
                    (int) (robot.sensor2.blue() * SCALE_FACTOR),
                    hsv2
            );
            telemetry.addData("Red", robot.sensor1.red());
            telemetry.addData("Green", robot.sensor1.green());
            telemetry.addData("Blue", robot.sensor1.blue());
            telemetry.addData("Hue1", hsv1[0]);
            telemetry.addData("Hue2", hsv2[0]);
            float hue1 = hsv1[0];
            float hue2 = hsv2[0];

            boolean tagCentered = false;

            if(hue1 < 30){
                telemetry.addData("Color", "Red");
                color1 = false;
            }

            else if (hue1 < 60) {
                telemetry.addData("Color", "Orange");
                color1 = false;
            }

            else if (hue1 < 140){
                telemetry.addData("Color", "Yellow");
                color1 = false;

            }

            else if (hue1 < 250){ //green --> 160
                telemetry.addData("Color", "Green");
                color1 = true;
            }

            else if (hue1 < 260){
                telemetry.addData("Color", "Blue");
                color1 = false;

            }

            else if (hue1 < 270){ //purple --> 230-250
                telemetry.addData("Color", "Purple");
                color1 = true;
            }

            else{
                telemetry.addData("Color", "Red");
                color1 = false;
            }

            if(hue2 < 30){
                telemetry.addData("Color2", "Red");
                color2 = false;
            }

            else if (hue2 < 60) {
                telemetry.addData("Color2", "Orange");
                color2 = false;
            }

            else if (hue2 < 140){
                telemetry.addData("Color2", "Yellow");
                color2 = false;
            }

            else if (hue2 < 180){ //green --> 160
                telemetry.addData("Color2", "Green");
                color2 = true;
            }

            else if (hue2 < 200){
                telemetry.addData("Color2", "Blue");
                color2 = false;
            }

            else if (hue2 < 250){ //purple --> 230-250
                telemetry.addData("Color2", "Purple");
                color2 = true;
            }

            else{
                telemetry.addData("Color2", "Red");
                color2 = false;
            }

            // --- Driver control ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * -1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.fLeftWheel.setPower(frontLeftPower * speed);
            robot.bLeftWheel.setPower(backLeftPower * speed);
            robot.fRightWheel.setPower(frontRightPower * speed);
            robot.bRightWheel.setPower(backRightPower * speed);

            // --- Launcher RPM Control ---
            boolean highSpeed = gamepad1.dpad_right;
            boolean midSpeed = gamepad1.dpad_up;
            boolean lowSpeed = gamepad1.dpad_left;

            // Update setpoint only when a D-pad button is newly pressed (rising edge),
            // so you don't keep re-setting it each loop.
            if (highSpeed && !lastUp) setpointRPM = 3200;
            if (midSpeed && !lastMid) setpointRPM = 2600;
            if (lowSpeed && !lastDown) setpointRPM = 2400;
            if (gamepad1.x && !lastX) setpointRPM = 0;

            // Measurements in ticks/sec
            double targetTicksPerSec = setpointRPM / 60.0 * ticksPerRev;
            double measuredTicksPerSec = robot.launcher.getVelocity();
            double measuredRPM = measuredTicksPerSec / ticksPerRev * 60.0;

            // Feedforward baseline (returns value in same "command" units as gains —
            // we've chosen gains so this approximates motor power)
            double ffOutput = feedforward.calculate(targetTicksPerSec);

            // PIDF returns correction. Give it the measurement and target (also ticks/sec).
            double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);

            // Combine and clamp to motor power range [-1, 1]
            double combinedOutput = ffOutput + pidOutput;
            combinedOutput = Math.max(-1.0, Math.min(1.0, combinedOutput));

            // If the driver pressed D-pad (we want launcher behavior), apply combined power.
            // If the player pressed 'x' or dpad_down, those override below.
            robot.launcher.setPower(combinedOutput);

            if (color1 && color2){
                if (colorTimer.milliseconds() > 700 && !intakeFull){
                    robot.intake.setPower(0);
                    robot.intakeServo.setPower(0);
                    intakeFull = true;
                    reversingLauncher = true;
                    reverseTimer.reset();
                }
            }
            else{
                colorTimer.reset();
                intakeFull = false;
            }

            if (reversingLauncher && !reversedBefore) {
                robot.launcher.setPower(-0.5);
                robot.intakeServo.setPower(1);
                if (reverseTimer.milliseconds() >= 500) {
                    reversingLauncher = false;
                    robot.intakeServo.setPower(0);
                    reversedBefore = true;
                }
            }

            // --- Dpad down: reverse intake & launcher negative (manual) ---
            if (gamepad1.dpad_down) {
                robot.intake.setPower(-0.3);
                setpointRPM = -1000;
                double targetTicksPerSecDown = targetRPM / 60.0 * ticksPerRev;
                double downPower = feedforward.calculate(targetTicksPerSecDown);
                downPower = Math.max(-1.0, Math.min(1.0, downPower));
                robot.launcher.setPower(downPower);
            }
            // --- A Button -- toggle intake and shuts off when full
            boolean aNow = gamepad1.a;
            if (aNow && !lastAState && !intakeFull) {
                // just pressed
                isIntakeRunning = !isIntakeRunning;
                if (isIntakeRunning) {
                    robot.intake.setPower(0.25);
                    robot.intakeServo.setPower(1);
                    buttonTimer.reset();
                } else {
                    robot.intake.setPower(0);
                    robot.intakeServo.setPower(0);
                }
            }
            lastAState = aNow;

            // --- B button: timed intake pu lse ---
            if (gamepad1.b && Math.abs(measuredRPM - setpointRPM) <= 25) {
                if (!bWasPressed) {
                    buttonTimer.reset();
                    robot.intake.setPower(0.75);
                    robot.intakeServo.setPower(1);
                    bWasPressed = true;
                }
                if (buttonTimer.milliseconds() >= 170) {
                    robot.intake.setPower(0);
                    robot.intakeServo.setPower(0);
                }
            } else if (!isIntakeRunning) {
                bWasPressed = false;
                robot.intake.setPower(0);
                robot.intakeServo.setPower(0);
            }

            // --- Telemetry for tuning ---
            telemetry.addData("Setpoint RPM", setpointRPM);
            packet.put("Setpoint RPM", setpointRPM);
            telemetry.addData("Measured RPM", "%.1f", measuredRPM);
            packet.put("Measured RPM", measuredRPM);
            telemetry.addData("FF Output", "%.4f", ffOutput);
            packet.put("FF Output", ffOutput);
            telemetry.addData("PID Output", "%.4f", pidOutput);
            packet.put("PID Output", pidOutput);
            telemetry.addData("Combined (power)", "%.4f", combinedOutput);
            packet.put("Combined (power)", combinedOutput);

            telemetry.addData("Clear", sensor1.alpha());
            telemetry.addData("Red  ", sensor1.red());
            telemetry.addData("Green", sensor1.green());
            telemetry.addData("Blue ", sensor1.blue());
            telemetry.addData("Hue1", hsv1[0]);
            telemetry.addData("Hue2", hsv2[0]);

            // --- AprilTag Centering (Y button) ---
            if (gamepad1.y) {
                if (!tagProcessor.getDetections().isEmpty()) {
                    AprilTagDetection tag = tagProcessor.getDetections().get(0);
                    if (tag.id == 24) {
                        double tagX = tag.center.x;
                        tagDist = tag.ftcPose.range;
                        telemetry.addData("Distance from Tag", "%.1f", tagDist);
                        centerX = frameWidth / 2;
                        if (tagDist > 100){centerX -= 50;}
                        if (tagX < centerX - tolerance) {
                            robot.fRightWheel.setPower(0.2);
                            robot.bRightWheel.setPower(0.2);
                            robot.fLeftWheel.setPower(-0.2);
                            robot.bLeftWheel.setPower(-0.2);
                            telemetry.addLine("Turning left to center tag");
                        } else if (tagX > centerX + tolerance) {
                            robot.fRightWheel.setPower(-0.2);
                            robot.bRightWheel.setPower(-0.2);
                            robot.fLeftWheel.setPower(0.2);
                            robot.bLeftWheel.setPower(0.2);
                            telemetry.addLine("Turning right to center tag");
                        } else {
                            robot.fRightWheel.setPower(0);
                            robot.bRightWheel.setPower(0);
                            robot.fLeftWheel.setPower(0);
                            robot.bLeftWheel.setPower(0);
                            telemetry.addLine("Tag centered!");
                            tagCentered = true;
                        }
                        telemetry.addData("Tag X", tag.center.x);
                        telemetry.addData("Center", centerX);
                    } else {
                        telemetry.addLine("Tag detected but not ID 24");
                    }
                } else {
                    telemetry.addLine("No tags detected");
                }
            }
// While the robot is looking at the tag once, it will toggle intake off
            if (tagCentered && isIntakeRunning) {
                isIntakeRunning = false;
                robot.intake.setPower(0);
                robot.intakeServo.setPower(0);
            }
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();

            // store previous D-pad states
            lastUp = highSpeed;
            lastMid = midSpeed;
            lastDown = lowSpeed;
        }
    }
}
