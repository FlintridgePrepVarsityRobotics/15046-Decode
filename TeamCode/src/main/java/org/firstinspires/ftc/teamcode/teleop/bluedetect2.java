package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Config
@TeleOp(name = "bluedetect2")
public class bluedetect2 extends LinearOpMode {

    public HWMap robot = new HWMap();
    public ElapsedTime buttonTimer = new ElapsedTime();

    // PIDF + Feedforward constants (starting values — tune these)
    // These gains are chosen so PIDF+FF outputs a motor power in [-1,1].
    public static double kP = 0.0003;
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
        int tolerance = 30; // pixels within which the tag is centered

        double speed = 1;
        boolean lastUp = false;
        boolean lastMid = false;
        boolean lastDown = false;
        boolean lastX = false;
        boolean bWasPressed = false;
        boolean isMotorRunning = false;


        // For A-button toggle
        boolean lastAState = false;

        int ticksPerRev = 28;
        double setpointRPM = 0;
        double targetRPM = 0;

        // Ensure launcher has encoder mode set if you want velocity feedback
        robot.launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        waitForStart();

        while (opModeIsActive()) {
            // Creating obj for PID Tuning
            TelemetryPacket packet = new TelemetryPacket();
            pidf.setPIDF(kP, kI, kD, kF);
            feedforward = new SimpleMotorFeedforward(kS, kV, kA);

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

            // --- X button logic (reverse slowly) ---
//            if (gamepad1.x) {
//                // reverse at low speed: -100 RPM example
//                targetRPM = -100;
//                double targetTicksPerSecX = targetRPM / 60.0 * ticksPerRev;
//                // convert to a power using feedforward (and clamp)
//                double revPower = feedforward.calculate(targetTicksPerSecX);
//                revPower = Math.max(-1.0, Math.min(1.0, revPower));
//                robot.launcher.setPower(revPower);
//
//                double currentRPM = robot.launcher.getVelocity() / ticksPerRev * 60.0;
//                telemetry.addData("Reverse Mode", "Active");
//                telemetry.addData("TargetRPM (X)", targetRPM);
//                telemetry.addData("CurrentRPM", currentRPM);
//            }

            // --- Dpad down: reverse intake & launcher negative (manual) ---
            if (gamepad1.dpad_down) {
                robot.intake.setPower(-0.3);
                targetRPM = -1000;
                double targetTicksPerSecDown = targetRPM / 60.0 * ticksPerRev;
                double downPower = feedforward.calculate(targetTicksPerSecDown);
                downPower = Math.max(-1.0, Math.min(1.0, downPower));
                robot.launcher.setPower(downPower);
            }

            // --- Intake Toggle (A button) with rising-edge detection ---
            boolean aNow = gamepad1.a;
            if (aNow && !lastAState) {
                // just pressed
                isMotorRunning = !isMotorRunning;
                if (isMotorRunning) {
                    robot.intake.setPower(0.5);
                    robot.intakeServo.setPower(0.8);
                } else {
                    robot.intake.setPower(0);
                    robot.intakeServo.setPower(0);
                }
            }
            lastAState = aNow;

            // --- B button: timed intake pu lse ---
            if (gamepad1.b && Math.abs(measuredRPM - setpointRPM) <= 100) {
                if (!bWasPressed) {
                    buttonTimer.reset();
                    robot.intake.setPower(0.6);
                    robot.intakeServo.setPower(1);
                    bWasPressed = true;
                }
                if (buttonTimer.milliseconds() >= 150) {
                    robot.intake.setPower(0);
                    robot.intakeServo.setPower(0);
                }
            } else if (!isMotorRunning) {
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

            // --- AprilTag Centering (Y button) ---
            if (gamepad1.y) {
                if (!tagProcessor.getDetections().isEmpty()) {
                    AprilTagDetection tag = tagProcessor.getDetections().get(0);
                    if (tag.id == 20) {
                        double tagX = tag.center.x;
                        double tagZ = tag.ftcPose.z;
                        double tagYaw = tag.ftcPose.yaw;
                        double leftBias = 30;
                        double baseRightBias = 60;
                        double angleBiasAdjustment = 0;
                        double closeRange = 59.87;
                        double midRange = 94.05;

                        if (tagZ < closeRange) {
                            angleBiasAdjustment = tagYaw * 1.5;
                        }

                        double adjustedRightBias = baseRightBias + angleBiasAdjustment;
                        double adjustedLeftCenter = centerX - leftBias;
                        double adjustedRightCenter = centerX + adjustedRightBias;

                        double basePower = 0.4;
                        double turnPower;

                        if (tagZ > midRange) {
                            turnPower = basePower * 0.4;
                        } else if (tagZ >= closeRange) {
                            turnPower = basePower * 0.6;
                        } else {
                            turnPower = basePower;
                        }
                        //greater than 94.05 --> turn left very very slightly
                        //yaw is 0 degrees(facing the apriltag head on) --> don't adjust
                        //yaw is 20 degrees --> turn right very slightly
                        if (tagX < adjustedLeftCenter - tolerance) {
                            // Turn left
                            robot.fRightWheel.setPower(turnPower);
                            robot.bRightWheel.setPower(turnPower);
                            robot.fLeftWheel.setPower(-turnPower);
                            robot.bLeftWheel.setPower(-turnPower);
                        } else if (-40<tagYaw&&tagYaw<-20) {
                            // Turn right
                            robot.fRightWheel.setPower(-turnPower);
                            robot.bRightWheel.setPower(-turnPower);
                            robot.fLeftWheel.setPower(turnPower);
                            robot.bLeftWheel.setPower(turnPower);
                        } else {
                            // Stop
                            robot.fRightWheel.setPower(0);
                            robot.bRightWheel.setPower(0);
                            robot.fLeftWheel.setPower(0);
                            robot.bLeftWheel.setPower(0);
                        }
                        telemetry.addData("Tag ID", tag.id);
                        telemetry.addData("Tag X", tag.center.x);
                        telemetry.addData("Distance (Z)", tagZ);
                        telemetry.addData("Yaw (deg)", tagYaw);
                        telemetry.addData("Turn Power", turnPower);
                        telemetry.addData("Right Bias (base+angle)", adjustedRightBias);
                        telemetry.addData("Angle Bias Adjustment", angleBiasAdjustment);
                    }
                }
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
