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
            if (highSpeed && !lastUp) setpointRPM = 3000;
            if (midSpeed && !lastMid) setpointRPM = 2600;
            if (lowSpeed && !lastDown) setpointRPM = 2400;

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
            if (gamepad1.x) {
                // reverse at low speed: -100 RPM example
                targetRPM = -100;
                double targetTicksPerSecX = targetRPM / 60.0 * ticksPerRev;
                // convert to a power using feedforward (and clamp)
                double revPower = feedforward.calculate(targetTicksPerSecX);
                revPower = Math.max(-1.0, Math.min(1.0, revPower));
                robot.launcher.setPower(revPower);

                double currentRPM = robot.launcher.getVelocity() / ticksPerRev * 60.0;
                telemetry.addData("Reverse Mode", "Active");
                telemetry.addData("TargetRPM (X)", targetRPM);
                telemetry.addData("CurrentRPM", currentRPM);
            }

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

            // --- B button: timed intake pulse ---
            if (gamepad1.b) {
                if (!bWasPressed) {
                    buttonTimer.reset();
                    robot.intake.setPower(0.3);
                    robot.intakeServo.setPower(0.6);
                    bWasPressed = true;
                }
                if (buttonTimer.milliseconds() >= 200) {
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
                        if (tagX < centerX - tolerance) {
                            robot.fRightWheel.setPower(0.4);
                            robot.bRightWheel.setPower(0.4);
                            robot.fLeftWheel.setPower(-0.4);
                            robot.bLeftWheel.setPower(-0.4);
                            telemetry.addLine("Turning left to center tag");
                        } else if (tagX > centerX + tolerance) {
                            robot.fRightWheel.setPower(-0.4);
                            robot.bRightWheel.setPower(-0.4);
                            robot.fLeftWheel.setPower(0.4);
                            robot.bLeftWheel.setPower(0.4);
                            telemetry.addLine("Turning right to center tag");
                        } else {
                            robot.fRightWheel.setPower(0);
                            robot.bRightWheel.setPower(0);
                            robot.fLeftWheel.setPower(0);
                            robot.bLeftWheel.setPower(0);
                            telemetry.addLine("Tag centered!");
                        }
                        telemetry.addData("Tag X", tag.center.x);
                        telemetry.addData("Center", centerX);
                    } else {
                        telemetry.addLine("Tag detected but not ID 20");
                    }
                } else {
                    telemetry.addLine("No tags detected");
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
