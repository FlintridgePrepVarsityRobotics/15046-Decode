package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "bluedetect")
public class bluedetect extends LinearOpMode {

    public HWMap robot = new HWMap ();
    public ElapsedTime buttonTimer = new ElapsedTime();

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
        int ticksPerRev = 28;
        double targetRPM = 0;
        double rpmStep = 100;
        double maxRPM = 6000;
        double minRPM = 0;
        boolean intakeOn = false;
        boolean bWasPressed = false;
        waitForStart();

        while (opModeIsActive()) {

            // --- Driver control ---
            double y = -gamepad1.left_stick_y * 1; // forward/backward
            double x = gamepad1.left_stick_x * -1.1; // strafe (counteract drift)
            double rx = gamepad1.right_stick_x * 1; // rotation

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
            robot.launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            boolean highSpeed = gamepad1.dpad_right;
            boolean midSpeed = gamepad1.dpad_up;
            boolean lowSpeed = gamepad1.dpad_left;
            boolean aPressed = false;

            if (highSpeed&& !lastUp) {
                targetRPM =3200;
                targetRPM = Math.min(targetRPM, maxRPM);
                robot.launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }
            if (midSpeed && !lastMid) {
                targetRPM = 2600;
                targetRPM = Math.max(targetRPM, minRPM);
                robot.launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }
            if (lowSpeed && !lastDown) {
                targetRPM =2400;
                targetRPM = Math.max(targetRPM, minRPM);
                robot.launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad1.x){
                targetRPM = -100;
                double targetTicksPerSec = targetRPM / 60.0 * ticksPerRev;
                robot.launcher.setVelocity(targetTicksPerSec);
                double currentRPM = robot.launcher.getVelocity() / ticksPerRev * 60.0;
                telemetry.addData("TargetRPM", targetRPM);
                telemetry.addData("CurrentRPM", currentRPM);
                telemetry.update();
            }
            if (gamepad1.a && !aPressed) {
                // Button was just pressed (not held)
                intakeOn = !intakeOn; // toggle the intake state

                if (intakeOn) {
                    robot.intake.setPower(0.5);
                    robot.intakeServo.setPower(0.6);
                }
                else {
                    robot.intake.setPower(0);
                    robot.intakeServo.setPower(0);
                }
            }

            if(gamepad1.dpad_down){
                robot.intake.setPower(-0.3);
                targetRPM=-1000;
                double targetTicksPerSec = targetRPM / 60.0 * ticksPerRev;
                robot.launcher.setVelocity(targetTicksPerSec);
                double currentRPM = robot.launcher.getVelocity() / ticksPerRev * 60.0;
                telemetry.addData("TargetRPM", targetRPM);
                telemetry.addData("CurrentRPM", currentRPM);
            }
            if (gamepad1.b) {
                // Only reset timer when B is first pressed
                if (!bWasPressed) {
                    buttonTimer.reset();
                    robot.intake.setPower(0.3);
                    robot.intakeServo.setPower(0.4);
                    bWasPressed = true;
                }

                // Stop after 200 ms
                if (buttonTimer.milliseconds() >= 200) {
                    robot.intake.setPower(0);
                    robot.intakeServo.setPower(0);
                }

            } else {
                // Reset for next press
                bWasPressed = false;
                robot.intake.setPower(0);
                robot.intakeServo.setPower(0);
            }


            lastUp = highSpeed;
            lastMid = midSpeed;
            lastDown= lowSpeed;

            double targetTicksPerSec = targetRPM / 60.0 * ticksPerRev;
            robot.launcher.setVelocity(targetTicksPerSec);
            double currentRPM = robot.launcher.getVelocity() / ticksPerRev * 60.0;

            // --- AprilTag Centering Mode (Y button) ---
            if (gamepad1.y) {
                if (!tagProcessor.getDetections().isEmpty()) {
                    AprilTagDetection tag = tagProcessor.getDetections().get(0);

                    if (tag.id == 20) {
                        double tagX = tag.center.x;

                        if (tagX < centerX - tolerance) {
                            // Turn left to center
                            robot.fRightWheel.setPower(0.4);
                            robot.bRightWheel.setPower(0.4);
                            robot.fLeftWheel.setPower(-0.4);
                            robot.bLeftWheel.setPower(-0.4);
                            telemetry.addLine("Turning left to center tag");
                        } else if (tagX > centerX + tolerance) {
                            // Turn right to center
                            robot.fRightWheel.setPower(-0.4);
                            robot.bRightWheel.setPower(-0.4);
                            robot.fLeftWheel.setPower(0.4);
                            robot.bLeftWheel.setPower(0.4);
                            telemetry.addLine("Turning right to center tag");
                        } else {
                            // Tag is centered
                            robot.fRightWheel.setPower(0);
                            robot.bRightWheel.setPower(0);
                            robot.fLeftWheel.setPower(0);
                            robot.bLeftWheel.setPower(0);
                            telemetry.addLine("Tag centered!");
                        }
                        telemetry.addData("Tag X", tagX);
                        telemetry.addData("Center", centerX);
                    } else {
                        telemetry.addLine("Tag detected but not ID 20");
                    }
                } else {
                    telemetry.addLine("No tags detected");
                }
            }

            telemetry.addData("TargetRPM", targetRPM);
            telemetry.addData("CurrentRPM", currentRPM);
            telemetry.update();
        }
    }
}
