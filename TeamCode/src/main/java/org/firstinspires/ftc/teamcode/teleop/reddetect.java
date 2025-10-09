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

@TeleOp(name = "redtele")
public class reddetect extends LinearOpMode {

    public HWMap robot = new HWMap();
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
        int tolerance = 30; // pixels within which the tag is "centered"

        double speed = 1;
        boolean lastUp = false;
        boolean lastDown = false;
        int ticksPerRev = 28;
        double targetRPM = 0;
        double rpmStep = 100;
        double maxRPM = 6000;
        double minRPM = 0;

        waitForStart();

        while (opModeIsActive()) {

            // --- Driver control ---
            double y = -gamepad1.left_stick_y * -1; // forward/backward
            double x = gamepad1.left_stick_x * -1.1; // strafe (counteract drift)
            double rx = gamepad1.right_stick_x * -1; // rotation

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.fLeftWheel.setPower(frontLeftPower * speed);
            robot.bLeftWheel.setPower(backLeftPower * speed);
            robot.fRightWheel.setPower(frontRightPower * speed);
            robot.bRightWheel.setPower(backRightPower * speed);

            // --- Intake Control (A + B) ---
            if (gamepad1.a) {
                if (buttonTimer.seconds() < 0.05) {
                    buttonTimer.reset();
                } else if (buttonTimer.seconds() >= 1.0) {
                    robot.intake.setPower(-0.5);
                    robot.intakeServo.setPower(-0.6);
                } else if (buttonTimer.seconds() < 1.0) {
                    robot.intake.setPower(0.5);
                    robot.intakeServo.setPower(-0.6);
                } else {
                    robot.intake.setPower(0);
                }
            } else {
                buttonTimer.reset();
                robot.intake.setPower(0.0);
            }

            if (gamepad1.b) {
                robot.intake.setPower(0.3);
                robot.intakeServo.setPower(0.4);
                sleep(300);
                robot.intake.setPower(0);
                robot.intakeServo.setPower(0);
            } else {
                robot.intake.setPower(0);
                robot.intakeServo.setPower(0);
            }

            // --- Launcher RPM Control ---
            robot.launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            boolean upPressed = gamepad1.dpad_up;
            boolean downPressed = gamepad1.dpad_down;

            if (upPressed && !lastUp) {
                targetRPM += rpmStep;
                targetRPM = Math.min(targetRPM, maxRPM);
                robot.launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }
            if (downPressed && !lastDown) {
                targetRPM -= rpmStep;
                targetRPM = Math.max(targetRPM, minRPM);
                robot.launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad1.x) {
                targetRPM = -3;
            }

            lastUp = upPressed;
            lastDown = downPressed;

            double targetTicksPerSec = targetRPM / 60.0 * ticksPerRev;
            robot.launcher.setVelocity(targetTicksPerSec);
            double currentRPM = robot.launcher.getVelocity() / ticksPerRev * 60.0;

            // --- AprilTag Centering Mode (Y button) ---
            if (gamepad1.y) {
                if (!tagProcessor.getDetections().isEmpty()) {
                    AprilTagDetection tag = tagProcessor.getDetections().get(0);

                    if (tag.id == 24) {
                        double tagX = tag.center.x;

                        if (tagX < centerX - tolerance) {
                            // Turn left to center
                            robot.fRightWheel.setPower(-0.4);
                            robot.bRightWheel.setPower(-0.4);
                            robot.fLeftWheel.setPower(0.4);
                            robot.bLeftWheel.setPower(0.4);
                            telemetry.addLine("Turning left to center tag");
                        } else if (tagX > centerX + tolerance) {
                            // Turn right to center
                            robot.fRightWheel.setPower(0.4);
                            robot.bRightWheel.setPower(0.4);
                            robot.fLeftWheel.setPower(-0.4);
                            robot.bLeftWheel.setPower(-0.4);
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
