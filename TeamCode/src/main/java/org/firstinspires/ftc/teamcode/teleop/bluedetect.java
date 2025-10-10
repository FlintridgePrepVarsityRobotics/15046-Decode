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
    public ElapsedTime buttonTimer = new ElapsedTime();
    public void leftturn() {
        robot.fRightWheel.setPower(-.5);
        robot.bRightWheel.setPower(-.5);
        robot.fLeftWheel.setPower(.5);
        robot.bLeftWheel.setPower(.5);
    }

    public void rightturn() {
        robot.fRightWheel.setPower(.5);
        robot.bRightWheel.setPower(.5);
        robot.fLeftWheel.setPower(-.5);
        robot.bLeftWheel.setPower(-.5);
    }

    public void stopdrive() {
        robot.fRightWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
    }

    public HWMap robot = new HWMap();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
//            double wheelDiameter = 4.09449;
//            double circumference = Math.PI * wheelDiameter;
//            int ticksPerRevolution = 28;
//            double inchespertick = circumference/ticksPerRevolution;
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480)).build();

        waitForStart();
        double speed = 1;
        waitForStart();
        boolean isSpinning = false;
        int ticksPerRev = 28;
        double targetRPM = 0;
        double rpmStep = 100;
        double maxRPM = 6000;
        double minRPM = 0;
        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastMid = false;
        while (opModeIsActive()) {
            boolean aButtonHeld = false;
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.fLeftWheel.setPower(frontLeftPower * speed);
            robot.bLeftWheel.setPower(backLeftPower * speed);
            robot.fRightWheel.setPower(frontRightPower * speed);
            robot.bRightWheel.setPower(backRightPower * speed);

            if (gamepad1.a){
                if (buttonTimer.seconds() < 0.05) {
                    buttonTimer.reset();
                }
                else if (buttonTimer.seconds() >= 1.0) {
                    robot.intake.setPower(-0.5);
                    robot.intakeServo.setPower(-0.6);

                }

                else {
                    robot.intake.setPower(0);
                }
            }
            else {
                buttonTimer.reset();
                robot.intake.setPower(0.0);
            }
            if (gamepad1.dpad_down){
                robot.intake.setPower(0.3);
                targetRPM=-1000;
                sleep(250);
                robot.intake.setPower(0);
                targetRPM = 0;
            }
            if (gamepad1.b){
                robot.intake.setPower(-0.3);
                targetRPM=-1000;
                sleep(250);
                robot.intake.setPower(0);
                targetRPM = 0;
            }
            else{
                robot.intake.setPower(0);
            }
            robot.launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            boolean highSpeed= gamepad1.dpad_right;
            boolean midSpeed = gamepad1.dpad_up;
            boolean lowSpeed = gamepad1.dpad_left;

            if (highSpeed && !lastUp) {
                targetRPM =3200;
                targetRPM = Math.min(targetRPM, maxRPM);
                robot.launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }
            if (midSpeed && !lastMid) {
                targetRPM =2600;
                targetRPM = Math.min(targetRPM, maxRPM);
                robot.launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }

            if (lowSpeed && !lastDown) {
                targetRPM = 2400;
                targetRPM = Math.max(targetRPM, minRPM);
                robot.launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad1.x){
                targetRPM = -3;
                sleep(500);
                targetRPM = 0;
            }


            lastUp = highSpeed;
            lastMid = midSpeed;
            lastDown = lowSpeed
            ;
            //

            double targetTicksPerSec = targetRPM / 60.0 * ticksPerRev;

            robot.launcher.setVelocity(targetTicksPerSec);
            double currentRPM = robot.launcher.getVelocity() / ticksPerRev * 60.0;

            telemetry.addData("TargetRPM", targetRPM);
            telemetry.addData("CurrentRPM", currentRPM);
            telemetry.update();
        }



        while (!isStopRequested() && opModeIsActive()) {

            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                if (gamepad1.y) {
                    if (tag.id == 20 && tag.ftcPose.yaw < -5) {
                        rightturn();
                    } else if (tag.id == 20 && tag.ftcPose.yaw > 5) {
                        leftturn();
                    } else {
                        leftturn();
                    }
                    if (tag.id == 20 && tag.ftcPose.yaw >= -1 && tag.ftcPose.yaw <= 1) {
                        stopdrive();
                    }
                }

                if (tag.id == 20 && tag.ftcPose.yaw >= -1 && tag.ftcPose.yaw <= 1) {
                    stopdrive();
                }
                    if (tag.id == 20 && tag.ftcPose.yaw >= -1 && tag.ftcPose.yaw <= 1) {
                        stopdrive();
                    }
            }


            telemetry.update();
        }
    }
}
