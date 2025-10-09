package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "bluedetect")
public class bluedetect extends LinearOpMode {
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


        while (!isStopRequested() && opModeIsActive()) {

            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                if (gamepad1.a) {
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
