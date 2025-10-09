package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "bluedetectplswork")
public class bluedetectplswork extends LinearOpMode {

    public HWMap robot = new HWMap();

    public void leftturn() {
        robot.fRightWheel.setPower(-0.5);
        robot.bRightWheel.setPower(-0.5);
        robot.fLeftWheel.setPower(0.5);
        robot.bLeftWheel.setPower(0.5);
    }

    public void rightturn() {
        robot.fRightWheel.setPower(0.5);
        robot.bRightWheel.setPower(0.5);
        robot.fLeftWheel.setPower(-0.5);
        robot.bLeftWheel.setPower(-0.5);
    }

    public void stopdrive() {
        robot.fRightWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

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

        waitForStart();

        // Image center and tolerance (in pixels)
        int frameWidth = 640;
        int centerX = frameWidth / 2;
        int tolerance = 30;

        while (opModeIsActive() && !isStopRequested()) {
            if (!tagProcessor.getDetections().isEmpty()) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                // Only act when A is pressed
                if (gamepad1.a && tag.id == 20) {
                    double tagX = tag.center.x;

                    if (tagX < centerX - tolerance) {
                        // Tag is left of center
                        leftturn();
                    } else if (tagX > centerX + tolerance) {
                        // Tag is right of center
                        rightturn();
                    } else {
                        // Tag is centered
                        stopdrive();
                    }

                    telemetry.addData("Tag X", tagX);
                    telemetry.addData("Center", centerX);
                }
            } else {
                stopdrive(); // stop if no tags are visible
                telemetry.addLine("No tags detected");
            }

            telemetry.update();
        }
    }
}
