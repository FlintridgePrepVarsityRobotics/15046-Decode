package org.firstinspires.ftc.teamcode.Projects;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;


@TeleOp(name = "turretTrackerOnly")

public class turretTrackerOnly extends LinearOpMode {
    private Limelight3A limelight;

    public turretHWMap robot = new turretHWMap();

    final double TICKS_PER_REV = 294.0;      // GoBilda 5202/5203
    final double GEAR_RATIO = 0.3953;        // 34 / 86
    final double MAX_DEGREES = 90.0;

    final double kP = 0.02;
    final double MIN_POWER_TO_MOVE = 0.05;
    final double BEARING_TOLERANCE = 1.5;    // degrees

    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tagProcessor = new AprilTagProcessor.Builder().build();



        telemetry.addLine("Turret Tracker Initialized");
        telemetry.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
//yeet
        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            for (FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();
            }
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                int encoderTicks = robot.turret.getCurrentPosition();
                double turretDegrees =
                        (encoderTicks / (TICKS_PER_REV / GEAR_RATIO)) * 360.0;

                double desiredTurretAngle = turretDegrees + tx;
                desiredTurretAngle = Range.clip(
                        desiredTurretAngle,
                        -MAX_DEGREES,
                        MAX_DEGREES
                );

                double angleError = desiredTurretAngle - turretDegrees;
                double motorPower = 0.0;

                if (Math.abs(angleError) > BEARING_TOLERANCE) {
                    motorPower = angleError * kP;

                    if (Math.abs(motorPower) < MIN_POWER_TO_MOVE) {
                        motorPower = Math.signum(motorPower) * MIN_POWER_TO_MOVE;
                    }
                }

                motorPower = Range.clip(motorPower, -1.0, 1.0);
                robot.turret.setPower(motorPower);

                telemetry.addData("Turret Angle (deg)", "%.2f", turretDegrees);
                telemetry.addData("Desired Angle (deg)", "%.2f", desiredTurretAngle);
                telemetry.addData("Angle Error (deg)", "%.2f", angleError);
                telemetry.addData("Motor Power", "%.2f", motorPower);
                telemetry.addData("Encoder Ticks", encoderTicks);
                telemetry.addData("Target X", tx);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            limelight.stop();



            telemetry.update();
        }

        visionPortal.close();
    }

    public double getAprilTagBearing() {

        if (tagProcessor.getDetections().isEmpty()) {
            return 0.0;
        }

        AprilTagDetection tag = tagProcessor.getDetections().get(0);

        if (tag.id != 22) { //blue or red
            return 0.0;
        }

        return tag.ftcPose.bearing;
    }
}
