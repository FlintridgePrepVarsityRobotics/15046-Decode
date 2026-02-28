package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Projects.newHWmap;

import java.util.List;
@Config
@TeleOp(name = "limelight")
public class limelightTracking extends LinearOpMode {
    public static double TP = 0.01;
    public static double TI = 0.00015;
    public static double TD = 0.00000005;
    PIDController turretpid = new PIDController(TP, TI, TD);
    private Limelight3A limelight;
    public newHWmap robot = new newHWmap();

    final double TICKS_PER_REV = 294.0;      // GoBilda 5202/5203
    final double GEAR_RATIO = 0.3953;        // 34 / 86
    final double MAX_DEGREES = 70;

    final double MIN_POWER_TO_MOVE = 0.05;
    final double BEARING_TOLERANCE = 7.5;    // degrees
    final double TICKS_PER_REV_INTAKE = 101.08;

    double targetTicksPerSec = 0;
    final double PROX_DIhST1 = 7.5;
    final double PROX_DIhST2 = 6.0;
    final double PROX_DIhST3 = 5.5;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.setMsTransmissionInterval(5);
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double distance = getDistance(result.getTa());
                double tx = result.getTx();
                double txnc = result.getTxNC();
                double ty = result.getTy();
                double tync = result.getTyNC();
                telemetry.addLine("eversonisgoat robot sees apritag");

                //shootCODE


                // telemetry.addData("balls are in?", filled);
                //shootEND
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    int apriltagID = fr.getFiducialId();
                    if (apriltagID == 20) {
                        telemetry.addLine("eversonisgoat robot sees correct apritag");

                        double targetX = fr.getTargetXDegrees();
                        double turretpidOutput = turretpid.calculate(0, targetX);

                        double turretfeedforward = 0;
                        double AngleError = -targetX;

                        if (Math.abs(turretpidOutput) > 0.01) {
                            turretfeedforward = Math.signum(turretpidOutput) * MIN_POWER_TO_MOVE;
                        }

                        double dynamicTolerance = 100.0 / distance; //use distance and if not accurate enough, decrease 100.0 justin everson isaac
                        dynamicTolerance = Range.clip(dynamicTolerance, 0.5, 8.0);

                        telemetry.addData("Dynamic Tolerance", dynamicTolerance);

                        double motorPower;

                        if (Math.abs(AngleError) < dynamicTolerance)
                            motorPower = 0;
                        else
                            motorPower = -(turretpidOutput + turretfeedforward);

                        int encoderTicks = robot.turret.getCurrentPosition();
                        double turretDegrees = (encoderTicks / (TICKS_PER_REV / GEAR_RATIO)) * 360.0;

                        if ((turretDegrees >= MAX_DEGREES && motorPower > 0) ||
                                (turretDegrees <= -MAX_DEGREES && motorPower < 0)) {
                            motorPower = 0;
                        }

                        robot.turret.setPower(motorPower);

                    }
                }
            }
        }
    }

public double getDistance (double ta){
    double scale = 10;
    double newDistance = scale / ta;
    return (newDistance);
}
}
