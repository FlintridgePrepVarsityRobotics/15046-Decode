package org.firstinspires.ftc.teamcode.auton;




import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import java.lang.Thread;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;




import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Projects.newHWmap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;




import java.util.List;




@Autonomous(name = "redCloseAuto_NewPaths", group = "fruitauto")
public class redCloseAutoTurret extends OpMode {




    public newHWmap robot = new newHWmap();
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Limelight3A limelight;




    private Timer pathTimer, actionTimer,opmodeTimer;
    private ElapsedTime colorTimer, servoTimer;
    private boolean finishedShooting = false;




    private int pathState;




    private final Pose startPose = new Pose(144-20.050, 122.513, Math.toRadians(180-143));




    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;




    public static double TP = 0.01;
    public static double TI = 0.00015;
    public static double TD = 0.00000005;

    public static double kP = 0.002;

    public static double kI = 0.0;
    public static double kD = 0.00025;
    public static double kF = 0.00042;

    // Feedforward: kS (static), kV (velocity), kA (acceleration)
    // kV roughly ~ 1 / (max_ticks_per_sec) as a starting point

    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    final double TICKS_PER_REV_INTAKE = 146.44;




    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    boolean flywheelReady = false;
    PIDController turretpid = new PIDController(TP, TI, TD);




    final double TICKS_PER_REV_TURRET = 294.0;
    final double GEAR_RATIO_TURRET = 0.3953;
    final double MAX_DEGREES = 70;
    final double MIN_POWER_TO_MOVE = 0.05;




    private boolean shooting = false;
    boolean shotsFired = false;




    // Dynamic RPM Variable
    private double currentFlywheelTargetRPM;
    private static final double FLYWHEEL_ALLOWED_ERR_RPM = 50;
    double startFlywheelTargetRPM = 100;
    private static final double FEED_TIME_SEC = 0.244; //perfect rn but earlier it was bad idk
    private static final double BETWEEN_SHOTS_MIN_SEC = 0.132;




    private boolean feeding = false;
    private double feedStartTime = 0.0;
    private double lastFeedEndTime = -999.0;
    private final int ticksPerRevLauncher = 28;
    double PROX_DIhST = 7.5;








    public double getdistance(double ta){
        double scale = 10;
        double distance = scale/ta;
        return(distance);
    }




    public void startIntake() {
        robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 850 / 60);
        robot.shootServo.setPosition(0.5);
    }




    public void stopIntake() {
        robot.intake.setPower(0);
        robot.shootServo.setPosition(0.5);
    }




    public void startShooting3() {
        shooting = true;
        finishedShooting = false;
        shotsFired = false;
        feeding = false;
        lastFeedEndTime = -999.0;
        actionTimer.resetTimer();
    }




    public void stopShooting() {
        servoTimer.reset();
        robot.shootServo.setPosition(0.5);
        robot.flywheel.setPower(0);
        robot.intake.setPower(0);
        shooting = false;
        feeding = false;
        finishedShooting = true;
    }




    public void updateShooting() {
        double dist2 = ((ColorRangeSensor) robot.sensor2).getDistance(DistanceUnit.CM);
        boolean sense2 = dist2 < PROX_DIhST;

        if (!shooting) {
            feeding = false;
            return;
        }

        if (!sense2) {
            if (colorTimer.seconds() > 2.0) {
                stopShooting();
                return;
            }
        } else {
            colorTimer.reset();
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double distance = getdistance(result.getTa());
            currentFlywheelTargetRPM = (415.2 * Math.log(distance)) + 1173.8 + 50;
        }

        double targetTicksPerSec = currentFlywheelTargetRPM / 60.0 * ticksPerRevLauncher;
        double measuredTicksPerSec = robot.flywheel.getVelocity();

        double ffOutput = feedforward.calculate(targetTicksPerSec);
        double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);

        double launcherPower = Math.max(-1.0, Math.min(1.0, ffOutput + pidOutput));
        robot.flywheel.setPower(launcherPower);

        double measuredRPM = measuredTicksPerSec / ticksPerRevLauncher * 60.0;
        double flywheelErrRPM = Math.abs(measuredRPM - currentFlywheelTargetRPM);

        double now = actionTimer.getElapsedTimeSeconds();
        if (flywheelErrRPM <= FLYWHEEL_ALLOWED_ERR_RPM){
            flywheelReady = true;
        }
        else{
            flywheelReady = false;
        }

        boolean cooldownDone = (now - lastFeedEndTime) >= BETWEEN_SHOTS_MIN_SEC;

        if (!flywheelReady) {
            robot.intake.setPower(0);
            feeding = false;
            robot.shootServo.setPosition(0.5);
            return;
        }

        if (!feeding) {
            if (cooldownDone && sense2 && flywheelReady) {
                feeding = true;
                feedStartTime = now;
            } else {
                robot.intake.setPower(0);
                robot.shootServo.setPosition(0.5);

            }
        }

        if (feeding) {
            robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 1100 / 60);
            robot.shootServo.setPosition(0);

            if ((now - feedStartTime) >= FEED_TIME_SEC) {
                feeding = false;
                lastFeedEndTime = now;

                robot.intake.setPower(0);
                robot.shootServo.setPosition(0.5);
            }
        }
    }

    public void updateTurret() {
        LLResult result = limelight.getLatestResult();




        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            boolean tagFound = false;




            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() == 24) {
                    tagFound = true;
                    double targetX = fr.getTargetXDegrees();
                    double turretpidOutput = turretpid.calculate(0, targetX);




                    double turretfeedforward = 0;
                    if (Math.abs(turretpidOutput) > 0.01) {
                        turretfeedforward = Math.signum(turretpidOutput) * MIN_POWER_TO_MOVE;
                    }




                    double motorPower = -(turretpidOutput + turretfeedforward);




                    int encoderTicks = robot.turret.getCurrentPosition();
                    double turretDegrees = (encoderTicks / (TICKS_PER_REV_TURRET / GEAR_RATIO_TURRET)) * 360.0;




                    if ((turretDegrees >= MAX_DEGREES && motorPower > 0) ||
                            (turretDegrees <= -MAX_DEGREES && motorPower < 0)) {
                        motorPower = 0;
                    }




                    robot.turret.setPower(motorPower);
                    break;
                }
            }
            if (!tagFound) {
                robot.turret.setPower(0);
            }
        } else {
            robot.turret.setPower(0);
        }
    }




    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-20.050, 122.513), new Pose(144-50.086956521739125, 98.60869565217389)))
                .setLinearHeadingInterpolation(Math.toRadians(180-143), Math.toRadians(180-140))
                .build();
        // shoot preload




        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-50.086956521739125, 98.60869565217389), new Pose(144-44.028, 59.729)))
                .setLinearHeadingInterpolation(Math.toRadians(180-143), Math.toRadians(180-180))
                .build();
// shoot to second spike




        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-44.028, 59.729), new Pose(144-12.533, 59.475)))
                .setLinearHeadingInterpolation(Math.toRadians(180-180), Math.toRadians(180-180))
                .build();
// intake second spike
        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-12.533, 59.475), new Pose(144-24.5, 59.351)))
                .setLinearHeadingInterpolation(Math.toRadians(180-180), Math.toRadians(180-180))
                .build();




        Path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-24.5, 59.351), new Pose(144-18.208, 64.693)))
                .setLinearHeadingInterpolation(Math.toRadians(180-180), Math.toRadians(180-180))
                .build();




        Path6 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(144-18.208, 64.693), new Pose(144-53.292, 61.051), new Pose(144-50.086956521739125, 98.60869565217389)))
                .setLinearHeadingInterpolation(Math.toRadians(180-180), Math.toRadians(180-140))
                .build();




        Path7 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(144-50.086956521739125, 98.60869565217389), new Pose(144-60, 77),new Pose(144-15, 77)))
                .setLinearHeadingInterpolation(Math.toRadians(180-140), Math.toRadians(180-180))
                .build();




        Path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-15, 77), new Pose(144-50.086956521739125, 98.60869565217389)))
                .setLinearHeadingInterpolation(Math.toRadians(180-180), Math.toRadians(180-140))
                .build();




        Path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-50.086956521739125, 98.60869565217389), new Pose(144-55.694249649368864, 105.68022440392707)))
                .setLinearHeadingInterpolation(Math.toRadians(180-180), Math.toRadians(180-140))
                .build();
    }




    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                double targetTicksPerSec = startFlywheelTargetRPM / 60.0 * ticksPerRevLauncher;
                double measuredTicksPerSec = robot.flywheel.getVelocity();




                double ffOutput = feedforward.calculate(targetTicksPerSec);
                double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);




                double launcherPower = Math.max(-1.0, Math.min(1.0, ffOutput + pidOutput));
                robot.flywheel.setPower(launcherPower);




                follower.followPath(Path1, true);
                setPathState(1);
                break;




            case 1:
                // 1st artifact launch
                if (!follower.isBusy()) {
                    startShooting3();
                    setPathState(2);
                }
                break;




            case 2:
                if (finishedShooting) {
                    follower.followPath(Path2, true);
                    setPathState(3);
                }
                break;




            case 3:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(Path3, 0.4, true);
                    setPathState(4);
                }
                break;




            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(Path4, true);
                    setPathState(5);
                }
                break;




            case 5:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(Path5, .8, true);
                    setPathState(6);
                }
                break;




            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(Path6, .8, true);
                    targetTicksPerSec = startFlywheelTargetRPM / 60.0 * ticksPerRevLauncher;
                    measuredTicksPerSec = robot.flywheel.getVelocity();




                    ffOutput = feedforward.calculate(targetTicksPerSec);
                    pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);




                    launcherPower = Math.max(-1.0, Math.min(1.0, ffOutput + pidOutput));
                    robot.flywheel.setPower(launcherPower);
                    try {
                        Thread.sleep(500); // Sleep for 1 second
                    } catch (InterruptedException e) {
                        // Handle the exception
                        Thread.currentThread().interrupt(); // Restore the interrupt flag
                    }
                    setPathState(7);
                }
                break;




            case 7:
                // shoot #2 b/w path 6  7
                if (!follower.isBusy()) {
                    startShooting3();
                    setPathState(8);
                }
                break;




            case 8:
                if (finishedShooting) {
                    startIntake();
                    follower.followPath(Path7, .67, true);
                    setPathState(9);
                }
                break;




            case 9:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(Path8, .67, true);
                    setPathState(10);
                }
                break;




            case 10:
                //3rd artifact launch
                if (!follower.isBusy()) {
                    startShooting3();
                    setPathState(11);
                }
                break;




            case 11:
                if (finishedShooting) {
                    follower.followPath(Path9, .67, true);
                    setPathState(12);
                }
                break;




            case 12:
                if (!follower.isBusy()) {
                    robot.turret.setTargetPosition(0);
                    setPathState(-1);
                }
                break;
        }
    }




    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }




    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        updateTurret();




        if (shooting) {
            updateShooting();
        }




        telemetry.addData("path state", pathState);
        telemetry.addData("shots fired", shotsFired);
        telemetry.addData("Target RPM", currentFlywheelTargetRPM);
        telemetry.update();
    }




    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        colorTimer = new ElapsedTime();
        servoTimer = new ElapsedTime();




        opmodeTimer.resetTimer();




        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();




        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);




        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();




        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);




        buildPaths();
        follower.setStartingPose(startPose);
    }




    @Override public void init_loop() {}
    @Override public void start() { opmodeTimer.resetTimer(); setPathState(0); }
    @Override public void stop() {
        limelight.stop();
    }
}
