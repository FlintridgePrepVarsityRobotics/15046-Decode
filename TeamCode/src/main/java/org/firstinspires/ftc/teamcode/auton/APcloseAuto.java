//package org.firstinspires.ftc.teamcode.auton; // make sure this aligns with class location
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import java.lang.Thread;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.ColorRangeSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Projects.newHWmap;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//import java.util.List;
//
//
//
//
//
//import org.firstinspires.ftc.teamcode.Projects.newHWmap;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//
//
//
//import java.util.List;
//
//
//import com.pedropathing.paths.Path;
//@Autonomous(name = "APautoclosered")
//public class APcloseAuto extends OpMode {
//    private static Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//
//    private int pathState;
//    public static class Paths {
//        public Path Shooting;
//        public PathChain Spike2Alignment, Spike2Intake, GateOpen, Scoring2, Spike1Alignment, Spike1Intake, Scoring3, Spike3, Spike3Intake, Scoring4, Park;
//
//        public void buildPaths() {
//
//            Shooting.setLinearHeadingInterpolation(104, 97);
//
//            Spike2Alignment = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(104.000, 97.000),
//                                    new Pose(89.000, 80.000),
//                                    new Pose(100.000, 60.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//
//                    .build();
//
//            Spike2Intake = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(100.000, 60.000),
//
//                                    new Pose(134.000, 60.000)
//                            )
//                    ).setTangentHeadingInterpolation()
//
//                    .build();
//
//            GateOpen = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(134.000, 60.000),
//                                    new Pose(117.000, 58.000),
//                                    new Pose(129.750, 64.500)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//
//                    .build();
//
//            Scoring2 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(129.750, 64.500),
//                                    new Pose(74.000, 70.000),
//                                    new Pose(104.000, 97.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(49))
//
//                    .build();
//
//            Spike1Alignment = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(104.000, 97.000),
//                                    new Pose(94.000, 90.000),
//                                    new Pose(100.000, 84.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//
//                    .build();
//
//            Spike1Intake = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(100.000, 84.000),
//
//                                    new Pose(127.500, 84.000)
//                            )
//                    ).setTangentHeadingInterpolation()
//
//                    .build();
//
//            Scoring3 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(127.500, 84.000),
//
//                                    new Pose(104.000, 97.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(49))
//
//                    .build();
//
//            Spike3 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(104.000, 97.000),
//                                    new Pose(89.000, 66.500),
//                                    new Pose(100.000, 36.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(0))
//
//                    .build();
//
//            Spike3Intake = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(100.000, 36.000),
//
//                                    new Pose(134.000, 36.000)
//                            )
//                    ).setTangentHeadingInterpolation()
//
//                    .build();
//
//            Scoring4 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(134.000, 36.000),
//
//                                    new Pose(104.000, 97.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(49))
//
//                    .build();
//
//            Park = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(104.000, 97.000),
//
//                                    new Pose(110.000, 91.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(66))
//
//                    .build();
//        }
//        public void autonomousPathUpdate() {
//            switch (pathState) {
//                case 0:
//                    follower.followPath(Shooting);
//                    setPathState(1);
//                    break;
//                case 1:
//                    if (!follower.isBusy()) {
//                        follower.followPath(Spike2);
//                        setPathState(1);
//                    }
//
//            }
//        }
//
//
//    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
//    @Override
//    public void loop() {
//
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
//        autonomousPathUpdate();
//
//        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//    }
//
//    /** This method is called once at the init of the OpMode. **/
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//
//    }
//
//    /** This method is called continuously after Init while waiting for "play". **/
//    @Override
//    public void init_loop() {}
//
//    /** This method is called once at the start of the OpMode.
//     * It runs all the setup actions, including building paths and starting the path system **/
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    /** We do not use this because everything should automatically disable **/
//    @Override
//    public void stop() {}
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//
//}
