//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.Projects.turretHWMap;
//
//
//@TeleOp(name = "newLauncherTest")
//public class newLauncherTest extends LinearOpMode {
//    public turretHWMap robot = new turretHWMap();
//    final double TICKS_PER_REV = 294;
//    final double GEAR_RATIO = .3953; // 34/86
//    public static double kP = 0.001;
//    public static double kI = 0.0006;
//    public static double kD = 0.0;
//    public static double kF = 0.0;
//
//    // Feedforward: kS (static), kV (velocity), kA (acceleration)
//    // kV roughly ~ 1 / (max_ticks_per_sec) as a starting point
//
//    public static double kS = 0.0;
//    public static double kV = 0.00042;
//    public static double kA = 0.0;
//
//    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
//    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        boolean lastUp = false;
//        boolean lastMid = false;
//        boolean lastDown = false;
//        boolean lastX = false;
//        int ticksPerRev = 28;
//        double setpointRPM = 0;
//        double targetRPM = 0;
//        boolean flywheelon = false;
//
//
//        robot.init(hardwareMap);
//
//        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        telemetry.addData("Status", "Initialized, turret pos reset to 0.");
//        telemetry.update();
//
//        robot.launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        waitForStart();
//
//        while (opModeIsActive()){
//
//            // --- Launcher RPM Control ---
//            boolean highSpeed = gamepad1.dpad_right;
//            boolean midSpeed = gamepad1.dpad_up;
//            boolean lowSpeed = gamepad1.dpad_left;
//
//            // Update setpoint only when a D-pad button is newly pressed (rising edge),
//            // so you don't keep re-setting it each loop.
//            if (highSpeed && !lastUp){
//                setpointRPM = 3200;
//                flywheelon = true;
//            }
//            if (midSpeed && !lastMid){
//                setpointRPM = 2600;
//                flywheelon = true;
//            }
//            if (lowSpeed && !lastDown){
//                setpointRPM = 2400;
//                flywheelon = true;
//            }
//            if (gamepad1.x && !lastX){
//                setpointRPM = 0;
//                flywheelon = false;
//            }
//            lastUp = highSpeed;
//            lastMid = midSpeed;
//            lastDown = lowSpeed;
//
//            double targetTicksPerSec = setpointRPM / 60.0 * ticksPerRev;
//            double measuredTicksPerSec = robot.launcher.getVelocity();
//            double measuredRPM = measuredTicksPerSec / ticksPerRev * 60.0;
//
//            // Feedforward baseline (returns value in same "command" units as gains —
//            // we've chosen gains so this approximates motor power)
//            double ffOutput = feedforward.calculate(targetTicksPerSec);
//
//            // PIDF returns correction. Give it the measurement and target (also ticks/sec).
//            double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);
//
//            // Combine and clamp to motor power range [-1, 1]
//            double combinedOutput = ffOutput + pidOutput;
//            combinedOutput = Math.max(-1.0, Math.min(1.0, combinedOutput));
//
//            // If the driver pressed D-pad (we want launcher behavior), apply combined power.
//            // If the player pressed 'x' or dpad_down, those override below.
//            robot.launcher.setPower(combinedOutput);
//
//            int currentPos = robot.turret.getCurrentPosition();
//
//            double degrees = (currentPos / (TICKS_PER_REV / GEAR_RATIO)) * 360;
//
//            telemetry.addData("Turret Tracking", "");
//            telemetry.addData("Encoder Ticks", currentPos);
//            telemetry.addData("Degrees", "%.2f", degrees);
//            telemetry.addData("flywheelon", flywheelon);
//            telemetry.update();
//        }
//    }
//}