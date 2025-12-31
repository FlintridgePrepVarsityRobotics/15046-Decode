package org.firstinspires.ftc.teamcode.Projects;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "turretTrackerOnly")
public class turretTrackerOnly extends LinearOpMode {
    public turretHWMap robot = new turretHWMap();
    final double TICKS_PER_REV = 294;
    final double GEAR_RATIO = .3953; // 34/86

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized, turret pos reset to 0.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            int currentPos = robot.turret.getCurrentPosition();

            double degrees = (currentPos / (TICKS_PER_REV / GEAR_RATIO)) * 360;

            telemetry.addData("Turret Tracking", "");
            telemetry.addData("Encoder Ticks", currentPos);
            telemetry.addData("Degrees", "%.2f", degrees);
            telemetry.update();
        }
    }
}