package org.firstinspires.ftc.teamcode.Projects;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Projects.HWMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "flywheel")
public class turretrunflywheeltest extends LinearOpMode {

    public turrettesthwmap robot = new turrettesthwmap();
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);


        waitForStart();

        while (opModeIsActive()){
            double speed = .7;
            if (gamepad1.dpad_down) {
                speed += .1;
            }
            if (gamepad1.dpad_up) {
                speed -= .1;
            }

            if (gamepad1.a) {
                robot.flywheel.setPower(speed);
            }



        }
    }
}
