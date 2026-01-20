package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Gamepad;



import org.firstinspires.ftc.teamcode.Projects.newHWmap;

@Autonomous(name = "ParkAuto")

public class autoILT extends LinearOpMode {
enum Parking {
rBlue,
lBlue,//a
rRed,
lRed,
    }

Gamepad currentGamepad1 = new Gamepad();
Gamepad previousGamepad1 = new Gamepad();
public newHWmap robot = new newHWmap();

@Override
public void runOpMode() throws InterruptedException {
//initialize hardware map

robot.init(hardwareMap);



// hi
// Autonomous code starts here

waitForStart(); //wait for play button to be pressed
tile(1600);

}

    public void tile(long time){
robot.bLeftWheel.setPower(0.2);
robot.fLeftWheel.setPower(0.2);
robot.bRightWheel.setPower(0.2);
robot.fRightWheel.setPower(0.2);
sleep(time);
robot.bLeftWheel.setPower(0);
robot.fLeftWheel.setPower(0);
robot.bRightWheel.setPower(0);
robot.fRightWheel.setPower(0);
}



}
