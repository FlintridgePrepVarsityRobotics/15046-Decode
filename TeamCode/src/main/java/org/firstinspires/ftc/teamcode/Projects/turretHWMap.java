package org.firstinspires.ftc.teamcode.Projects;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;



public class turretHWMap {
    public DcMotorEx turret = null;
    public DcMotorEx flywheel = null;
    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");
//        flywheel = hwMap.get(DcMotorEx.class, "launcher");


        turret.setDirection(DcMotor.Direction.REVERSE);
//        flywheel.setDirection(DcMotor.Direction.FORWARD);


        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Stop();
    }


    public void Stop() {
        turret.setPower(0);
//        flywheel.setPower(0);
    }
}
