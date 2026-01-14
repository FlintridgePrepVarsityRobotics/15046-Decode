package org.firstinspires.ftc.teamcode.Projects;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class motortestHWmap {
    public DcMotor spin = null;

    public void init(HardwareMap hwMap) {
        spin = hwMap.dcMotor.get("spin");
        spin.setDirection(DcMotor.Direction.FORWARD);
        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Stop();


    }

    public void Stop() {
        spin.setPower(0);
    }
}
