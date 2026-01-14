package org.firstinspires.ftc.teamcode.Projects;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class newHWmap {

    public DcMotor fLeftWheel = null;  //expansion hub port0
    public DcMotor fRightWheel = null; //control hub port0
    public DcMotor bLeftWheel = null; //expansion hub port1
    public DcMotor bRightWheel = null; //control hub port1
    public ColorSensor sensor1 = null;
    public ColorSensor sensor2 = null;

    public DcMotorEx intake = null;
    public DcMotorEx turret = null;
    public DcMotorEx flywheel = null;
    public void init(HardwareMap hwMap) {
        fLeftWheel = hwMap.dcMotor.get("FL");
        fRightWheel = hwMap.dcMotor.get("FR");
        bLeftWheel = hwMap.dcMotor.get("BL");
        bRightWheel = hwMap.dcMotor.get("BR");

        sensor1 = hwMap.colorSensor.get("sensor1");
        sensor2 = hwMap.colorSensor.get("sensor2");

        intake = hwMap.get(DcMotorEx.class, "intake");
        flywheel = hwMap.get(DcMotorEx.class, "flywheel");

        turret = hwMap.get(DcMotorEx.class, "turret");

        fRightWheel.setDirection(DcMotor.Direction.REVERSE);
        fLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        bRightWheel.setDirection(DcMotor.Direction.REVERSE);
        bLeftWheel.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.FORWARD);

        turret.setDirection(DcMotor.Direction.REVERSE);

        fRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





        Stop();
    }


    public void Stop() {
        turret.setPower(0);
        fRightWheel.setPower(0);
        fLeftWheel.setPower(0);
        bRightWheel.setPower(0);
        bLeftWheel.setPower(0);
        intake.setPower(0);
        flywheel.setPower(0);

    }
}
