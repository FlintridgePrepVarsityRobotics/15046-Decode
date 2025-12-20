package org.firstinspires.ftc.teamcode.Projects;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class turrettesthwmap {
    public DcMotor flywheel = null;
    public void init(HardwareMap hwMap) {
        flywheel = hwMap.dcMotor.get("fwheel");
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Stop();
    }
    public void Stop() {
        flywheel.setPower(0);
    }
}
