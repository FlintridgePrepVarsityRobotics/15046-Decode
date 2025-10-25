package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Projects.HWMap;

/**
 * Created by tom on 9/26/17.
 * This assumes that you are using a REV Robotics Control Hub or REV Robotics Expansion Hub
 * as your DC motor controller.  This op mode uses the extended/enhanced
 * PID-related functions of the DcMotorEx class.
 */

@Autonomous(name="Concept: Change PID", group = "Concept")
public class PIDTUning extends LinearOpMode {

    // our DC motor.

    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;
    public HWMap robot = new HWMap ();

    public void runOpMode() {
        // get reference to DC motor.

        // since we are using the Control Hub or Expansion Hub,
        // cast this motor to a DcMotorEx object.
        robot.init(hardwareMap);

        // wait for start command.
        waitForStart();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDCoefficients pidOrig = robot.launcher.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        robot.launcher.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDCoefficients pidModified = robot.launcher.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // display info to user.
        while(opModeIsActive()) {
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                    pidOrig.p, pidOrig.i, pidOrig.d);
            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                    pidModified.p, pidModified.i, pidModified.d);
            telemetry.update();
        }
    }
}