package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(9.8)
            .forwardZeroPowerAcceleration(-72.439080727)
            .lateralZeroPowerAcceleration(-67.042457146)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.008, 0.045))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01,0.06))
            ;

//

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .threeWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();

    }
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//60.0469213
            .xVelocity(77.8308890368)
            .yVelocity(60.00195376767557)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotor.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.003098687534)//.002926 5836395901167
            .strafeTicksToInches(.00299073299)
            .turnTicksToInches(.002859893175775)
            .leftPodY(4.5)
            .rightPodY(-4.5)
            .strafePodX(-7.5)
            .leftEncoder_HardwareMapName("FL")
            .rightEncoder_HardwareMapName("FR")
            .strafeEncoder_HardwareMapName("BR")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE);
}
//Everson was here