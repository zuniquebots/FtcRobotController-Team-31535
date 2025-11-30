package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.8)
            .forwardZeroPowerAcceleration(-31.15121088813849)
            .lateralZeroPowerAcceleration(-48.66403338955584)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.005,0,0.025,0.028))
            .headingPIDFCoefficients(new PIDFCoefficients( 0.001,0,0.009,0.028))  //0.030
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(1.25,0,0.0003,0.6,0.028))
            .centripetalScaling(0.5)
            .translationalPIDFSwitch(4)
            .drivePIDFSwitch(15);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFrontDrive")
            .rightRearMotorName("rightBackDrive")
            .leftRearMotorName("leftBackDrive")
            .leftFrontMotorName("leftFrontDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            //// --- SET MOTOR AND SERVO DIRECTIONS ---
            //// This configuration allows for proper strafing
            //leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            //leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Changed from FORWARD
            //rightFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Changed from REVERSE
            //rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            .xVelocity(64.01834514948327)
            .yVelocity(56.55151607483391);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2)
            .strafePodX(-4)
            //.distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("PinpointComputer")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    /* These are the PathConstraints in order:
    tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
    brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart

    The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
     */
    /*public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1.8,
             1);*/


    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            0.1,
            0.1,
            0.009,
            500,
            1.8,
            10,
            1
    );



    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
