import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
public class Navigation extends LinearOpMode {
        DcMotor leftEncoder;
        DcMotor perpEncoder;
        BNO055IMU imu;

        @Override
        public void runOpMode() throws InterruptedException {
            // Corrected the syntax and types here
            leftEncoder = hardwareMap.get(DcMotor.class, "leftEncoder");
            perpEncoder = hardwareMap.get(DcMotor.class, "rightEncoder");
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            boolean initialize = imu.initialize(parameters);

            while (opModeIsActive()) {
                double heading = imu.getAngularOrientation().firstAngle;

                telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
                telemetry.addData("Perp Encoder", perpEncoder.getCurrentPosition());
                telemetry.addData("Heading (rad)", heading);
                telemetry.update();
            }


        }
}



