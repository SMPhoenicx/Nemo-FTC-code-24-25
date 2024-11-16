import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class forwardParkDriftLeft extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    @Override
    public void runOpMode(){
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");

        //initialize motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        double speed = 0.5;
        double phase2Offset = 0;
        while(opModeIsActive()) {
            //strafe left
//            leftFrontDrive.setPower(-speed);
//            rightBackDrive.setPower(-speed);
//            leftBackDrive.setPower(speed);
//            rightFrontDrive.setPower(speed);
            leftFrontDrive.setPower(speed + phase2Offset);
            rightBackDrive.setPower(speed - phase2Offset);
            leftBackDrive.setPower(speed + phase2Offset);
            rightFrontDrive.setPower(speed - phase2Offset);
            telemetry.addData("Elapsed Time", runtime.seconds());
            telemetry.update();

            // Break after 3 seconds as an example
            if (runtime.seconds() > 1) {
                phase2Offset = -0.2;
            }
            if (runtime.seconds() > 2) {
                break;
            }
        }
    }
}
