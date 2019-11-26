import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@Autonomous(name="RedJustPark", group="Auto")


public class RedJustPark extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareJenksTron3000 robot           = new HardwareJenksTron3000();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        waitForStart();
        runtime.reset();

        robot.init(hardwareMap);

        //wait for 10 seconds for other team movement
        sleep(10000);

        //move away from the wall
        robot.driveRightLeft(0.25);
        sleep(125);
        robot.driveNotAtAll();
        sleep(150);

        //drive over the line and park
        robot.driveForwardBackward(0.25);
        sleep(800);
        robot.driveNotAtAll();



    }
}


