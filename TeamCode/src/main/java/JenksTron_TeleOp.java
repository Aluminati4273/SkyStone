import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="JenksTron: TeleOp", group="TeleOp")

public class JenksTron_TeleOp extends OpMode {

    HardwareJenksTron3000 robot = new HardwareJenksTron3000(); //use the class to define JenksTron's hardware


    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }


    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // threshold for dead space on joysticks
        double THRESHOLD = 0.05;


        //************************* DRIVING THE ROBOT *************************************

        if(gamepad1.dpad_right){
            robot.driveOrient = 1;
        }
        else if(gamepad1.dpad_down){
            robot.driveOrient = 2;
        }
        else if(gamepad1.dpad_left) {
            robot.driveOrient = 3;
        }
        else {
            robot.driveOrient = 0;
        }

        // left stick to move forward and backward
        if(Math.abs(gamepad1.left_stick_y) > THRESHOLD) {
            robot.driveForwardBackward(gamepad1.left_stick_y);

            // allows right and left motion to be combined
            if(Math.abs(gamepad1.left_stick_x)> THRESHOLD){
                robot.driveRightLeft(gamepad1.left_stick_x);
                robot.driveForwardBackward(gamepad1.left_stick_y);
            }
        }

        // left stick to move right and left
        else if(Math.abs(gamepad1.left_stick_x)> THRESHOLD){
            robot.driveRightLeft(gamepad1.left_stick_x);

            //allows forward and backward motion to be combined
            if(Math.abs(gamepad1.left_stick_y)> THRESHOLD){
                robot.driveForwardBackward(gamepad1.left_stick_y);
                robot.driveRightLeft(gamepad1.left_stick_x);
            }
        }

        //rotates the robot with joystick one right stick x direction
        else if (Math.abs(gamepad1.right_stick_x) > THRESHOLD){
            robot.rotate(gamepad1.right_stick_x);
        }

        // if no joys on player one all wheels stop
        else{
            robot.driveNotAtAll();
        }

        //********************** CONTROLLING ROBOT PERIPHERIALS *************************

        //control the foundation moving servos with player 1's right bumper to grab and the left bumper to release
        if (gamepad1.right_bumper){
            robot.foundation1.setPosition(robot.FOUNDATION1_GRAB);
            robot.foundation2.setPosition(robot.FOUNDATION2_GRAB);
        }
        if (gamepad1.left_bumper){
            robot.foundation1.setPosition(robot.FOUNDATION1_START);
            robot.foundation2.setPosition(robot.FOUNDATION2_START);
        }



    }

}