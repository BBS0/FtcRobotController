package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PracticeMecanumDrive;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Practice Drive", group="Drive")
//Disabled
public class PracticeDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private PracticeMecanumDrive driveTrain = null;
    private GamepadEx controller1 = null;

    final double STICK_DEAD_ZONE = 0.1;
    double strafe = 0;
    double forward = 0;
    double turn = 0;

    @Override
    public void runOpMode() {

        driveTrain = new PracticeMecanumDrive(hardwareMap);
        controller1 = new GamepadEx(gamepad1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (Math.abs(controller1.getLeftY()) > STICK_DEAD_ZONE) {
                forward = controller1.getLeftY();
            } else {
                forward = 0;
            }

            if (Math.abs(controller1.getLeftX()) > STICK_DEAD_ZONE) {
                strafe = controller1.getLeftX();
            } else {
                strafe = 0;
            }

            if (Math.abs(controller1.getRightX()) > STICK_DEAD_ZONE) {
                //turn = controller1.getRightX();
                if (forward == 0 && strafe == 0) {
                    forward = 1;
                }
                turn = controller1.getRightX();
            } else {
                turn = 0;
            }
            //double strafe, double forward, double turn
            driveTrain.driveRobot(strafe, forward, turn);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("strafe", strafe);
            telemetry.addData("forward", forward);
            telemetry.addData("turn", turn);
            telemetry.addData("version", "1");
            telemetry.update();
        }
    }
}

