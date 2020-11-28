package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PracticeMecanumDrive;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




@TeleOp(name="Motor Test", group="Test")
@Disabled
public class MotorTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        DcMotor left = hardwareMap.get(DcMotor.class,"back_left_motor");

        DcMotor right = hardwareMap.get(DcMotor.class,"front_left_motor");

        DcMotor centre = hardwareMap.get(DcMotor.class,"back_right_motor");

        DcMotor other = hardwareMap.get(DcMotor.class,"front_right_motor");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            left.setPower(0.5);
            sleep(1000);
            left.setPower(0);
            sleep(1000);

            right.setPower(0.5);
            sleep(1000);
            right.setPower(0);
            sleep(1000);

            centre.setPower(0.5);
            sleep(1000);
            centre.setPower(0);
            sleep(1000);

            other.setPower(0.5);
            sleep(1000);
            other.setPower(0);
            sleep(1000);

            telemetry.addData("Left", left.getCurrentPosition());
            telemetry.addData("Right", right.getCurrentPosition());
            telemetry.addData("centre", centre.getCurrentPosition());


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

