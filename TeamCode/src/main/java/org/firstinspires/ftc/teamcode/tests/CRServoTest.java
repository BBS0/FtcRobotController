package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PracticeMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




@TeleOp(name="CR Servo", group="Test")
//@Disabled
public class CRServoTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {

        //BotBuildersMecBot bot = new BotBuildersMecBot(hardwareMap);

        DcMotor armFlip = hardwareMap.get(DcMotor.class, "right_enc"); //port 2

        Servo grip_servo = hardwareMap.get(Servo.class, "arm_grip"); //port 2

        Servo base_servo = hardwareMap.get(Servo.class, "arm_base"); //port 2

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


       // armFlip.setDirection(DcMotorSimple.Direction.REVERSE);


       // armFlip.setVelocity(50, AngleUnit.DEGREES);
       // armFlip.setPower(0.1);
        // run until the end of the match (driver presses STOP)

        armFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int start = armFlip.getCurrentPosition();
        armFlip.setTargetPosition(500 - start );



        while (opModeIsActive()) {

            grip_servo.setPosition(0);
            sleep(1500);

            armFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armFlip.setPower(0.2);

            while(armFlip.isBusy() && opModeIsActive()){
                telemetry.addData("Pos", armFlip.getCurrentPosition());
                telemetry.addData("Target", armFlip.getTargetPosition());
                telemetry.update();
            }

            armFlip.setPower(0);
            sleep(5000);
            grip_servo.setPosition(0);



            /*telemetry.addData("Status", "Base");
            telemetry.update();
            arm_base.setPosition(0);
            sleep(2000);
            arm_base.setPosition(1);
            sleep(2000);
            telemetry.addData("Status", "Mid");
            telemetry.update();
            arm_mid.setPosition(0);
            sleep(2000);
            arm_mid.setPosition(1);*/


                telemetry.update();

        }
    }
}

