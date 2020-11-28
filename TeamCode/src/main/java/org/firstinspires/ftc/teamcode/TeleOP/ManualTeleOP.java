package org.firstinspires.ftc.teamcode.TeleOP;
/*
This is a modified version of LocalizationTest.java from the ACME Robotics Road-Runner-Quickstart project.
Included with permission from ACME Robotics.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot;

//@Config
@TeleOp(name = "ManualTeleOP", group = "drive")
public class ManualTeleOP extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;



    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //PracticeMecBot drive = new PracticeMecBot(hardwareMap);
        BotBuildersMecBot drive = new BotBuildersMecBot(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        ElapsedTime telemetryTimer = new ElapsedTime();
        while (opModeIsActive()) {
            // only user 1 should be able to move the robot
            Pose2d baseVel = new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );

            //when press slow down

            if(gp1.gamepad.left_trigger > 0.6 || gp2.gamepad.left_trigger > 0.6)
            {
                baseVel = new Pose2d(
                        -gamepad1.left_stick_y * 0.5,
                        -gamepad1.left_stick_x * 0.5,
                        -gamepad1.right_stick_x * 0.5
                );

            }

            Pose2d vel;
            if (Math.abs(baseVel.getX()) + Math.abs(baseVel.getY()) + Math.abs(baseVel.getHeading()) > 1) {
                // re-normalize the powers according to the weights
                double denom = VX_WEIGHT * Math.abs(baseVel.getX())
                        + VY_WEIGHT * Math.abs(baseVel.getY())
                        + OMEGA_WEIGHT * Math.abs(baseVel.getHeading());
                vel = new Pose2d(
                        VX_WEIGHT * baseVel.getX(),
                        VY_WEIGHT * baseVel.getY(),
                        OMEGA_WEIGHT * baseVel.getHeading()
                ).div(denom);
            } else {
                vel = baseVel;
            }

            drive.setDrivePower(vel);
            drive.update();

            if(gp1.gamepad.a || gp2.gamepad.a){
                drive.TurnIntakeOn();
            }else{
                drive.TurnIntakeOff();
            }

            if(gp1.gamepad.x || gp2.gamepad.x){
                drive.TurnOuttakeOn();
            }else{
               // drive.TurnIntakeOff(); //this can't happen - it turns the inake off - needs to be in the logic above.
            }

            if(gp1.getButton(GamepadKeys.Button.B) || gp2.getButton(GamepadKeys.Button.B)){
                if(gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.6 || gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.6){
                    drive.TurnJHopShooterOn(210); // slow
                } else {
                    drive.TurnJHopShooterOn(230); //0.75 normal
                }
            }else{
                drive.TurnJHopShooterOff();
            }

            if(gp1.getButton(GamepadKeys.Button.DPAD_LEFT) || gp2.getButton(GamepadKeys.Button.DPAD_LEFT)){
              drive.UnlockArm();
            }
            if(gp1.getButton(GamepadKeys.Button.DPAD_RIGHT) || gp2.getButton(GamepadKeys.Button.DPAD_RIGHT)){
              drive.LockArm();
            }
            boolean fastArm = true;
            if(gp1.gamepad.left_trigger > 0.6 || gp2.gamepad.left_trigger > 0.6) {
                fastArm = false;
            }

            if(gp1.getButton(GamepadKeys.Button.DPAD_UP) || gp2.getButton(GamepadKeys.Button.DPAD_UP)){
                drive.ArmUp(fastArm);
            }
            else if(gp1.getButton(GamepadKeys.Button.DPAD_DOWN) || gp2.getButton(GamepadKeys.Button.DPAD_DOWN)){
                drive.ArmDown(fastArm);
            }
            else{
                drive.StopArm();
            }

            if(gp1.getButton(GamepadKeys.Button.RIGHT_BUMPER) || gp2.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
                telemetry.addData("Shoot", "Now");
                drive.ShootServo();
            }else{
                telemetry.addData("Reset", "Now");
                drive.ReArmShootServo();
            }

            if(gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6 || gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6){
                drive.HopperForShooting();
            }else{
                drive.HopperForIntake();
            }


            telemetry.update();

            if (telemetryTimer.milliseconds() > 200) {
                telemetryTimer.reset();
                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
                telemetry.update();
            }
        }
    }
}
