/*
This is a modified version of SampleMecanumDrive.java from the ACME Robotics Road-Runner-Quickstart project.
Included with permission from ACME Robotics.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.robot;

//import android.support.annotation.NonNull;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.jetbrains.annotations.NotNull;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.*;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.robot.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.robot.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.robot.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.robot.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.robot.DriveConstants.getMotorVelocityF;
import static  org.firstinspires.ftc.teamcode.robot.DriveConstants.kA;
import static  org.firstinspires.ftc.teamcode.robot.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.robot.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware, for use with Acme Robotics RoadRunner.
 * This class uses three dead-wheel encoders for localization.
 *
 */
@Config
public class  BotBuildersMecBot extends MecanumDrive
{

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static double LATERAL_MULTIPLIER = 1;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot.Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    public DriveConstraints constraints;
    private TrajectoryFollower follower;

    private List<Pose2d> poseHistory;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    private Pose2d lastPoseOnTurn;

    private VoltageSensor batteryVoltageSensor;

    // Robot mechanisms

    public LinearOpMode opMode;

    private Servo arm_base;
    private Servo arm_mid;
    private Servo arm_grip;

    private CRServo upWheel_intake;
    private DcMotor intake;
    private DcMotorEx armFlip;
    private DcMotorEx shooter;
    private Servo shot_loader;
    private Servo hopper;

    public BotBuildersMecBot(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot.Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new ArrayList<>();

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "front_left_motor"); //port 0
        leftRear = hardwareMap.get(DcMotorEx.class, "back_left_motor"); //port 1
        rightRear = hardwareMap.get(DcMotorEx.class, "back_right_motor"); //port 2
        rightFront = hardwareMap.get(DcMotorEx.class, "front_right_motor"); //port 3

        armFlip = hardwareMap.get(DcMotorEx.class, "right_enc"); //port 2

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
           // this.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
           this.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // reverse any motors using DcMotor.setDirection()
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));



        //Setup the robot specific items

        arm_grip = hardwareMap.get(Servo.class, "arm_grip"); //port 3

        arm_mid = hardwareMap.get(Servo.class, "arm_mid");//port 2

        arm_base = hardwareMap.get(Servo.class, "arm_base");  //port 1

        upWheel_intake = hardwareMap.get(CRServo.class, "crservo");

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shot_loader = hardwareMap.get(Servo.class, "shot_loader"); //port 4

        hopper = hardwareMap.get(Servo.class, "hopper"); //port

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void ArmUp(boolean fast){
        armFlip.setDirection(DcMotorSimple.Direction.REVERSE);
        armFlip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armFlip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(fast) {
            armFlip.setPower(0.5);
        }else{
            armFlip.setPower(0.1);
        }
    }

    public void ArmDown(boolean fast){
        armFlip.setDirection(DcMotorSimple.Direction.FORWARD);
        armFlip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armFlip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(fast) {
            armFlip.setPower(0.3);
        }else{
            armFlip.setPower(0.1);
        }
    }

    public void TurnIntakeOn()
    {
       upWheel_intake.setPower(1);
        intake.setPower(1);

    }

    public void TurnOuttakeOn()
    {

        intake.setPower(-1);

    }
    public void TurnIntakeOff(){
        upWheel_intake.setPower(0);
        intake.setPower(0);

    }

    public void AutoLockGoal()
    {
        arm_grip.setPosition(0);
    }

    public void LockArm(){
        arm_grip.setPosition(0);
    }

    public void UnlockArm(){
        arm_grip.setPosition(1);
    }

    public void StopArm(){
        armFlip.setPower(0);
    }

    public void AutoArmPickUp()
    {
        //move the arm
        armFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int start = armFlip.getCurrentPosition();
        armFlip.setTargetPosition(500 - start );
        armFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armFlip.setPower(0.1);
        while(armFlip.isBusy() && this.opMode.opModeIsActive()  ){
            this.opMode.telemetry.addData("Pos", armFlip.getCurrentPosition());
            this.opMode.telemetry.addData("Target", armFlip.getTargetPosition());
            this.opMode.telemetry.update();
        }
    }


    public void AutoArmTop()
    {
        armFlip.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
        //move the arm
        armFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int start = armFlip.getCurrentPosition();
        armFlip.setTargetPosition(start - 1000 ); //800
        armFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armFlip.setPower(0.3);
        while(armFlip.isBusy() && this.opMode.opModeIsActive() ){
            this.opMode.telemetry.addData("Pos", armFlip.getCurrentPosition());
            this.opMode.telemetry.addData("Target", armFlip.getTargetPosition());
            this.opMode.telemetry.update();
        }
        armFlip.setPower(0);
    }

    public void AutoArmDropOff()
    {
        armFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int start = armFlip.getCurrentPosition();
        armFlip.setTargetPosition(20 - start );
        armFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armFlip.setPower(0.1);
        while(armFlip.isBusy() && this.opMode.opModeIsActive() && !armFlip.isOverCurrent() ){
            this.opMode.telemetry.addData("Pos", armFlip.getCurrentPosition());
            this.opMode.telemetry.addData("Target", armFlip.getTargetPosition());
            this.opMode.telemetry.update();
        }
        armFlip.setPower(0);
        armFlip.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT);
        UnlockArm();
    }

    public void HopperForIntake(){
        hopper.setPosition(1);
    }

    public void HopperForShooting(){
        hopper.setPosition(0);
    }

    public void TurnJHopShooterOn(double power){
        shooter.setVelocity(power, AngleUnit.DEGREES);
    } // treat 240 as the max power. and 222 slow

    public void TurnJHopShooterOff(){



        shooter.setPower(0);
    }

    public void ShootServo(){
        shot_loader.setPosition(0);

    }

    public void ReArmShootServo(){
        shot_loader.setPosition(1);
    }

    public void FullAutoShoot(){
        AutoShootLoad();
        InternalSleep(1500);
        ShootServo();
        InternalSleep(1000);
        ReArmShootServo();
        AutoShootOff();
    }

    private void InternalSleep(int milliseconds){

        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void AutoShootLoad(){
        //TurnJHopShooterOn();
        HopperForShooting();

    }

    public void AutoShootOff(){
       // TurnJHopShooterOff();
        HopperForIntake();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, constraints);
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );

        turnStart = clock.seconds();
        mode = org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot.Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot.Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);



        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawRobot(fieldOverlay, newPose);

                if (t >= turnProfile.duration()) {
                    mode = org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot.Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

                if (!follower.isFollowing()) {
                    mode = org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot.Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot.Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }



    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }


    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v); //good
        leftRear.setPower(v1); //good
        rightRear.setPower(v2); //good
        rightFront.setPower(v3); //good
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }


}