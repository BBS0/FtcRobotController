/*
This is a modified version of StraightTest.java from the ACME Robotics Road-Runner-Quickstart project.
Included with permission from ACME Robotics.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot;
import org.firstinspires.ftc.teamcode.robot.OpenCvDetector;

import static android.icu.lang.UCharacter.DecompositionType.SQUARE;

//@Config
@Autonomous(name = "BlueLeftPark", group = "drive")
public class BlueLeftPark extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecBot drive = new BotBuildersMecBot(hardwareMap);
        drive.opMode = this;
        drive.AutoLockGoal();

        //sleep(25000);

        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(80, 10), 0)
                .build();

        Trajectory strafe = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .strafeLeft(15)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        sleep(500);
        drive.turn(Math.toRadians(105));
        sleep(100);
        drive.AutoArmPickUp();
        sleep(100);
        drive.AutoArmDropOff();
        drive.followTrajectory(strafe);
        sleep(500);
        drive.turn(Math.toRadians(-105));
    }
}
