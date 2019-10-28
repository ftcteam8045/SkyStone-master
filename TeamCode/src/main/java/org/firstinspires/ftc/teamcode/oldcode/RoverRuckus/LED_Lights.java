
package org.firstinspires.ftc.teamcode.oldcode.RoverRuckus;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static org.firstinspires.ftc.teamcode.oldcode.DriveTrain.drive_COEF;
import static org.firstinspires.ftc.teamcode.oldcode.DriveTrain.drive_THRESHOLD;

@Autonomous(name = "LED Lights", group = "Cosmo")
@Disabled
public class LED_Lights extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware8045testbot Cosmo = new Hardware8045testbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime LEDcycletime = new ElapsedTime();

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AWfr4/T/////AAAAGRMg80Ehu059mDMJI2h/y+4aBmz86AidOcs89UScq+n+QQyGFT4cZP+rzg1M9B/CW5bgDoVf16x6x3WlD5wYKZddt0UWQS65VIFPjZlM9ADBWvWJss9L1dj4X2LZydWltdeaBhkXTXFnKBkKLDcdTyC2ozJlcAUP0VnLMeI1n+f5jGx25+NdFTs0GPJYVrPQRjODb6hYdoHsffiOCsOKgDnzFsalKuff1u4Z8oihSY9pvv3me2gJjzrQKqp2gCRIZAXDdYzln28Z/8vNSU+aXr6eoRrNXPpYdAwyYI+fX2V9H04806eSUKsNYcPBSbVlhe2KoUsSD7qbOsBMagcEIdMZxo010kVCHHhnhV3IFIs8";

    /** {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.     */
    private VuforiaLocalizer vuforia;

     @Override
    public void runOpMode() {

         final double FORWARD_SPEED = 0.3;
         final double TURN_SPEED = 0.3;
         final int blinktime = 300;  // milliseconds for the lights to be on/off

         int goldPosition = 0;   // 0 is on left, 1 in center, 2 on right

         final boolean teamIsRed = false;
         /*
          * Initialize the drive system variables.the Robot
          * The init() method of the hardware class does all the work here
          */
         Cosmo.init(hardwareMap);

         //Create variable thing for light colors  based on team color
         RevBlinkinLedDriver.BlinkinPattern teamColor;
         if(teamIsRed){
             teamColor = RevBlinkinLedDriver.BlinkinPattern.RED;
         }
         else{
             teamColor = RevBlinkinLedDriver.BlinkinPattern.BLUE;
         }

         /** TURN ON LIGHTS */
         Cosmo.LEDDriver.setPattern(teamColor);

         LEDcycletime.reset();
         while (!opModeIsActive()  && !isStopRequested() ) {
         /**   LED Light signalling  **/
         if (goldPosition == 2) {
             if        ((LEDcycletime.milliseconds() <     blinktime)) {                 // blink pattern white white gold
                 Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
             } else if ((LEDcycletime.milliseconds() < 2 * blinktime)) {
                 Cosmo.LEDDriver.setPattern(teamColor);
             } else if ((LEDcycletime.milliseconds() < 3 * blinktime)) {
                 Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
             } else if ((LEDcycletime.milliseconds() < 4 * blinktime)) {
                 Cosmo.LEDDriver.setPattern(teamColor);
             } else if ((LEDcycletime.milliseconds() < 5 * blinktime)) {
                 Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
             } else if ((LEDcycletime.milliseconds() < 12 * blinktime)) {                                       // back to team color
                 Cosmo.LEDDriver.setPattern(teamColor);
             } else  {                                      // reset timer, repeat cycle
                 LEDcycletime.reset();
             }

         } else if (goldPosition == 1) {     // insert blink pattern for white gold white  here
             if        ((LEDcycletime.milliseconds() <     blinktime)) {                 // blink pattern white white gold
                 Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
             } else if ((LEDcycletime.milliseconds() < 2 * blinktime)) {
                 Cosmo.LEDDriver.setPattern(teamColor);
             } else if ((LEDcycletime.milliseconds() < 3 * blinktime)) {
                 Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
             } else if ((LEDcycletime.milliseconds() < 4 * blinktime)) {
                 Cosmo.LEDDriver.setPattern(teamColor);
             } else if ((LEDcycletime.milliseconds() < 5 * blinktime)) {
                 Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
             } else if ((LEDcycletime.milliseconds() < 12 * blinktime)) {                                       // back to team color
                 Cosmo.LEDDriver.setPattern(teamColor);
             } else  {                                      // reset timer, repeat cycle
                 LEDcycletime.reset();
             }
         } else if (goldPosition == 0) {     // insert blink pattern for gold white white  here
             if        ((LEDcycletime.milliseconds() <     blinktime)) {                 // blink pattern white white gold
                 Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
             } else if ((LEDcycletime.milliseconds() < 2 * blinktime)) {
                 Cosmo.LEDDriver.setPattern(teamColor);
             } else if ((LEDcycletime.milliseconds() < 3 * blinktime)) {
                 Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
             } else if ((LEDcycletime.milliseconds() < 4 * blinktime)) {
                 Cosmo.LEDDriver.setPattern(teamColor);
             } else if ((LEDcycletime.milliseconds() < 5 * blinktime)) {
                 Cosmo.LEDDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
             } else if ((LEDcycletime.milliseconds() < 12 * blinktime)) {                                       // back to team color
                 Cosmo.LEDDriver.setPattern(teamColor);
             } else  {                                      // reset timer, repeat cycle
                 LEDcycletime.reset();
             }
         }
         /**   End of  LED Light signalling  **/
     }
        // Wait for the game to start (driver presses PLAY) replaced by init loop
        waitForStart();
                    runtime.reset();
                    LEDcycletime.reset();
        while (opModeIsActive() && !isStopRequested() ) {
            telemetry.addData("Path", " %2.5f S Elapsed", runtime.seconds());
            telemetry.update();



        }

// goldposition 0 = left,1 = center, 2 = right


        if (goldPosition == 0) {        // left position
//            mecanumTurn(0.3, 45);
            mecanumDrive(0.5, 12, 0, 0);     // drive forward
            mecanumDrive(0.5, 15, 0, -90);    // drive left
            mecanumDrive(0.5, 8, 0, 0);     // drive forward

            mecanumDrive(0.5, -8, 0, 0);     // drive backwards

            //mecanumDrive(0.5, 15, 0, 90);    // drive right backwards
            //mecanumTurn(0.3, -45);



        }

        if (goldPosition == 1) {

            mecanumDrive(0.5, 12, 0, 0);     // drive forward
            mecanumDrive(0.5, 12, 0, 0);     // drive forward

            mecanumDrive(0.5, -8, 0, 0);     // drive backwards
            mecanumDrive(0.5,30,0,-90);      // drive left

        }

        if (goldPosition == 2) {

            mecanumDrive(0.5, 12, 0, 0);     // drive forward
            mecanumDrive(0.5, 15, 0, 90);    // drive right
            mecanumDrive(0.5, 12, 0, 0);     // drive forward

            mecanumDrive(0.5, -8, 0, 0);     // drive backwards
            mecanumDrive(0.5,30,0,-90);      // drive left
            //mecanumDrive(0.5, 15, 0, 90);    // drive right backwards
        }

        // mecanumDrive(0.5,30,0,-90);      // drive towards wall

        // Step 4:  Stop and close the claw.
        Cosmo.leftFront.setPower(0);
        Cosmo.rightFront.setPower(0);
        Cosmo.leftRear.setPower(0);
        Cosmo.rightRear.setPower(0);

//        robot.leftClaw.setPosition(1.0);
//        robot.rightClaw.setPosition(0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    //  Drive routine using the IMU and Mecanum wheels
    //  Robot Orientation is to the field
    //  Drive direction is from the robot

    public void mecanumDrive(double speed, double distance, double robot_orientation, double drive_direction) {
        double max;
        double multiplier;
        int right_start;
        int left_start;
        int moveCounts;
        //int drive_direction = -90;
        moveCounts = (int) (distance * Cosmo.COUNTS_PER_INCH);
        right_start = Cosmo.rightRear.getCurrentPosition();
        left_start = Cosmo.leftRear.getCurrentPosition();
        double lfpower;
        double lrpower;
        double rfpower;
        double rrpower;

        double lfbase;
        double lrbase;
        double rfbase;
        double rrbase;
        lfbase = signum(distance) * Math.cos(Math.toRadians(drive_direction + 45));
        lrbase = signum(distance) * Math.sin(Math.toRadians(drive_direction + 45));
        rfbase = signum(distance) * Math.sin(Math.toRadians(drive_direction + 45));
        rrbase = signum(distance) * Math.cos(Math.toRadians(drive_direction + 45));
        while (((abs(Cosmo.rightRear.getCurrentPosition() - right_start) + abs(Cosmo.leftRear.getCurrentPosition() - left_start)) / 2 < abs(moveCounts)) && opModeIsActive() /* ENCODERS*/) {//Should we average all four motors?
            //Determine correction
            double correction = robot_orientation - getheading();
            if (correction <= -180) {
                correction += 360;
            } else if (correction >= 180) {                      // correction should be +/- 180 (to the left negative, right positive)
                correction -= 360;
            }
            lrpower = lrbase; //MIGHT BE MORE EFFECIENT TO COMBINE THESE WITHT HE ADJUSTMENT PART AND SET ADJUSTMENT TO ZERO IF NOT NEEDED
            lfpower = lfbase;
            rrpower = rrbase;
            rfpower = rfbase;
            if (abs(correction) > drive_THRESHOLD) {//If you are off
                //Apply power to one side of the robot to turn the robot back to the right heading
                double right_adjustment = Range.clip((drive_COEF * correction / 45), -1, 1);
                lrpower -= right_adjustment;
                lfpower -= right_adjustment;
                rrpower = rrbase + right_adjustment;
                rfpower = rfbase + right_adjustment;

            }//Otherwise you Are at the right orientation

            //Determine largest power being applied in either direction
            max = abs(lfpower);
            if (abs(lrpower) > max) max = abs(lrpower);
            if (abs(rfpower) > max) max = abs(rfpower);
            if (abs(rrpower) > max) max = abs(rrpower);

            multiplier = speed / max; //multiplier to adjust speeds of each wheel so you can have a max power of 1 on atleast 1 wheel

            lfpower *= multiplier;
            lrpower *= multiplier;
            rfpower *= multiplier;
            rrpower *= multiplier;

            Cosmo.leftFront.setPower(lfpower);
            Cosmo.leftRear.setPower(lrpower);
            Cosmo.rightFront.setPower(rfpower);
            Cosmo.rightRear.setPower(rrpower);

//            RobotLog.ii("[GromitIR] ", Double.toString(18.7754*Math.pow(sharpIRSensor.getVoltage(),-1.51)), Integer.toString(left_front.getCurrentPosition()));

        }
        //gromit.driveTrain.stopMotors();
        Cosmo.leftFront.setPower(0.0);
        Cosmo.rightFront.setPower(0.0);
        Cosmo.rightRear.setPower(0.0);
        Cosmo.leftRear.setPower(0.0);
    }


    // Turn using the IMU and meccanum drive
    public void mecanumTurn(double speed, double target_heading) {
        if (speed > 1) speed = 1.0;
        //else if(speed <= 0) speed = 0.1;

        double correction = target_heading - getheading();
        if (correction <= -180) {
            correction += 360;   // correction should be +/- 180 (to the left negative, right positive)
        } else if (correction >= 180) {
            correction -= 360;
        }

        while (abs(correction) >= Cosmo.turn_THRESHOLD && opModeIsActive()) { //opmode active?{
            correction = target_heading - getheading();
            if (abs(correction) <= Cosmo.turn_THRESHOLD) break;

            if (correction <= -180)
                correction += 360;   // correction should be +/- 180 (to the left negative, right positive)
            if (correction >= 180) correction -= 360;
            /**^^^^^^^^^^^MAYBE WE ONLY NEED TO DO THIS ONCE?????*/

            double adjustment = Range.clip((Math.signum(correction) * Cosmo.turn_MIN_SPEED + Cosmo.turn_COEF * correction / 100), -1, 1);  // adjustment is motor power: sign of correction *0.07 (base power)  + a proportional bit

            Cosmo.leftFront.setPower(-adjustment * speed);
            Cosmo.leftRear.setPower(-adjustment * speed);
            Cosmo.rightFront.setPower((adjustment * speed));
            Cosmo.rightRear.setPower((adjustment * speed));
        }
//        gromit.driveTrain.stopMotors();
        Cosmo.leftFront.setPower(0.0);
        Cosmo.rightFront.setPower(0.0);
        Cosmo.rightRear.setPower(0.0);
        Cosmo.leftRear.setPower(0.0);
    }


    public float getheading() {
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
//        angles = Cosmo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle; //For a -180 to 180 range
        //return (angles.firstAngle + 180 + 180)%360; // for a zero to 360 range
    }
}






