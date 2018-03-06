package org.firstinspires.ftc.teamcode;

/**
 * Created by mg15 on 2/19/18.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

//package org.firstinspires.ftc.teamcode;

//@TeleOp(name = "Chassis", group = "Chassis")

public class Chassis extends OpMode {
    // basic modes of operation for the chassis

//@Disabled                            // Uncomment this to add to the opmode list


    private static final String TAGChassis = "8492-Chassis";

    public enum gameColor {UNKNOWN, BLUE, RED}

    ;

    public static final int ChassisMode_Stop = 0;
    public static final int ChassisMode_Drive = 1;
    public static final int ChassisMode_Turn = 2;
    public static final int ChassisMode_Idle = 3;
    public static final int ChassisMode_TeleOp = 4;
    //Max speeds based on how high the lifter is.  Only in TeleOp
    public static final double ChassisPower_BOTTOM_MAX = 1;
    public static final double ChassisPower_CARRY_MAX = 1;
    public static final double ChassisPower_STACK1_MAX = .6;
    public static final double ChassisPower_STACK2_MAX = .4;
    //Gyro KP for driving straight
    public static final double chassis_KPGyroStraight = .02;
    //for truning this is the tolerance of trun in degrees
    public static final int chassis_GyroHeadingTol = 2;
    //Timeout value for executing turns
    public static final int chassis_TurnTimeout_mS = 4000;
    public static final double chassis_driveTolInches = .25;
    public static final int chassis_driveTimeout_mS = 5000;
    public static final int ticksPerMotorOutputRev = 1120;
    public static final double wheelDistPreRev = 4 * 3.14159;
    public static final double gearRatio = 80 / 80;   // Motor Gear over Wheel Gear
    public static final double ticksPerInch = ticksPerMotorOutputRev / gearRatio / wheelDistPreRev;

    //From http://www.revrobotics.com/content/docs/HDMotorEncoderGuide.pdf
    //Page 6
    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    //current mode of operation for chassis
    private int ChassisMode_Current = ChassisMode_Stop;
    private int cmdStartTime_mS = 0;
    private int currentheading;
    private boolean cmdComplete = true;
    //LDM=leftDriveMotor
    //RDM=rightDriveMotor
    private DcMotor LDM1 = null;
    private DcMotor LDM2 = null;
    private DcMotor RDM1 = null;
    private DcMotor RDM2 = null;
    private ElapsedTime runtime = new ElapsedTime();
    //Target values go here.   The robot is trying to get to these values
    private double TargetMotorPowerLeft = 0;
    private double TargetMotorPowerRight = 0;
    private int TargetHeadingDeg = 0;
    private double TargetDistanceInches = 0;
    private double TurnMotorPowerLeft = 0;
    private double TurnMotorPowerRight = 0;

    @Override
    public void init() {

        // telemetry.addData("Status", "Initialized");
        composeTelemetry();
        //telemetry.log().add("Waiting for start...");


        RDM1 = hardwareMap.dcMotor.get("RDM1");
        LDM1 = hardwareMap.dcMotor.get("LDM1");
        LDM2 = hardwareMap.dcMotor.get("LDM2");
        RDM2 = hardwareMap.dcMotor.get("RDM2");

        if (LDM1 == null) {
            telemetry.log().add("LDM1 is null...");
        }

        if (LDM2 == null) {
            telemetry.log().add("LDM2 is null...");
        }

        if (RDM1 == null) {
            telemetry.log().add("RDM1 is null...");
        }

        if (RDM2 == null) {
            telemetry.log().add("RDM2 is null...");
        }

        LDM1.setDirection(DcMotor.Direction.FORWARD);
        LDM2.setDirection(DcMotor.Direction.FORWARD);
        RDM1.setDirection(DcMotor.Direction.REVERSE);
        RDM2.setDirection(DcMotor.Direction.REVERSE);

        LDM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LDM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //DriveMotorEncoderReset();

        LDM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RDM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LDM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RDM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LDM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LDM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        RDM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RDM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        boolean bLedOn = true;

    }

    //private LightSensor lightSensorLineFollow = null;

    /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
    @Override
    public void init_loop() {
    }

    private void setMotorMode(DcMotor.RunMode newMode) {

        LDM1.setMode(newMode);
        RDM1.setMode(newMode);
        LDM2.setMode(newMode);
        RDM2.setMode(newMode);
    }

    public void setMotorMode_RUN_WITHOUT_ENCODER() {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
        currentheading = gyroNormalize(getGyroHeading());
        RobotLog.aa(TAGChassis, "Mode: " + ChassisMode_Current);
        RobotLog.aa(TAGChassis, "loop() currHeading: " + currentheading + " target: " + TargetHeadingDeg);

        RobotLog.aa(TAGChassis, "Runtime: " + runtime.seconds());


        if (ChassisMode_Stop == ChassisMode_Current) {
            Dostop();
        }

        if (ChassisMode_Drive == ChassisMode_Current) {
            DoDrive();
        }

        if (ChassisMode_Turn == ChassisMode_Current) {
            DoTurn();
        }


        telemetry.update();
        telemetry.addLine("chassisMode" + ChassisMode_Current);

    } //loop()

    private void Dostop() {
       /*
        * executes the logic needed to stop the chassis
       */

        //set start speeds to zero
        TargetMotorPowerLeft = 0;
        TargetMotorPowerRight = 0;
        TargetDistanceInches = 0;
        //stop the motors
        LDM1.setPower(TargetMotorPowerLeft);
        LDM2.setPower(TargetMotorPowerLeft);
        RDM1.setPower(TargetMotorPowerRight);
        RDM2.setPower(TargetMotorPowerRight);

        //goto chassis idle mode
        ChassisMode_Current = ChassisMode_Idle;
        DriveMotorEncoderReset();
        RobotLog.aa(TAGChassis, "STOP  leftpower: " + TargetMotorPowerLeft + " rightpower: " + TargetMotorPowerRight);

    }  //dostop()

    private void DoDrive() {
        /*
        * executes the logic for a single scan of driving straight by gyro
        */
        double delta = -deltaHeading(gyroNormalize(getGyroHeading()), TargetHeadingDeg);

        double leftPower = TargetMotorPowerLeft - (delta * chassis_KPGyroStraight);
        double rightPower = TargetMotorPowerRight + (delta * chassis_KPGyroStraight);


        if (leftPower < -1) {
            leftPower = -1;
        }
        if (rightPower < -1) {
            rightPower = -1;
        }

        if (leftPower > 1) {
            leftPower = 1;
        }
        if (rightPower > 1) {
            rightPower = 1;
        }
        RobotLog.aa(TAGChassis, "DoDrive  leftpower: " + leftPower + " rightpower: " + rightPower + " delta: " + delta);


        LDM1.setPower(leftPower);
        LDM2.setPower(leftPower);
        RDM1.setPower(rightPower);
        RDM2.setPower(rightPower);
        ChassisMode_Current = ChassisMode_Drive;

        double inchesTraveled = Math.abs(getEncoderInches());
        //if ((inchesTraveled >= (Math.abs(TargetDistanceInches) - chassis_driveTolInches)) ||
        //        (runtime.milliseconds() > chassis_driveTimeout_mS))

        if (inchesTraveled >= Math.abs(TargetDistanceInches - chassis_driveTolInches)) {
            cmdComplete = true;
            Dostop();
        }
        telemetry.addLine(" MP " + TargetMotorPowerLeft + " IT = " + inchesTraveled);
        RobotLog.aa(TAGChassis, "DoDrive - MP " + TargetMotorPowerLeft + " IT = " + inchesTraveled);

    } // DoDrive()

    private void DoTurn() {
        /*
        *   executes the logic of a single scan of turning the robot to a new heading
         */

        int currHeading = gyroNormalize(getGyroHeading());
        RobotLog.aa(TAGChassis, "Turn currHeading: " + currHeading + " target: " + TargetHeadingDeg + " LP: " + LDM1.getPower() + " RP: " + RDM1.getPower());
        RobotLog.aa(TAGChassis, "Runtime: " + runtime.seconds());

        if (gyroInTol(currHeading, TargetHeadingDeg, chassis_GyroHeadingTol * 5)) {

            if (gyroInTol(currHeading, TargetHeadingDeg, chassis_GyroHeadingTol)) {
                //We are there stop
                cmdComplete = true;
                ChassisMode_Current = ChassisMode_Stop;
                Dostop();
                RobotLog.aa(TAGChassis, "Complete currHeading: " + currHeading);}
else{// getting in range, slow down
                    if (RDM1.getPower() > .2) {

                        TurnMotorPowerRight = RDM1.getPower() / 1.5;
                        TurnMotorPowerLeft = LDM1.getPower() / 1.5;
                    }
                    cmdTurn(TurnMotorPowerLeft, TurnMotorPowerRight, TargetHeadingDeg);
/*                LDM1.setPower(LDM1.getPower() / 2);
                LDM2.setPower(LDM2.getPower() / 2);
                RDM1.setPower(RDM1.getPower() / 2);
                RDM2.setPower(RDM2.getPower() / 2);LDM2.getPower() / 2
*/
                }
            }
        }  //DoTurn

    public void DriveMotorEncoderReset() {

        LDM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LDM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LDM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RDM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LDM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RDM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }  // DriveMotorEncoderReset

    public void cmdDrive(double speed, int headingDeg, double inches) {
        /*
        called by other opmodes to start a drive straight by gyro command
         */
        RobotLog.aa(TAGChassis, "cmdDrive: speed: " + speed + " heading: " + headingDeg + " inches: " + inches);
//        ChassisMode_Current = ChassisMode_Drive;
        cmdComplete = false;

        TargetHeadingDeg = headingDeg;
        TargetMotorPowerLeft = speed;
        TargetMotorPowerRight = speed;
        TargetDistanceInches = inches;
        runtime.reset();
        cmdComplete = false;
        if (ChassisMode_Current != ChassisMode_Drive) {
            DriveMotorEncoderReset();
            ChassisMode_Current = ChassisMode_Drive;
        }
        DoDrive();
    }  // cmdDrive

    public boolean getcmdComplete() {

        return (cmdComplete);
    }

    public void cmdTurn(double LSpeed, double RSpeed, int headingDeg) {
        //can only be called one time per movement of the chassis
        ChassisMode_Current = ChassisMode_Turn;
        TargetHeadingDeg = headingDeg;
        RobotLog.aa(TAGChassis, "cmdTurn target: " + TargetHeadingDeg);

// /        DriveMotorEncoderReset();
        LDM1.setPower(LSpeed);
        LDM2.setPower(LSpeed);
        RDM1.setPower(RSpeed);
        RDM2.setPower(RSpeed);
        cmdComplete = false;
        runtime.reset();
        DoTurn();
    }  // cmdTurn

    public int deltaHeading(int currHeading, int targetHeading) {


        //float signumCurrHeading = Math.signum (currHeading);
        //float signumTargetHeading = Math.signum (targetHeading);
        int returnValue = 0;

        //Positive value
        if (currHeading >= 0 && targetHeading >= 0) {
            returnValue = targetHeading - currHeading;
        }
        // one of each
        else if (currHeading >= 0 && targetHeading <= 0) {
            returnValue = (targetHeading + currHeading);
        }
        //one of each again
        else if (currHeading <= 0 && targetHeading >= 0) {
            returnValue = -1 * (targetHeading + currHeading);
        }
        // both negative
        else if (currHeading <= 0 && targetHeading <= 0) {
            returnValue = targetHeading - currHeading;
        }

        return returnValue;
    }  // deltaHeading

    public int getGyroHeading() {
        //Read the gyro and return its reading in degrees

        //this should pull heading angle from onboard IMU Gyro
        //https://ftcforum.usfirst.org/forum/ftc-technology/49904-help-with-rev-expansion-hub-integrated-gyro
        //hint: composeTelemetry() also captures this information below.

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //return formatAngle(angles.angleUnit, angles.firstAngle);
        return -1 * (int) (angles.firstAngle);
    }

    public double getEncoderInches() {
        // read the values from the encoders
        // LDM1.getCurrentPosition()
        // convert that to inches
        // by dividing by ticksPerInch

        // average the distance traveled by each wheel to determine the distance travled by the
        // robot

        int totaltics = Math.abs(LDM1.getCurrentPosition()) +
                Math.abs(LDM2.getCurrentPosition()) +
                Math.abs(RDM1.getCurrentPosition()) +
                Math.abs(RDM2.getCurrentPosition());
        double averagetics = totaltics / 4;
        double inches = averagetics / ticksPerInch;

        return inches;
    } // getEncoderInches

    public void doTeleOp(double LDMpower, double RDMpower) {

        ChassisMode_Current = ChassisMode_TeleOp;

        double LDMpower_signum = Math.signum(-1 * LDMpower);
        double RDMpower_signum = Math.signum(-1 * RDMpower);

        double LDM_new_Power = Math.abs(LDMpower);
        double RDM_new_Power = Math.abs(RDMpower);


        //Set the power based new calculations and clamping
        LDM1.setPower(LDMpower_signum * LDM_new_Power);
        LDM2.setPower(LDMpower_signum * LDM_new_Power);
        RDM1.setPower(RDMpower_signum * RDM_new_Power);
        RDM2.setPower(RDMpower_signum * RDM_new_Power);
    }  // doTeleOp

    /*
        * Code to run ONCE when the driver hits PLAY
        */
    @Override
    public void start() {
        runtime.reset();

    }

    @Override
    public void stop() {

        //go to brake mode at the end of program
        LDM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LDM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RDM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RDM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public int gyroNormalize(int heading) {
        // takes the full turns out of heading
        // gives us values from 0 to 180 for the right side of the robot
        // and values from 0 to -179 degrees for the left side of the robot

        int degrees = heading % 360;

        if (degrees > 180) {
            degrees = degrees - 360;
        }

        if (degrees < -179) {
            degrees = degrees + 360;
        }

        return (degrees);
    } // gyroNormalize

    public boolean gyroInTol(int currHeading, int desiredHeading, int tol) {
        //checks to see if we've reached the target heading within the tolorance specified

        int upperTol = gyroNormalize(desiredHeading + tol);
        int lowerTol = gyroNormalize(desiredHeading - tol);
        int normalCurr = gyroNormalize(currHeading);

        float signumUpperTol = Math.signum(upperTol);  // get the positive or negative sign
        float signumLowerTol = Math.signum(lowerTol);

        RobotLog.aa(TAGChassis, "gyrointol: UT " + upperTol + " LT: " + lowerTol + " normCurr: " + normalCurr);
//
        boolean retValue = false;
        // works for all positive numbers direction values
        if (signumUpperTol > 0 && signumLowerTol > 0) {
            if ((normalCurr >= lowerTol) && (normalCurr <= upperTol)) {
                retValue = true;
            }
        }

        // works for negative values
        else if (signumUpperTol < 0 && signumLowerTol < 0) {
            if ((normalCurr >= lowerTol) && (normalCurr <= upperTol)) {
                retValue = true;
            }
        }
        // mixed values -tol to + tol  This happens at 180 degrees
        else if ((signumUpperTol < 0) && (signumLowerTol > 0)) {
            // System.out.println("upperTol " + upperTol + " Current " +
            // normalCurr + " lowerTol " + lowerTol);
            if ((Math.abs(normalCurr) >= Math.abs(lowerTol)) &&
                    (Math.abs(normalCurr) >= Math.abs(upperTol))) {
                retValue = true;
            }

        }
        // mixed values -tol to + tol  This happens at 0 degrees
        else if ((signumUpperTol > 0) && (signumLowerTol < 0)) {
            // System.out.println("upperTol " + upperTol + " Current " +
            // normalCurr + " lowerTol " + lowerTol);
            if ((Math.abs(normalCurr) <= Math.abs(lowerTol)) &&
                    (Math.abs(normalCurr) <= Math.abs(upperTol))) {
                retValue = true;
            }

        }
        return (retValue);
    } // gryoInTol

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    //  public enum gameColor {UNKNOWN, BLUE, RED}


}


