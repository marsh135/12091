
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "ND_Mecanum_TF_Vu", group = "ND:")
//@TeleOp(name = "ND_Mecanum_TF_Vu", group = "ND:")

public class ND_Mecanum_TF_Vu extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: Gobilda MotorsMotor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = .667 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     numerator               = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     denominator             = WHEEL_DIAMETER_INCHES * 3.14159;
    static final double     COUNTS_PER_INCH         = numerator / denominator;
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.1;
    static final double     STRAFE_SPEED            = 0.5;
    static final double     INTAKE_DRIVE_SPEED      = 0.2;
    
    static final double     STRAFE_DISTANCE         = 12.0;
    static final double     DRIVE_DISTANCE          = 40.0;
    static final double     INTAKE_DISTANCE         = 20.0;
    static final double     TIMEOUT                 = 5.0;
    static final int        armTarget               = -12250;
    
    
    
    private DcMotor rightRear;
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor intake;
    private DcMotor grabberArm;
    
    private TouchSensor touch;
    int pos = 0;
    int count = 0;

    private static final String VUFORIA_KEY = "AeOnfqX/////AAABmZDM2GkZfE8xpIvDEnx14h55V20QbXA9grzAh+Rhj28Egy33NrxsyZRF58oJKYVvumMA5TJnIyBbwMOFTu8GCAWqueDi1nJLgSqlPLnS1RLJL8nEbs6QqwUXkr+5sFdHaCN7K+AwKi22uiav9fJKDVtWdN+eJcR+kevJ0tsv/vbr/QjF/mwgD9TNh7DDQElMvh6hdGp3dhzZXPXbqqip5Wy06rT0CaN0zj0BKW6YbRNrk52558Au8TXGP52/gBz+lmaptmj9QQsezvGdpUjKMPUhXQZk13JnXrX1oRWkT2iQXdG3dYH5ECrelWQJol9/vTBLwlE1TZriHVW0EH/A/0JQmVvJbrAp87se1UYaeq4d";


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        //sleep(1000);     // pause for servos to move
        
        initVuforia();
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        touch = hardwareMap.touchSensor.get("touch");
        intake = hardwareMap.dcMotor.get("intake");
        grabberArm = hardwareMap.dcMotor.get("grabberArm");
        
        //rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        this.resetStartTime();
        

        /*if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        sleep(500);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabberArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        telemetry.addData("Status", "Run Encoders");    //
        telemetry.update();
        sleep(500);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabberArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                          leftFront.getCurrentPosition(),
                          leftRear.getCurrentPosition(),
                          rightFront.getCurrentPosition(),
                          rightRear.getCurrentPosition());
        telemetry.update();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
            }
            while (opModeIsActive()) {
                grabberArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabberArm.setTargetPosition(armTarget);
                grabberArm.setPower(1);
                sleep(5000);
                
                telemetry.update();
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                          } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                          } else {
                            silverMineral2X = (int) recognition.getLeft();
                          }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                          if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            pos = 1;
                            encoderDrive(DRIVE_SPEED,  -DRIVE_DISTANCE,  -DRIVE_DISTANCE, -DRIVE_DISTANCE, -DRIVE_DISTANCE, TIMEOUT);  // S1: Forward 47 Inches with 5 Sec timeout
                            sleep(1000);

                            encoderDrive(STRAFE_SPEED,  STRAFE_DISTANCE, -STRAFE_DISTANCE, -STRAFE_DISTANCE, STRAFE_DISTANCE, TIMEOUT);  // S2: Turn Right 12 Inches with 4 Sec timeout
                            //sarah is the goat
                            sleep(1000);
                            intake.setPower(1);
                            sleep(1000);
                            encoderDrive(INTAKE_DRIVE_SPEED,  -INTAKE_DISTANCE,  -INTAKE_DISTANCE, -INTAKE_DISTANCE, -INTAKE_DISTANCE, TIMEOUT);
                            intake.setPower(0);
                            grabberArm.setTargetPosition(-1*armTarget);
                            count++;

                        //encoderDrive(STRAFE_SPEED,  -30.0, 30.0, 30.0, -30.0, 10.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
                            //encoderDrive(DRIVE_SPEED, -30.0, -30.0, -30.0, -30.0, 10.0); 
                          } 
                          else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) 
                          {
                            telemetry.addData("Gold Mineral Position", "Right");
                            telemetry.addData("Going right","");
                            pos = 3;
                            encoderDrive(DRIVE_SPEED,  -DRIVE_DISTANCE,  -DRIVE_DISTANCE, -DRIVE_DISTANCE, -DRIVE_DISTANCE, TIMEOUT);  // S1: Forward 47 Inches with 5 Sec timeout
                        //encoderDrive(STRAFE_SPEED,  30.0, -30.0, -30.0, 30.0, 10.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
                            encoderDrive(STRAFE_SPEED,  -STRAFE_DISTANCE, STRAFE_DISTANCE, STRAFE_DISTANCE, -STRAFE_DISTANCE, TIMEOUT);  // S2: Turn Right 12 Inches with 4 Sec timeout
                            intake.setPower(1);
                            encoderDrive(INTAKE_DRIVE_SPEED,  -INTAKE_DISTANCE,  -INTAKE_DISTANCE, -INTAKE_DISTANCE, -INTAKE_DISTANCE, TIMEOUT);
                            intake.setPower(0);
                            grabberArm.setTargetPosition(-1*armTarget);
                            count++;
                            

                            //encoderDrive(DRIVE_SPEED, -30.0, -30.0, -30.0, -30.0, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeout
                            
                          } 
                          else 
                          {
                            telemetry.addData("Gold Mineral Position", "Center");
                            
                            encoderDrive(DRIVE_SPEED,  -DRIVE_DISTANCE,  -DRIVE_DISTANCE, -DRIVE_DISTANCE, -DRIVE_DISTANCE, TIMEOUT); 
                            
                            intake.setPower(1);
                            
                            encoderDrive(INTAKE_DRIVE_SPEED,  -INTAKE_DISTANCE,  -INTAKE_DISTANCE, -INTAKE_DISTANCE, -INTAKE_DISTANCE, TIMEOUT);
                            intake.setPower(0);
                            grabberArm.setTargetPosition(-1*armTarget);
                            count++;
                             
                          }

                      
                        }

                      }
                      telemetry.update();
                     
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    } 
    
    public void encoderDrive(double speed,
                             double leftFrontE, double rightFrontE, double leftRearE,
                             double rightRearE, double timeout) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget1;
        int newRightTarget1;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = (int)(leftRearE * COUNTS_PER_INCH);
            newRightTarget = (int)(rightRearE * COUNTS_PER_INCH);
            newLeftTarget1 = (int)(leftFrontE * COUNTS_PER_INCH);
            newRightTarget1 = (int)(rightFrontE * COUNTS_PER_INCH);
            leftFront.setTargetPosition(newLeftTarget1);
            rightFront.setTargetPosition(newRightTarget1);
            leftRear.setTargetPosition(newLeftTarget);
            rightRear.setTargetPosition(newRightTarget);
            

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));
            leftRear.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   runtime.seconds() < timeout &&
                   leftFront.isBusy() && rightFront.isBusy() && 
                   leftRear.isBusy() && rightRear.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d : %7d", newLeftTarget,  newRightTarget, newLeftTarget1,  newRightTarget1 );
                
                telemetry.update();
                
                
                
            }
            
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Stop all motion;
            leftRear.setPower(0);
            rightFront.setPower(0);
            leftFront.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(1000);   // optional pause after each move
            
            //Strafe
            
        }
    }

}
