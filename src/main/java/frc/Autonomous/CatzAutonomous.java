package frc.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.*;

public class CatzAutonomous 
{
    //drive straight variables
    public Boolean startDriving = false;

    final double DRV_S_GEAR_RATIO = 6.75;       //SDS mk4i L2 ratio
    final double DRV_S_THREAD_PERIOD = 0.02;

    final double TALONFX_INTEGRATED_ENC_CNTS_PER_REV = 2048.0;

    final double DRVTRAIN_WHEEL_DIAMETER             = 4.0;
    final double DRVTRAIN_WHEEL_CIRCUMFERENCE        = (Math.PI * DRVTRAIN_WHEEL_DIAMETER);
    final double DRVTRAIN_ENC_COUNTS_TO_INCH         = DRVTRAIN_WHEEL_CIRCUMFERENCE / TALONFX_INTEGRATED_ENC_CNTS_PER_REV / DRV_S_GEAR_RATIO;

    private double DRV_S_STOP_DISTANCE = 0.5;
    private double DRV_S_MIN_POWER     = 0.1;

    private double DRV_S_ERROR_GAIN    = 0.05;
    private double DRV_S_RATE_GAIN     = 0.001;

    //public  double drvSdistance = 0.0;

    //turn in place variables
    private final static double PID_TURN_THRESHOLD   = 1.25;

	private final static double PID_TURN_IN_PLACE_KP = 0.008;
    
    private final static double TURN_DRIVE_MAX_POS_POWER  =  0.4;
	private final static double TURN_DRIVE_MAX_NEG_POWER  = -0.4;

    private final static double TURN_DRIVE_MIN_POWER = 0.1;

    private final static double TURN_IN_PLACE_PERIOD = 0.010;


	private static double turnCurrentError; 

	private static double turnCurrentAngle;
	private static double turnTargetAngle;

    private Timer autonTimer;
    public CatzLog data;



    public CatzAutonomous()
    {
        autonTimer = new Timer();
    }



    /*-----------------------------------------------------------------------------------------
    *
    *  DriveStraight
    *
    *----------------------------------------------------------------------------------------*/
    public void DriveStraight(double distance, double decelDistance, double maxSpeed, double drvSwheelPos, double maxTime)
    {
        double distanceRemain = 0.0;
        double targetPower    = 0.0;
        double turnPower = 0.0;
        double angleError = 0.0;
        double prevAngleError = 0.0;
        double time = 0.0;
        double prevTime = -1.0; // no big initial rate
        double angleErrorRate = 0;

        Boolean drvSbackwards;

        double startingPosition = 0.0;
        double currentPosition  = 0.0;

        double currentAngle     = 0.0;
        double startingAngle    = 0.0;
        
        //this.drvSwheelPos = drvSwheelPos;
        //this.drvSdistance = distance;

        startDriving = true;

        if(maxSpeed < 0)
        {
            drvSbackwards = true;
        }
        else
        {
            drvSbackwards = false;
        }

        currentPosition    = Robot.drivetrain.LT_FRNT_MODULE.getDriveMotorPosition();
        startingPosition   = currentPosition;

        startingAngle    = Robot.navX.getAngle(); // change for final 2/4 EL
        distanceRemain = distance;

        autonTimer.reset();
        autonTimer.start();

        while(Math.abs(distanceRemain) >= DRV_S_STOP_DISTANCE && startDriving && time < maxTime)
        {
            time             = autonTimer.get();
            currentPosition  = Robot.drivetrain.LT_FRNT_MODULE.getDriveMotorPosition();
            currentAngle     = Robot.navX.getAngle();
            
            angleError = startingAngle - currentAngle;
            angleErrorRate    = (angleError - prevAngleError) / (time - prevTime);

            distanceRemain = distance - (Math.abs(currentPosition - startingPosition) * DRVTRAIN_ENC_COUNTS_TO_INCH);
            targetPower    = Clamp(-1.0, distanceRemain / distance / decelDistance, 1.0) * maxSpeed;
            
            turnPower      = Clamp(-1.0, DRV_S_ERROR_GAIN * angleError + DRV_S_RATE_GAIN * angleErrorRate, 1.0); //"-" in front of Error and Rate
            
            if(Math.abs(targetPower) < DRV_S_MIN_POWER)
            {
                targetPower = DRV_S_MIN_POWER * Math.signum(targetPower);
            }

            if(drvSbackwards)
            {
                turnPower = -turnPower;
            }

            Robot.drivetrain.translateTurn(drvSwheelPos, targetPower, turnPower, Robot.drivetrain.getGyroAngle()); //TBD need to check

            prevTime      = time;
            prevAngleError = angleError;

            //if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_DRV_STRAIGHT)
            {
                //TBD Use Time and position used in calculations
                data = new CatzLog(time, distanceRemain, currentPosition, 
                                                             targetPower, 
                                                             angleError, 
                                                             currentAngle, 
                                                             angleErrorRate, 
                                                             turnPower, 
                                                             Robot.drivetrain.LT_FRNT_MODULE.getAngle(),
                                                             Robot.drivetrain.LT_BACK_MODULE.getAngle(),
                                                             Robot.drivetrain.RT_FRNT_MODULE.getAngle(),
                                                             Robot.drivetrain.RT_BACK_MODULE.getAngle(),
                                                             0.0, 0.0, 0.0, 0);  
                Robot.dataCollection.logData.add(data);
            }

            Timer.delay(DRV_S_THREAD_PERIOD);
        }

        System.out.println("Done driving");
        Robot.drivetrain.autoDrive(0);

        startDriving = false;
    }


    /*-----------------------------------------------------------------------------------------
    *
    *  TurnInPlace
    *
    *----------------------------------------------------------------------------------------*/
    public void TurnInPlace(double degreesToTurn, double timeoutSeconds)
    {
        boolean turnInPlaceDone = false;

        double  currentTime       = 0.0;
        double  angleRemainingAbs = 999.0;
        double  turnPower = 0.0;

        autonTimer.reset();
        autonTimer.start(); 

        turnCurrentAngle  = Robot.navX.getAngle();
        turnTargetAngle   = degreesToTurn + turnCurrentAngle;

        while (turnInPlaceDone == false)
        {
            currentTime  = autonTimer.get();
            turnCurrentAngle = Robot.navX.getAngle();
    
            // calculate error
            turnCurrentError      = turnTargetAngle - turnCurrentAngle;
            angleRemainingAbs = Math.abs(turnCurrentError);

            if (angleRemainingAbs < PID_TURN_THRESHOLD) 
            { 
                turnInPlaceDone = true;
                Robot.drivetrain.rotateInPlace(0.0);
            }
            else
            {
                if (currentTime > timeoutSeconds) 
                {
                    turnInPlaceDone = true;
                    Robot.drivetrain.rotateInPlace(0.0);
                } 
                else
                {
                    turnPower = turnCurrentError * PID_TURN_IN_PLACE_KP;

                    //Clamp
                    //MAX POWER
                    if(turnPower >= TURN_DRIVE_MAX_POS_POWER)
                    {
                        turnPower = TURN_DRIVE_MAX_POS_POWER;
                    } 
                    else if(turnPower <= TURN_DRIVE_MAX_NEG_POWER)
                    {
                        turnPower = TURN_DRIVE_MAX_NEG_POWER;
                    }

                    //MIN POWER
                    if (Math.abs(turnPower) < TURN_DRIVE_MIN_POWER)
                    {
                        turnPower = Math.signum(turnPower) * TURN_DRIVE_MIN_POWER;
                    }
                    
                    Robot.drivetrain.rotateInPlace(turnPower);
                }
            }

            if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_TURN_IN_PLACE)
            {
            data = new CatzLog(currentTime, turnCurrentAngle, turnCurrentError, turnPower, 
                                -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999);

            Robot.dataCollection.logData.add(data);
            }

            Timer.delay(TURN_IN_PLACE_PERIOD);
        }
    }

    public void StopDriving()
    {
        startDriving = false;
    }

    public double Clamp(double min, double in, double max)
    {
        if(in > max)
        {
            return max;
        }
        else if(in < min)
        {
            return min;
        }
        else
        {
            return in;
        }
    }

    public double getDistance()
    {
        return -999.9;  //TBD  drvSdistance;
    }
}
