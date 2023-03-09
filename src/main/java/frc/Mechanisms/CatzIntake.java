package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;

public class CatzIntake {

  private Thread intakeThread;
  private final double INTAKE_THREAD_PERIOD  = 0.020;
  
  public CatzLog data;

  //------------------------------------------------------------------------------------------------
  //
  //  Roller
  //
  //------------------------------------------------------------------------------------------------
  private WPI_TalonFX intakeRollerMotor;

  private final int    INTAKE_ROLLER_MC_ID           = 11; 

  private final double INTAKE_MOTOR_POWER_ROLLER_IN  = -1.0;
  private final double INTAKE_MOTOR_POWER_ROLLER_OUT =  1.0;
  private final double INTAKE_MOTOR_POWER_ROLLER_OFF =  0.0;

  private final double INTAKE_INPUT_THRESHOLD = 0.2;

  public final int INTAKE_ROLLER_OFF = 0;  //TBD to see if still needed
  public final int INTAKE_ROLLER_IN  = 1;
  public final int INTAKE_ROLLER_OUT = 2;
  public final int INTAKE_ROLLER_UNINITIALIZED = -999;

  public int   intakeRollerState = INTAKE_ROLLER_OFF;


  //------------------------------------------------------------------------------------------------
  //
  //  Deploy/Stow and Pivot
  //
  //------------------------------------------------------------------------------------------------
  private WPI_TalonFX intakePivotMotor;
  
  private final int     INTAKE_PIVOT_MC_ID           = 10; 
  private final double  INTAKE_PIVOT_REL_ENCODER_CPR = 2048.0;

  private final double  CURRENT_LIMIT_AMPS            = 60.0;
  private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0;
  private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
  private final boolean ENABLE_CURRENT_LIMIT          = true;

  private SupplyCurrentLimitConfiguration EncCurrentLimit;


  //Pivot status
  private  final int INTAKE_PIVOT_MODE_NULL                = 0;
  private  final int INTAKE_PIVOT_MODE_DEPLOY              = 1;
  private  final int INTAKE_PIVOT_MODE_DEPLOY_CALC         = 10;
  private  final int INTAKE_PIVOT_MODE_FULLY_DEPLOYED      = 2;
  private  final int INTAKE_PIVOT_MODE_INITIALIZATION      = 3;
  private  final int INTAKE_PIVOT_MODE_STOW_CALC           = 14;
  private  final int INTAKE_MODE_STOW_HOLD                 = 5;

  //Pivot IntakeMode initialization
  private int intakePivotMode = INTAKE_PIVOT_MODE_NULL;


  //------------------------------------------------------------------------------------------------
  //  Gear ratio
  //------------------------------------------------------------------------------------------------
  private final double INTAKE_PIVOT_PINION_GEAR = 11.0;
  private final double INTAKE_PIVOT_SPUR_GEAR   = 56.0;  
  private final double INTAKE_PIVOT_GEAR_RATIO  =INTAKE_PIVOT_SPUR_GEAR/INTAKE_PIVOT_PINION_GEAR;

  private final double INTAKE_PIVOT_SPROCKET_1  = 16.0;
  private final double INTAKE_PIVOT_SPROCKET_2  = 56.0;
  private final double INTAKE_PIVOT_SPROCKET_RATIO  = INTAKE_PIVOT_SPROCKET_2/INTAKE_PIVOT_SPROCKET_1;
  
  private final double INTAKE_PIVOT_FINAL_RATIO = INTAKE_PIVOT_GEAR_RATIO*INTAKE_PIVOT_SPROCKET_RATIO;
  
  
  //------------------------------------------------------------------------------------------------
  //  Angle Definitions
  //------------------------------------------------------------------------------------------------
  private final static double INTAKE_STOWED_ANGLE   =  0.197;   //TBD - Angles are prototype only and will change with final hardstops
  private final static double INTAKE_DEPLOYED_ANGLE = 85.827;   //TBD angle need correct

  private final static double INTAKE_STOW_INITIAL_ANGLE   = INTAKE_DEPLOYED_ANGLE;
  private final static double INTAKE_DEPLOY_INITIAL_ANGLE = INTAKE_STOWED_ANGLE;


  //------------------------------------------------------------------------------------------------
  //  Motion Magic Approach
  //------------------------------------------------------------------------------------------------
  private final int    INTAKE_STOW_SLOT     = 0;        //TBD - up should be stow, down should be deploy; Also don't need PID in name
  private final double INTAKE_STOW_KP   = 0.08; 
  private final double INTAKE_STOW_KI   = 0.00;
  private final double INTAKE_STOW_KD   = 0.00;

  private final int    INTAKE_DEPLOY_SLOT   = 1;
  private final double INTAKE_DEPLOY_KP = 0.035;
  private final double INTAKE_DEPLOY_KI = 0.00;
  private final double INTAKE_DEPLOY_KD = 0.00;

  private final static double INTAKE_DEPLOY_SET_POSITION_START_ANGLE = 25.0;
  private final static double INTAKE_STOW_REDUCE_POWER_ANGLE         = 55.0;

  private final double INTAKE_DEPLOYED_ANGLE_COUNTS   = -(INTAKE_DEPLOYED_ANGLE * (INTAKE_PIVOT_REL_ENCODER_CPR / 360.0) * INTAKE_PIVOT_FINAL_RATIO);
  private final double INTAKE_STOWED_ANGLE_COUNTS     =  (INTAKE_STOWED_ANGLE   * (INTAKE_PIVOT_REL_ENCODER_CPR / 360.0) * INTAKE_PIVOT_FINAL_RATIO);
  
  private final double INTAKE_SOFTLIMIT_OFFSET_ANGLE        = 0.0;  //Setting to zero assuming deploy code will not run motor into the hard-stops
  private final double INTAKE_SOFTLIMIT_OFFSET_ANGLE_COUNTS = (INTAKE_SOFTLIMIT_OFFSET_ANGLE * (INTAKE_PIVOT_REL_ENCODER_CPR / 360.0) * INTAKE_PIVOT_FINAL_RATIO);

  private final double INTAKE_SOFTLIMIT_MAX_ANGLE_COUNTS    = INTAKE_DEPLOYED_ANGLE_COUNTS - INTAKE_SOFTLIMIT_OFFSET_ANGLE_COUNTS;
  private final double INTAKE_SOFTLIMIT_MIN_ANGLE_COUNTS    = INTAKE_STOWED_ANGLE_COUNTS   + INTAKE_SOFTLIMIT_OFFSET_ANGLE_COUNTS;


  //------------------------------------------------------------------------------------------------
  //  5th order polynomial approach
  //------------------------------------------------------------------------------------------------
  //coeffients
  public  final double DEG2RAD = Math.PI / 180.0;

  private final double INTAKE_STOW_CALC_Kp    = 0.010;
  private final double INTAKE_STOW_CALC_Kd    = 0.001;
  private final double INTAKE_DEPLOYMENT_TIME = 0.26;

  public final double COEFF1  =  10.0;
  public final double COEFF2  = -15.0;
  public final double COEFF3  =   6.0;

  public final double INTAKE_INERTIA    = 0.61;  //kg * m^2 
  public final double INTAKE_MAX_TORQUE = 5.84;

  public final double B_DEPLOY  = (INTAKE_DEPLOYED_ANGLE - INTAKE_DEPLOY_INITIAL_ANGLE) / INTAKE_DEPLOYMENT_TIME;;
  public final double A3_DEPLOY = COEFF1 * B_DEPLOY / INTAKE_DEPLOYMENT_TIME / INTAKE_DEPLOYMENT_TIME;
  public final double A4_DEPLOY = COEFF2 * B_DEPLOY / INTAKE_DEPLOYMENT_TIME / INTAKE_DEPLOYMENT_TIME / INTAKE_DEPLOYMENT_TIME;
  public final double A5_DEPLOY = COEFF3 * B_DEPLOY / INTAKE_DEPLOYMENT_TIME / INTAKE_DEPLOYMENT_TIME / INTAKE_DEPLOYMENT_TIME / INTAKE_DEPLOYMENT_TIME;

  public final double B_STOW    = (INTAKE_STOWED_ANGLE - INTAKE_STOW_INITIAL_ANGLE) / INTAKE_DEPLOYMENT_TIME;
  public final double A3_STOW   = COEFF1 * B_STOW   / INTAKE_DEPLOYMENT_TIME / INTAKE_DEPLOYMENT_TIME;
  public final double A4_STOW   = COEFF2 * B_STOW   / INTAKE_DEPLOYMENT_TIME / INTAKE_DEPLOYMENT_TIME / INTAKE_DEPLOYMENT_TIME;  
  public final double A5_STOW   = COEFF3 * B_STOW   / INTAKE_DEPLOYMENT_TIME / INTAKE_DEPLOYMENT_TIME / INTAKE_DEPLOYMENT_TIME / INTAKE_DEPLOYMENT_TIME;
  
  public final double ALPHA3_DEPLOY = (A3_DEPLOY * DEG2RAD * INTAKE_INERTIA) / INTAKE_PIVOT_FINAL_RATIO / INTAKE_MAX_TORQUE;
  public final double ALPHA4_DEPLOY = (A4_DEPLOY * DEG2RAD * INTAKE_INERTIA) / INTAKE_PIVOT_FINAL_RATIO / INTAKE_MAX_TORQUE;
  public final double ALPHA5_DEPLOY = (A5_DEPLOY * DEG2RAD * INTAKE_INERTIA) / INTAKE_PIVOT_FINAL_RATIO / INTAKE_MAX_TORQUE;

  public final double ALPHA3_STOW   = (A3_STOW   * DEG2RAD * INTAKE_INERTIA) / INTAKE_PIVOT_FINAL_RATIO / INTAKE_MAX_TORQUE;
  public final double ALPHA4_STOW   = (A4_STOW   * DEG2RAD * INTAKE_INERTIA) / INTAKE_PIVOT_FINAL_RATIO / INTAKE_MAX_TORQUE;
  public final double ALPHA5_STOW   = (A5_STOW   * DEG2RAD * INTAKE_INERTIA) / INTAKE_PIVOT_FINAL_RATIO / INTAKE_MAX_TORQUE;


  private final double INTAKE_PIVOT_DEPLOY_POWER = -0.2;    //TBD May not need depending on final implementation


  private final int INTAKE_PIVOT_STOWED = 0;       //TBD  MAY DELETE
  private final int INTAKE_PIVOT_DEPLOYED = 1;
  private final int INTAKE_PIVOT_IN_TRANSIT = 2;
  private final int INTAKE_PIVOT_UNINITIALIZED = -999;
  private int intakePivotState = INTAKE_PIVOT_STOWED;


  public double powerForMotor;
  public double finalMotorPower   = 0.0;

  public double angleDot          = 0.0;
  public double angleOld          = 0.0;
  public double currentAngle      = 0.0;
  public double targetAngle       = 0.0;
  public double targetAngularRate = 0.0;
  public double deltaAngle        = 0.0;

  public Timer  pivotTimer;
  public double time      = Timer.getFPGATimestamp();     //TBD - why this method? why are we saving this time? normally used with pivotTimer
  public double timeOld   = 0.0;
  public double deltaTime = 0.0;

  private double deploymentMotorRawPosition;


//---------------------------------------------definitions part end--------------------------------------------------------------
  
    /*-----------------------------------------------------------------------------------------
    *  
    *  CatzIntake()
    *
    *----------------------------------------------------------------------------------------*/
    public CatzIntake() {
    //need add softlimits

    intakeRollerMotor = new WPI_TalonFX(INTAKE_ROLLER_MC_ID);
    intakePivotMotor  = new WPI_TalonFX(INTAKE_PIVOT_MC_ID);

    EncCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, 
                                                                                CURRENT_LIMIT_TRIGGER_AMPS, 
                                                                                CURRENT_LIMIT_TIMEOUT_SECONDS);
    intakeRollerMotor.configFactoryDefault();
    intakeRollerMotor.setNeutralMode(NeutralMode.Coast);
    intakeRollerMotor.configSupplyCurrentLimit(EncCurrentLimit);


    intakePivotMotor.configFactoryDefault();
    intakePivotMotor.setNeutralMode(NeutralMode.Brake);
    intakePivotMotor.configSupplyCurrentLimit(EncCurrentLimit);
  
    intakePivotMotor.config_kP(INTAKE_STOW_SLOT, INTAKE_STOW_KP);
    intakePivotMotor.config_kI(INTAKE_STOW_SLOT, INTAKE_STOW_KI);
    intakePivotMotor.config_kD(INTAKE_STOW_SLOT, INTAKE_STOW_KD);

    intakePivotMotor.config_kP(INTAKE_DEPLOY_SLOT, INTAKE_DEPLOY_KP);
    intakePivotMotor.config_kI(INTAKE_DEPLOY_SLOT, INTAKE_DEPLOY_KI);
    intakePivotMotor.config_kD(INTAKE_DEPLOY_SLOT, INTAKE_DEPLOY_KD);

    intakePivotMotor.configForwardSoftLimitThreshold(INTAKE_SOFTLIMIT_MIN_ANGLE_COUNTS);
    intakePivotMotor.configReverseSoftLimitThreshold(INTAKE_SOFTLIMIT_MAX_ANGLE_COUNTS);

    intakePivotMotor.configForwardSoftLimitEnable(true);
    intakePivotMotor.configReverseSoftLimitEnable(true);

    pivotTimer = new Timer();
   
    intakeControl();

  }

    /*-----------------------------------------------------------------------------------------
    *  
    *  Roller Methods
    *
    *----------------------------------------------------------------------------------------*/
    public void intakeRollerIn()
    {
      intakeRollerMotor.set(ControlMode.PercentOutput,INTAKE_MOTOR_POWER_ROLLER_IN);
      intakeRollerState = INTAKE_ROLLER_IN;
    }

    public void intakeRollerOut()
    {
        intakeRollerMotor.set(ControlMode.PercentOutput,INTAKE_MOTOR_POWER_ROLLER_OUT);
        intakeRollerState = INTAKE_ROLLER_OUT;
    }


    public void intakeRollerOff()
    {
        intakeRollerMotor.set(ControlMode.PercentOutput,INTAKE_MOTOR_POWER_ROLLER_OFF);
        intakeRollerState = INTAKE_ROLLER_OFF;
    }


    public void procCmdRoller(double xboxValueIn, double xboxValueOut)
    {
      if (xboxValueIn > INTAKE_INPUT_THRESHOLD)
      {
        intakeRollerIn();      
      }
      else if (xboxValueOut > INTAKE_INPUT_THRESHOLD)
      { 
        intakeRollerOut();     
      }
      else
      {
        intakeRollerOff();
      }
    }

    

    /*-----------------------------------------------------------------------------------------
    *  
    *  Deploy/Stow Methods
    *
    *----------------------------------------------------------------------------------------*/
    public void intakeControl()
    {
      intakeThread = new Thread(() ->
      { 
        while(true)
        {
          switch(intakePivotMode)
          {
              case INTAKE_PIVOT_MODE_NULL:
                
              break;
               
              case INTAKE_PIVOT_MODE_DEPLOY:

                currentAngle = getIntakePositionDegrees();

                if(currentAngle > INTAKE_DEPLOY_SET_POSITION_START_ANGLE )
                {
                  intakePivotMotor.set(ControlMode.Position, INTAKE_DEPLOYED_ANGLE_COUNTS); //TBD review
                  //intakePivotMode = INTAKE_PIVOT_MODE_NULL;
                 // intakePivotMotor.set(ControlMode.MotionMagic, INTAKE_PIVOT_DEPLOY_ROTATION);
                }
                
              break;
              
              /* 
               case INTAKE_PIVOT_MODE_DEPLOY_CALC:
                System.out.println("in deploycalc");
                if(firstTimeThrough == true )
                {
                  pivotTimer.reset();
                  firstTimeThrough = false;
                }
                    
                currentAngle = getIntakePositionDegrees();
                time = pivotTimer.get();
                deltaAngle = currentAngle - angleOld;
                if(time > 0.01)
                {
                  deltaTime = time - timeOld;
                }
                else
                {
                  deltaTime = 100;//this is for initial time
                }
                angleOld = getIntakePositionDegrees();
                timeOld = time;
                angleDot = deltaAngle/deltaTime;
                powerForMotor = ALPHA3_DEPLOY * Math.pow(time , 3) - ALPHA4_DEPLOY *Math.pow(time , 4) + ALPHA5_DEPLOY *Math.pow(time , 5);
                
                targetAngle = (A3_DEPLOY*Math.pow(time , 3)) + (A4_DEPLOY*Math.pow(time , 4)) + (A5_DEPLOY*Math.pow(time , 5));
                targetAngularRate = (3 * A3_DEPLOY * Math.pow(time , 2)) + (4 * A4_DEPLOY * Math.pow(time , 3)) + (5 * A5_DEPLOY * Math.pow(time , 4));
                finalMotorPower = powerForMotor + Kp*(targetAngle - getIntakePositionDegrees()) + Kd*(targetAngularRate - deltaAngle/deltaTime); 
                finalMotorPower = -finalMotorPower;
                System.out.println("in deploy" + finalMotorPower);
                intakePivotMotor.set(ControlMode.PercentOutput,finalMotorPower);
                  
                currentAngle = getIntakePositionDegrees();
                if(currentAngle > INTAKE_DEPLOY_FINAL_ANGLE)
                {
                  firstTimeThrough = true;
                  timeOld = 0;
                  intakePivotMode = INTAKE_PIVOT_MODE_FULLY_DEPLOYED;
                }   
              break;
                
              case INTAKE_PIVOT_MODE_FULLY_DEPLOYED:
                intakePivotMotor.set(0);
              break;
*/
              
              case INTAKE_PIVOT_MODE_STOW_CALC:


                currentAngle = getIntakePositionDegrees();
                deltaAngle   = currentAngle - angleOld;

                time         = pivotTimer.get();

                if(time > 0.01)     //TBD - not sure we need this either
                {
                  deltaTime = time - timeOld;
                }
                else
                {
                  deltaTime = 100;// this is for initial time/ to provent a s       TBD - this doesn't seem right
                }

                angleOld = getIntakePositionDegrees();

                timeOld = time;//Math.pow(4,2)  TBD
                
                angleDot = deltaAngle / deltaTime;

                //TBD - can we calculate some of the Math.pow() terms once and just use?  Also make parens consistent
                double timePow3 = Math.pow(time, 3);
                double timePow4 = Math.pow(time, 4);
                double timePow5 = Math.pow(time, 5);

                powerForMotor     = (ALPHA3_STOW * timePow3         ) - (ALPHA4_STOW * timePow4) + (ALPHA5_STOW * timePow5); 
                targetAngle       = (    A3_STOW * timePow3         ) + (    A4_STOW * timePow4) + (    A5_STOW * timePow5);
                targetAngularRate = (3 * A3_STOW * Math.pow(time, 2)) + (4 * A4_STOW * timePow3) + (5 * A5_STOW * timePow4);

                finalMotorPower = powerForMotor + INTAKE_STOW_CALC_Kp * (targetAngle       - getIntakePositionDegrees()) + //TBD why not use currentAngle? 
                                                  INTAKE_STOW_CALC_Kd * (targetAngularRate - deltaAngle / deltaTime); //TBD - why not use angleDot?
                finalMotorPower = -finalMotorPower;
                System.out.println("finalMotor Power:" + finalMotorPower);

                intakePivotMotor.set(ControlMode.PercentOutput,finalMotorPower);

                currentAngle = getIntakePositionDegrees();        //TBD - This should be checked first
                if(currentAngle < INTAKE_STOW_REDUCE_POWER_ANGLE)       //TBD - This should be checked first
                {
                  timeOld = 0;
                  intakePivotMode = INTAKE_MODE_STOW_HOLD;  //TBD Set power to zero here and go to null
                } 
              break;

              case INTAKE_MODE_STOW_HOLD:
                intakePivotMotor.set(0);
              break;

              default:
                intakePivotMotor.set(INTAKE_MOTOR_POWER_ROLLER_OFF);
                intakeRollerOff();
              break;

          }   //eng of switch

          if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_INTAKE)
          {
            //TBD Use the time used for calculatiobns; Group angles together
            data = new CatzLog(Robot.currentTime.get(), targetAngularRate, targetAngle, 
                               finalMotorPower, powerForMotor, currentAngle, deltaAngle, deltaTime,
                               -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0,
                               DataCollection.boolData);  
            Robot.dataCollection.logData.add(data);
          }

          Timer.delay(INTAKE_THREAD_PERIOD);

        }  //end of while true
              
      });
      intakeThread.start();
    
    }   //end of intakeControl();


    /*-----------------------------------------------------------------------------------------
    *  
    * intakeDeploy()
    *
    *----------------------------------------------------------------------------------------*/
    public void intakeDeploy()
    {
      intakePivotMotor.set(ControlMode.PercentOutput,INTAKE_PIVOT_DEPLOY_POWER);
      intakePivotMode = INTAKE_PIVOT_MODE_DEPLOY;
    }

    public void intakeStow()
    {
      pivotTimer.reset();
      pivotTimer.start();

      intakePivotMode = INTAKE_PIVOT_MODE_STOW_CALC;
    }


    public void procCmdDeploy(boolean xboxValueDeploy, boolean xboxValueStow)
    {

        if (xboxValueDeploy == true)
        {
            if(Robot.elevatorState == Robot.DEPLOYED)
            {
                //flash colors indicating that you can't deploy
            }
            else
            {
              intakeDeploy();
              Robot.intakeState = Robot.DEPLOYED;
            }
        }
        else if (xboxValueStow == true)
        {
          intakeStow();
          Robot.intakeState = Robot.STOWED;
        }
    }


    /*-----------------------------------------------------------------------------------------
    *
    *  getIntakePositionDegrees
    *
    *----------------------------------------------------------------------------------------*/
    public double getIntakePositionDegrees()
    {
        deploymentMotorRawPosition = intakePivotMotor.getSelectedSensorPosition();

        double motorShaftRevolution = deploymentMotorRawPosition / INTAKE_PIVOT_REL_ENCODER_CPR;
        double pivotShaftRevolution = motorShaftRevolution       / INTAKE_PIVOT_FINAL_RATIO;
        double pivotAngle           = pivotShaftRevolution * -360.0; //motor  spin forward is positive 
        
        return pivotAngle;   
    }


    /*-----------------------------------------------------------------------------------------
    *  
    *  Smart Dashboard
    *
    *----------------------------------------------------------------------------------------*/
    public void smartDashboardIntake()
    {
        SmartDashboard.putNumber("PivotAngle", getIntakePositionDegrees());
       
    }

    public void smartDashboardIntake_Debug()
    {
      SmartDashboard.putNumber("PivotCounts", deploymentMotorRawPosition);
      SmartDashboard.putNumber("getClosedLoopError", intakePivotMotor.getClosedLoopError());
      SmartDashboard.putNumber("getClosedLoopTarget", intakePivotMotor.getClosedLoopTarget());
      SmartDashboard.putNumber("getStatorCurrent", intakePivotMotor.getStatorCurrent());
      
    }
  
}