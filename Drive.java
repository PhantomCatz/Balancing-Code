package frc.Mechanisms;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Robot;

public class Drive{
    public Boolean startDriving;
    public DriveTrain driveTrain;

    public double accelTime;
    public double decelTime;
    public double endTime = -1.0;
    public double time = 0;
    public double speed;

    final double LOW_GEAR_RATIO = 14.0 / 60.0;
    final double HIG_GEAR_RATIO = 24.0 / 50.0;
    final double TIME_DELTA = 0.05;

    public final double DUMMY = 0.0001;

    final double TALONFX_INTEGRATED_ENC_CNTS_PER_REV      = 2048.0;
    final double DRVTRAIN_WHEEL_RADIUS                    = 2.0;
    final double DRVTRAIN_WHEEL_CIRCUMFERENCE             = (2.0 * Math.PI * DRVTRAIN_WHEEL_RADIUS);
    final double INCH_TO_FEET                             = 1.0/12.0;
    final double HUNDRED_MS_TO_S                          = 1.0/10.0;
    public final double DRVTRAIN_ENC_COUNTS_TO_FEET_PER_SECOND_LOW = DUMMY; //(1/LOW_GEAR_RATIO) * DRVTRAIN_WHEEL_CIRCUMFERENCE * (1.0/TALONFX_INTEGRATED_ENC_CNTS_PER_REV) * INCH_TO_FEET / HUNDRED_MS_TO_S;
    public final double DRVTRAIN_ENC_COUNTS_TO_FEET_PER_SECOND_HIG = DUMMY; //(1/HIG_GEAR_RATIO) * DRVTRAIN_WHEEL_CIRCUMFERENCE * (1.0/TALONFX_INTEGRATED_ENC_CNTS_PER_REV) * INCH_TO_FEET / HUNDRED_MS_TO_S;

    public Drive(){
        startDriving = false;    
    }

    public void DriveStraight(double distance, double maxAccel, double maxSpeed){
                startDriving = true;
                Timer timer = new Timer();

                if(distance > maxSpeed * maxSpeed / maxAccel){
                    accelTime = maxSpeed / maxAccel;
                    decelTime = (distance + accelTime) / maxSpeed - 1 / maxAccel;
                    endTime = decelTime + accelTime;
                }
                else{
                    accelTime = Math.sqrt(distance / maxAccel);
                    decelTime = Math.sqrt(distance / maxAccel);
                    endTime = 2 * Math.sqrt(distance / maxAccel);
                }

                timer.reset(); timer.start();
                while(time <= endTime && startDriving){
                    if(time <= accelTime){
                        speed = maxAccel * time / DRVTRAIN_ENC_COUNTS_TO_FEET_PER_SECOND_LOW;
                    }
                    else if(time > accelTime && time < decelTime){
                        speed = maxSpeed / DRVTRAIN_ENC_COUNTS_TO_FEET_PER_SECOND_LOW;
                    }
                    else{
                        speed = maxAccel * (endTime - time) / DRVTRAIN_ENC_COUNTS_TO_FEET_PER_SECOND_LOW;
                    }
                    SmartDashboard.putNumber("Speed", speed);         

                    Robot.driveTrain.LtDrvSet(ControlMode.Velocity, speed);
                    Robot.driveTrain.RtDrvSet(ControlMode.Velocity, speed);

                    Timer.delay(TIME_DELTA);
                    time = timer.get();
                }

                Robot.driveTrain.LtDrvSet(ControlMode.Velocity, 0);
                Robot.driveTrain.RtDrvSet(ControlMode.Velocity, 0);

                startDriving = false;
                endTime = -0.1;
    }

    public void StopDriving(){
        startDriving = false;
    }
}