package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Balance{
    public AHRS navx;

    public static Timer timer;
    public static double prevPitchAngle;
    public static double yAxis;
    public static double prevTime = -0.1;
    public static double time = 0;
    public Boolean startBalance;

    public double pitchAngle;
    public double angleRate;
    public double power;

    

    public final double ANG_SLOWBAND = 10; //10
    public final double ANG_GAIN = 0.01;
    public final double RATE_GAIN = 0.01;
    public final double DRV_GAIN = 1;
    public final double MAX_POWER = 0.175;
    public final double TIME_DELTA = 0.1;

    public double Clamp(double min, double in, double max){
        if(in > max){
            return max;
        }
        else if(in < min){
            return min;
        }
        else{
            return in;
        }
    }

    public Balance(){
        navx = new AHRS(Port.kMXP, (byte) 200);
        navx.reset();
        navx.calibrate();

        startBalance = false;
        timer = new Timer();

        final Thread balanceThread = new Thread(){
            public void run(){
                timer.reset();
                timer.start();

                while(true){
                    pitchAngle = navx.getRoll();
                    
                    if(startBalance){
                        time = timer.get();
                        angleRate = (pitchAngle - prevPitchAngle)/(time - prevTime);
                        
                        //System.out.println(Clamp(-MIN_MAX_POWER, MAX_POWER - time * TIME_GAIN, MIN_MAX_POWER));
                        
                        power = Clamp(-MAX_POWER, -pitchAngle * ANG_GAIN - angleRate * RATE_GAIN, MAX_POWER);
                        if(Math.abs(pitchAngle) < ANG_SLOWBAND){
                            power = power/2;
                        }

                        Robot.driveTrain.LtDrvSet(ControlMode.PercentOutput, power);
                        Robot.driveTrain.RtDrvSet(ControlMode.PercentOutput, power);
                        
                        SmartDashboard.putNumber("Pitch", pitchAngle);
                        SmartDashboard.putNumber("Pitch Rate", angleRate);
                        SmartDashboard.putNumber("Power", power);

                        prevPitchAngle = pitchAngle;
                        prevTime = time;
                    }
                    Timer.delay(TIME_DELTA);
                }
            }
        };
        balanceThread.start();
    }

    public void StartBalancing(){
        startBalance = true;

        timer.reset();
        timer.start();
    }

    public void StopBalancing(){
        startBalance = false;

        Robot.driveTrain.LtDrvSet(ControlMode.PercentOutput, 0);
        Robot.driveTrain.RtDrvSet(ControlMode.PercentOutput, 0);
    }
}