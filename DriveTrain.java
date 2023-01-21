package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DriveTrain {
    public final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;

    public final double kRtP = 0.05;  
    public final double kLtP = 0.05;
    public final double kI = 0.0; 
    public final double kD = 0.0;    
    public final double kF = 1023.0/20666.0; 

    public WPI_TalonFX FtLtDrv;
    public WPI_TalonFX FtRtDrv;
    public WPI_TalonFX BkLtDrv;
    public WPI_TalonFX BkRtDrv;
    public DoubleSolenoid solenoid;

    public final int FT_LT_DRV_CAN_ID = 1;
    public final int FT_RT_DRV_CAN_ID = 4;
    public final int BK_LT_DRV_CAN_ID = 2;
    public final int BK_RT_DRV_CAN_ID = 3;

    public final int DEPLOY_PCM_PORT  = 4;
    public final int RETRACT_PCM_PORT = 3;

    public final int DRVTRAIN_VELOCITY_PID_IDX = 0;
    public final int PID_TIMEOUT_MS            = 10;

    public DriveTrain(){
        FtLtDrv = new WPI_TalonFX(FT_LT_DRV_CAN_ID);
        FtRtDrv = new WPI_TalonFX(FT_RT_DRV_CAN_ID);
        BkLtDrv = new WPI_TalonFX(BK_LT_DRV_CAN_ID);
        BkRtDrv = new WPI_TalonFX(BK_RT_DRV_CAN_ID);
        
        FtLtDrv.configFactoryDefault();
        FtRtDrv.configFactoryDefault();
        BkLtDrv.configFactoryDefault();
        BkRtDrv.configFactoryDefault();

        FtRtDrv.setInverted(true);
        BkRtDrv.setInverted(true);

        BkLtDrv.follow(FtLtDrv);
        BkRtDrv.follow(FtRtDrv);

        FtLtDrv.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, DRVTRAIN_VELOCITY_PID_IDX, PID_TIMEOUT_MS);
        FtLtDrv.config_kP(DRVTRAIN_VELOCITY_PID_IDX, kLtP);
        FtLtDrv.config_kI(DRVTRAIN_VELOCITY_PID_IDX, kI);
        FtLtDrv.config_kD(DRVTRAIN_VELOCITY_PID_IDX, kD);
        FtLtDrv.config_kF(DRVTRAIN_VELOCITY_PID_IDX, kF);

        FtRtDrv.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, DRVTRAIN_VELOCITY_PID_IDX, PID_TIMEOUT_MS);
        FtRtDrv.config_kP(DRVTRAIN_VELOCITY_PID_IDX, kRtP);
        FtRtDrv.config_kI(DRVTRAIN_VELOCITY_PID_IDX, kI);
        FtRtDrv.config_kD(DRVTRAIN_VELOCITY_PID_IDX, kD);
        FtRtDrv.config_kF(DRVTRAIN_VELOCITY_PID_IDX, kF);

        solenoid = new DoubleSolenoid(PCM_TYPE, RETRACT_PCM_PORT, DEPLOY_PCM_PORT);
    }

    public void CoastMode(){
        FtLtDrv.setNeutralMode(NeutralMode.Coast);
        FtRtDrv.setNeutralMode(NeutralMode.Coast);
        BkLtDrv.setNeutralMode(NeutralMode.Coast);
        BkRtDrv.setNeutralMode(NeutralMode.Coast);
    }

    public void BrakeMode(){
        FtLtDrv.setNeutralMode(NeutralMode.Brake);
        FtRtDrv.setNeutralMode(NeutralMode.Brake);
        BkLtDrv.setNeutralMode(NeutralMode.Brake);
        BkRtDrv.setNeutralMode(NeutralMode.Brake);
    }

    public void RtDrvSet(ControlMode mode, double value){
        FtRtDrv.set(mode, value);
    }

    public void LtDrvSet(ControlMode mode, double value){
        FtLtDrv.set(mode, value);
    }

    public void LowGear(){
        solenoid.set(Value.kReverse);
    }

    public void HighGear(){
        solenoid.set(Value.kForward);
    }
}