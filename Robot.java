package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.Drive;
import frc.Mechanisms.Balance;
import frc.Mechanisms.DriveTrain;
import edu.wpi.first.wpilibj.DataLogManager;

import java.io.IOException;
import java.util.ArrayList;

public class Robot extends TimedRobot {

  public static Drive drive;
  public static Balance balance;
  public static DriveTrain driveTrain;
  public static Timer timer;

  public ArrayList<CatzLog> dataArrayList;
  public DataCollection dataCollection;
  public DoubleLogEntry myDoubleLog;
  
  @Override
  public void robotInit() {
    dataCollection = new DataCollection();
    dataArrayList = new ArrayList<CatzLog>();
    dataCollection.dataCollectionInit(dataArrayList);

    DataLog log = DataLogManager.getLog();
    myDoubleLog = new DoubleLogEntry(log, "/my/double");

    timer = new Timer();

    drive = new Drive();
    balance = new Balance();
    driveTrain = new DriveTrain();

    driveTrain.BrakeMode();
    driveTrain.LowGear();
  }

  @Override
  public void robotPeriodic() {
    DataLogManager.start(); 
  }

  @Override
  public void autonomousInit() {
    dataCollection.startDataCollection();
    timer.reset();
    timer.start();

    drive.DriveStraight(1.3, 1, 2);
    drive.StopDriving();
    balance.StartBalancing();
  }

  @Override
  public void autonomousPeriodic() {
    /*myDoubleLog.append(balance.power);
    myDoubleLog.append(balance.pitchAngle);
    myDoubleLog.append(balance.angleRate);*/
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    driveTrain.CoastMode();
    balance.StopBalancing();
    drive.StopDriving();

    dataCollection.stopDataCollection();
    try {
      dataCollection.exportData(dataCollection.logData);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void disabledPeriodic() {}
}