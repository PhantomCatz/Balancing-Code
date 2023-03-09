package frc.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;

/***************************************************************************
    *
    * Autonomous selections
    * 
***************************************************************************/
public class CatzAutonomousPaths
{  
    private final SendableChooser<Boolean> chosenAllianceColor = new SendableChooser<>();
    private final SendableChooser<Integer> chosenPath = new SendableChooser<>();
    private final SendableChooser<Boolean> chosenBalance = new SendableChooser<>();
    private final SendableChooser<Boolean> chosenCube = new SendableChooser<>();


    private final double STRAIGHT = 0.0;
    public static double CORRECTED_STRAIGHT = 12.0;
    public static double CORRECTED_RIGHT = -90.0; //RIGHT = negative for reasons
    public static double RIGHT = -90.0; 
    public static double LEFT  = 90.0;

    private final double TEMP_MAX_TIME = 8.0;
    private final double TEMP_DECEL_DIST = 0.1;
    private final double MIN_DIST = 20.0;
    private final double MAX_DIST = 100.0;
    private final double GP_TO_GP = 48.0;

    private final double INDEXER_EJECT_TIME = 0.5;

    public static boolean doBalance = false; // default
    public static boolean score = false;

    public static int Path;
    
    //MAX SPEED
    public static double FAST = 0.35;
    public static double SLOW = 0.25;

    public static String path;

     /*  DRIVE STRAIGHT VALUES: 
     * if distance > 70, then FAST, else SLOW
     * 8 second maxTime is an arbitrary number, subject to change upon more testing 
     * only robot backwards movement has negative signs over distance and maxspeed
     * left and right distances and max speed aren't negative
     * TEMP_DECEL_DIST decelDistance is an arbitrary number, subject to change upon more testing
     * 
     *   *note* - autonomous is 15 seconds, meaning that all of this will have to finsih within that time
     *          - THIS CODE IS MADE FOR BLUE SIDE 
     *          - FOR RED, CHANGE LEFTS WITH RIGHTS AND RIGHTS WITH LEFTS (from blue)
     *          - movement similar to code.org level programming
    */

    /* PATH NAME:
     *    /CenterRightTunnel/
     * - CenterRight (Starting Position)
     * - Tunnel (type of movement/movement path)
     */

    /* Distances:          -______-
     * drive.DriveStraight(distance, decelDistance, maxSpeed, wheelPos, maxTime);
     *  - 224 = distance from grid to center pieces
     *                
     */
    // drive.DriveStraight(distance, decelDist, )
    public CatzAutonomousPaths() {}


    public void initializePathOptions()
    {
        chosenAllianceColor.setDefaultOption("Blue Alliance", Robot.constants.BLUE_ALLIANCE);
        chosenAllianceColor.addOption("Red Alliance", Robot.constants.RED_ALLIANCE);
        SmartDashboard.putData(Robot.constants.ALLIANCE_COLOR, chosenAllianceColor);

        chosenPath.setDefaultOption(Robot.constants.POSITION_SELECTOR1, 1);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR2, 2);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR3, 3);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR4, 4);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR5, 5);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR6, 6);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR7, 7);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR8, 8);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR9, 9);
        SmartDashboard.putData(Robot.constants.ALLIANCE_POSITION, chosenPath);

        chosenBalance.setDefaultOption("Don't Balance", Robot.constants.NO_BALANCE);
        chosenBalance.addOption("Do Balance", Robot.constants.YES_BALANCE);
        SmartDashboard.putData(Robot.constants.BALANCE, chosenBalance);

        chosenCube.setDefaultOption("Regular Version", Robot.constants.REGULAR_PATH);
        chosenCube.addOption("Cube Version", Robot.constants.CUBE_PATH);
        SmartDashboard.putData(Robot.constants.PATH_TYPE, chosenCube);


    }

    public void Red() {
        RIGHT *= -1;
        LEFT *= -1;
    }
    
    public void OppositeDirection() 
    {
        FAST *= -1;
        SLOW *= -1;
        score = true;
    }

    public void CubeIndexerScore()
    {
        Robot.indexer.indexerFrntEject();
        Timer.delay(INDEXER_EJECT_TIME);
        Robot.indexer.indexerFrntStop();
    }

    /**************************************************************************************
     *                                 Drive Foward
     **************************************************************************************/

    public void centerToGrid() 
    {
        Robot.auton.DriveStraight(MAX_DIST, TEMP_DECEL_DIST, FAST, STRAIGHT, TEMP_MAX_TIME);
    }
    public void gridToCenter() 
    {
        Robot.auton.DriveStraight(MAX_DIST, TEMP_DECEL_DIST, FAST, CORRECTED_STRAIGHT, TEMP_MAX_TIME);
    }
    
    public void gridToAreaInfrontOfCargo() 
    {
        Robot.auton.DriveStraight(MAX_DIST - MIN_DIST, TEMP_DECEL_DIST, FAST, STRAIGHT, TEMP_MAX_TIME);
    }
    
    public void fowardToCargo() 
    {
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, SLOW, STRAIGHT, TEMP_MAX_TIME);
    }
    public void gridToChargingStation() 
    {
        Robot.auton.DriveStraight(GP_TO_GP + 24, TEMP_DECEL_DIST, SLOW, STRAIGHT, TEMP_MAX_TIME);
    }


    /**************************************************************************************
     *                                 Drive Backward
     **************************************************************************************/


    public void centerToAreaInfrontOfDock() 
    {
        Robot.auton.DriveStraight(MAX_DIST - MIN_DIST, TEMP_DECEL_DIST, -FAST, STRAIGHT, TEMP_MAX_TIME);
    }
    public void dockToGrid() 
    {
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, -SLOW, STRAIGHT, TEMP_MAX_TIME);
    }
    public void centerToChargingStation() 
    {
        Robot.auton.DriveStraight(MIN_DIST * 13 / 2, TEMP_DECEL_DIST, -FAST, STRAIGHT, TEMP_MAX_TIME); //dist used to be -128.75
    }

    /**************************************************************************************
     *                                  Drive Right
     **************************************************************************************/
    public void translateRight48() 
    {
        Robot.auton.DriveStraight(GP_TO_GP + 24, TEMP_DECEL_DIST, SLOW, RIGHT, TEMP_MAX_TIME);
    }
    public void translateFowardRight() 
    {
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, SLOW, STRAIGHT, TEMP_MAX_TIME);
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, SLOW, RIGHT, TEMP_MAX_TIME);
    }
    public void translateBackRight() //Backwards and Right
    {
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, SLOW, RIGHT, TEMP_MAX_TIME);
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, -SLOW, STRAIGHT, TEMP_MAX_TIME);
    }

    /**************************************************************************************
     *                                 Drive Left
     **************************************************************************************/

    public void translateLeft48() 
    {
        Robot.auton.DriveStraight(GP_TO_GP, TEMP_DECEL_DIST, SLOW, LEFT, TEMP_MAX_TIME);
    }
    public void translateFowardLeft() 
    {
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, SLOW, STRAIGHT, TEMP_MAX_TIME);
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, SLOW, LEFT, TEMP_MAX_TIME);
    }
    public void translateBackLeft() 
    {
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, SLOW, LEFT, TEMP_MAX_TIME);
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, -SLOW, STRAIGHT, TEMP_MAX_TIME);
    }    
    public void diagonal()
    {
        Robot.auton.DriveStraight(GP_TO_GP, TEMP_DECEL_DIST, FAST, GP_TO_GP, TEMP_MAX_TIME); //wheelPos used to be 45 (foward left)
        Robot.auton.DriveStraight(GP_TO_GP, TEMP_DECEL_DIST, SLOW, STRAIGHT, TEMP_MAX_TIME);
    }
   
    private final int CENTER_PRELOAD_TAXI_BALANCE = 1;
    private final int SIDE_PRELOAD_INTAKE_SCORE = 2;
    private final int SIDE_PRELOAD_INTAKE_SCORE_BALANCE = 3;
    private final int DEFENSE_PRELOAD_POSITIONING = 4;
    private final int CENTER_PRELOAD_SCORE_INTAKE_BALANCE = 5;
    private final int CENTER_PRELOAD_INTAKE_SCORE_INTAKE_SCORE_BALANCE = 6;
    private final int SIDE_PRELOAD_INTAKE_SCORE_INTAKE = 7;
    private final int SIDE_2_PRELOAD_INTAKE_SCORE_INTAKE_SCORE_BALANCE = 8;
    private final int TEST_PATH = 9;
    
    public void determinePath()
    {
        Path = chosenPath.getSelected();
        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
        {
            Red();
        }
        if(chosenCube.getSelected() == Robot.constants.CUBE_PATH) 
        {
            OppositeDirection();
        }

        if(chosenBalance.getSelected()){
            doBalance = true;
        }
        else{
            doBalance = false;
        }

        if(Path == CENTER_PRELOAD_TAXI_BALANCE) 
        {
            CenterPreloadTaxiBalance();
        }
        if(Path == SIDE_PRELOAD_INTAKE_SCORE) 
        {
            SidePreloadIntakeScore();
        }
        if(Path == SIDE_PRELOAD_INTAKE_SCORE_BALANCE)
        {
            SidePreloadIntakeScoreBalance();
        }
        if(Path == DEFENSE_PRELOAD_POSITIONING) 
        {
            DefensePreloadPositioning();
        }
        if(Path == CENTER_PRELOAD_SCORE_INTAKE_BALANCE) 
        {
            CenterPreloadIntakeScoreBalance();
        } 
        if(Path == CENTER_PRELOAD_INTAKE_SCORE_INTAKE_SCORE_BALANCE) 
        {
            CenterPreloadIntakeScoreIntakeScoreBalance();
        } //SidePreloadIntakeScoreIntake
        if(Path == SIDE_PRELOAD_INTAKE_SCORE_INTAKE) 
        {
            SidePreloadIntakeScoreIntake();
        } // Side2PreloadIntakeScoreIntakeScoreBalance
        if(Path == SIDE_2_PRELOAD_INTAKE_SCORE_INTAKE_SCORE_BALANCE) 
        {
            Side2PreloadIntakeScoreIntakeScoreBalance();
        }
        if(Path == TEST_PATH) 
        {
            TestPath();
        }
    }

    public void TestPath() 
    {
        if(score) { CubeIndexerScore(); }
        Robot.auton.DriveStraight(30, TEMP_DECEL_DIST, SLOW, CORRECTED_STRAIGHT, TEMP_MAX_TIME);

        Robot.auton.TurnInPlace(180, 8);
        Robot.auton.DriveStraight(150, TEMP_DECEL_DIST, SLOW, CORRECTED_STRAIGHT, TEMP_MAX_TIME);
        Timer.delay(3.0);
        
        Robot.auton.TurnInPlace(180, 8);
        Robot.auton.DriveStraight(165, TEMP_DECEL_DIST, -SLOW, CORRECTED_STRAIGHT, TEMP_MAX_TIME);
        Robot.auton.DriveStraight(48, TEMP_DECEL_DIST, SLOW, CORRECTED_RIGHT, TEMP_MAX_TIME);
        Robot.auton.DriveStraight(25, TEMP_DECEL_DIST, -SLOW, CORRECTED_STRAIGHT, TEMP_MAX_TIME);
        if(score) { CubeIndexerScore(); }

        Robot.auton.StopDriving();
    }

    public void CenterPreloadTaxiBalance() 
    {
        dockToGrid();
        if(score) { CubeIndexerScore(); }
        //score code
        gridToCenter();
        //pickup cone;
        if(doBalance) 
        {
            centerToChargingStation();
            Robot.balance.StartBalancing();
        }
        Robot.auton.StopDriving();
    }
    
    public void SidePreloadIntakeScore()
    {
        dockToGrid();
        if(score) { CubeIndexerScore(); }
        //score code
        gridToCenter();
        //pickup cone;
        centerToGrid();
        if(score) { CubeIndexerScore(); }
        //score cone
        Robot.auton.StopDriving();
    }

    public void SidePreloadIntakeScoreBalance() 
    {
        translateBackLeft();
        if(score) { CubeIndexerScore(); }
        //score cone
        translateFowardRight();
        gridToCenter();
        //pickup cube
        centerToGrid();
        if(score) { CubeIndexerScore(); }
        //score cube
        translateFowardRight();
        translateRight48();
        if(doBalance) 
        {
            gridToChargingStation();
            Robot.balance.StartBalancing();
        }
        Robot.auton.StopDriving();
    }

    public void DefensePreloadPositioning() 
    {
        translateBackLeft();
        if(score) { CubeIndexerScore(); }
        //score cone;
        translateFowardRight();
        gridToAreaInfrontOfCargo();
        diagonal();
        Robot.auton.StopDriving();
    }

    public void CenterPreloadIntakeScoreBalance() 
    {  
        dockToGrid();
        if(score) { CubeIndexerScore(); }
        //score cone;
        gridToCenter();
        //pickup cone;
        centerToAreaInfrontOfDock();
        translateBackRight();
        if(score) { CubeIndexerScore(); }
        //score cone;
        translateFowardLeft();
        if(doBalance) 
        {
            gridToChargingStation();
            Robot.balance.StartBalancing();
        }
        Robot.auton.StopDriving();
    }

    public void CenterPreloadIntakeScoreIntakeScoreBalance() 
    {
        dockToGrid();
        if(score) { CubeIndexerScore(); }
        //score cone;
        gridToCenter();
        //pickup cone;
        centerToAreaInfrontOfDock();
        translateBackRight();
        if(score) { CubeIndexerScore(); }
        //score cone
        translateFowardLeft();
        gridToAreaInfrontOfCargo();
        //pickup cone
        centerToGrid();
        if(score) { CubeIndexerScore(); }
        //score cone
        if(doBalance) 
        {
            gridToChargingStation();
            Robot.balance.StartBalancing();
        }
        Robot.auton.StopDriving();
    }


    public void SidePreloadIntakeScoreIntake()
    {
        translateBackLeft();
        if(score) { CubeIndexerScore(); }
        //score cone;
        translateFowardRight();
        gridToAreaInfrontOfCargo();
        //pickup cube
        centerToGrid();
        if(score) { CubeIndexerScore(); }
        //score cube
        gridToAreaInfrontOfCargo();
        translateRight48();
        fowardToCargo();
        //pickup cone
        centerToGrid();
        if(score) { CubeIndexerScore(); }
        //score cone
        if(doBalance) 
        {
            gridToChargingStation();
            Robot.balance.StartBalancing();
        }
        Robot.auton.StopDriving();
    }


    public void Side2PreloadIntakeScoreIntakeScoreBalance()
    {
        translateBackRight();
        if(score) { CubeIndexerScore(); }
        //score cone;
        translateFowardLeft();
        gridToAreaInfrontOfCargo();
        //pickup cube
        centerToGrid();
        if(score) { CubeIndexerScore(); }
        //score cube
        gridToAreaInfrontOfCargo();
        translateLeft48();
        fowardToCargo();
        //pickup cone
        centerToGrid();
        if(score) { CubeIndexerScore(); }
        //score cone
        if(doBalance) 
        {
            gridToChargingStation();
            Robot.balance.StartBalancing();
        }
        Robot.auton.StopDriving();
    }
}

