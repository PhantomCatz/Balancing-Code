package frc.DataLogger;

public class CatzLog 
{
    public double robotTime;
    public double robotData1;
    public double robotData2;
    public double robotData3;

    public CatzLog(double time, double data1, double data2, double data3)
    {
        robotTime  = time;
        robotData1 = data1;
        robotData2 = data2;
        robotData3 = data3;
    }

    public String toString()
    {
        return robotTime +", " + robotData1 + ", " + robotData2 + ", " + robotData3;
    }
}