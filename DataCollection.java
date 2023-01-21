package frc.DataLogger;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.String;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

public class DataCollection 
{	
    Date date = Calendar.getInstance().getTime();	
    SimpleDateFormat sdf = new SimpleDateFormat("_yyyyMMdd_kkmmss");	
    String dateFormatted = sdf.format(date);

    public boolean fileNotAppended = false;

    //decide the location and extender
    public final String logDataFilePath = "//media//sda1//RobotData";
    public final String logDataFileType = ".csv";

    private       Thread  dataThread;

    public        boolean logDataValues = false;   
    public static int     logDataID;

    public ArrayList<CatzLog> logData;

    StringBuilder sb = new StringBuilder();

    private final double LOG_SAMPLE_RATE = 0.1;
    
    public void dataCollectionInit(final ArrayList<CatzLog> list)
    {   
        date = Calendar.getInstance().getTime();
        sdf = new SimpleDateFormat("_yyyyMMdd_kkmmss");	
        dateFormatted = sdf.format(date);

        logData = list;

        dataThread = new Thread( () ->
        {
            while(!Thread.interrupted())
            {   
                if(logDataValues == true)
                {
                    collectData();
                } 
                Timer.delay(LOG_SAMPLE_RATE);
            }
        } );

        dataThread.start();
    }

    public void startDataCollection() 
    {
        logDataValues = true;
    }

    public void stopDataCollection() 
    {
        logDataValues = false; 
    }

    public void collectData()
    {
        logData.add(new CatzLog(Robot.timer.get(), Robot.balance.pitchAngle, Robot.balance.angleRate, 100.0 * Robot.balance.power));
    }
    
    public void writeHeader(PrintWriter pw) 
    {
        pw.print("time, angle, rate, power (x100)");
    }
    
    //create log file
    public String createFilePath()
    {
	    String logDataFullFilePath = logDataFilePath + dateFormatted + logDataFileType;
    	return logDataFullFilePath;
    }

    // print out data after fully updated
    public void exportData(ArrayList<CatzLog> data) throws IOException
    {   
        System.out.println("Export Data ///////////////");    
        try (
            
        FileWriter     fw = new FileWriter(createFilePath(), fileNotAppended);
        BufferedWriter bw = new BufferedWriter(fw);
        PrintWriter    pw = new PrintWriter(bw))

        {
            writeHeader(pw);
            pw.print("\n");

            // loop through arraylist and adds it to the StringBuilder
            int dataSize = data.size();
            for (int i = 0; i < dataSize; i++)
            {
                pw.print(data.get(i).toString() + "\n");
                pw.flush();
            }

            pw.close();
        }
    }

    public static int getLogDataID()
    {
        return logDataID;
    }
}