package org.firstinspires.ftc.teamcode.worlds;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.Locale;

public class DataLogger {
    static private final boolean ENABLE_LOGGING = true;
    static private final boolean ENABLE_MULTIPLE_LOGS = true;
    private Writer writer;
    private StringBuffer lineBuffer;
    private long msBase;
    private long nsBase;

    public DataLogger(String dirName, String fileName, Telemetry telemetry) {


        if (!ENABLE_LOGGING) {
            return;
        }

        telemetry.addData("00:DataLogger", "Invoked...");

        String state = Environment.getExternalStorageState();

/*        if (Environment.MEDIA_MOUNTED.equals(state)) {
                    telemetry.addData("01:DataLogger","ExternalStorage is writeable");
        } else {
                    telemetry.addData("01:DataLogger","ExternalStorage is NOT writeable");
        }
*/

        if (ENABLE_MULTIPLE_LOGS) {
            Date now = Calendar.getInstance().getTime();
            // (2) create a date "formatter" (the date and time format we want)
            SimpleDateFormat formatter = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss", Locale.US);
            fileName = formatter.format(now) + "-" + fileName;
        }

        File logdir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS), dirName);
        //       telemetry.addData("02:DataLogger", Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS) + "/" + dirName);
        File logfile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS), dirName + "/" + fileName);
        //       telemetry.addData("03:DataLogger", Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS) + dirName + "/" + fileName + ".csv");

/*
        if (!logdir.mkdirs()) {
                   telemetry.addData("05:DataLogger", "Directory not created");
        } else {
                   telemetry.addData("05:DataLogger", "Directory is created");
        }
*/

        // Make sure that the directory exists

        try {
            writer = new FileWriter(logfile);
            lineBuffer = new StringBuffer(128);
        } catch (IOException e) {
        }

        msBase = System.currentTimeMillis();
        nsBase = System.nanoTime();
        addField("sec");
        addField("d ms");
    }

    private void flushLineBuffer() {
        long milliTime, nanoTime;


        if (!ENABLE_LOGGING) {
            return;
        }

        try {
            lineBuffer.append('\n');
            writer.write(lineBuffer.toString());
            writer.flush();
            lineBuffer.setLength(0);
        } catch (IOException e) {
        }
        milliTime = System.currentTimeMillis();
        nanoTime = System.nanoTime();
        addField(String.format(Locale.US, "%.3f", (milliTime - msBase) / 1.0E3));
        addField(String.format(Locale.US, "%.3f", (nanoTime - nsBase) / 1.0E6));
        nsBase = nanoTime;
    }

    public void closeDataLogger() {

        if (!ENABLE_LOGGING) {
            return;
        }

        try {
            writer.close();
        } catch (IOException e) {
        }
    }

    public void addField(String s) {

        if (!ENABLE_LOGGING) {
            return;
        }

        if (lineBuffer.length() > 0) {
            lineBuffer.append(',');
        }
        lineBuffer.append(s);
    }

    public void addField(char c) {

        if (!ENABLE_LOGGING) {
            return;
        }

        if (lineBuffer.length() > 0) {
            lineBuffer.append(',');
        }
        lineBuffer.append(c);
    }

    public void addField(boolean b) {
        addField(b ? '1' : '0');
    }

    public void addField(byte b) {
        addField(Byte.toString(b));
    }

    public void addField(short s) {
        addField(Short.toString(s));
    }

    public void addField(long l) {
        addField(Long.toString(l));
    }

    public void addField(float f) {
        addField(Float.toString(f));
    }

    public void addField(double d) {
        addField(Double.toString(d));
    }

    public void newLine() {
        flushLineBuffer();
    }

    @Override
    protected void finalize() throws Throwable {
        closeDataLogger();
        super.finalize();
    }
}
