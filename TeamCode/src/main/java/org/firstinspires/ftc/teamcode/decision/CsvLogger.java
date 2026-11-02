package org.firstinspires.ftc.teamcode.decision;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

// ---------------------------------------------------------------------
// CSV logger for DecisionModelTeleOp: one file per run. Decimals always use dots.
// Copy the files to the laptop with:  adb pull /sdcard/FIRST/decision_logs logs
// ---------------------------------------------------------------------
public class CsvLogger {
    public static final String HEADER = "t_ms,loop_ms,episode,event,mode,source,state,state_name,"
            + "x,y,heading,vx,vy,omega,"
            + "balls,have_target,tx,ty,area,conf,"
            + "shoot_fwd_err,shoot_left_err,shoot_head_err,has_spot,"
            + "target_rpm,rpm,ready,feeding,firing,intake_amps,stopped,"
            + "act_fwd,act_strafe,act_turn,act_fire,teacher_active,"
            + "model_ok,model_fwd,model_strafe,model_turn,model_fire,"
            + "cmd_fwd,cmd_strafe,cmd_turn,obs_seq";

    private final String dir;
    private File file;
    private BufferedWriter out;
    private int rows = 0;

    public CsvLogger(String dir) { this.dir = dir; }

    public boolean open() {
        try {
            File folder = new File(dir);
            if (!folder.exists() && !folder.mkdirs()) return false;
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            file = new File(folder, "run_" + stamp + ".csv");
            out = new BufferedWriter(new FileWriter(file));
            out.write(HEADER);
            out.newLine();
            return true;
        } catch (IOException e) {
            out = null;
            return false;
        }
    }

    public String path() { return file == null ? "" : file.getAbsolutePath(); }

    public void row(Object[] values) {
        if (out == null) return;
        StringBuilder sb = new StringBuilder(400);
        for (int i = 0; i < values.length; i++) {
            if (i > 0) sb.append(',');
            Object v = values[i];
            if (v instanceof Double) sb.append(String.format(Locale.US, "%.4f", (Double) v));
            else sb.append(v);
        }
        try {
            out.write(sb.toString());
            out.newLine();
            if (++rows % 30 == 0) out.flush(); // about once a second
        } catch (IOException e) {
            out = null; // stop logging rather than crash the OpMode
        }
    }

    public void close() {
        if (out == null) return;
        try { out.close(); } catch (IOException ignored) { }
        out = null;
    }
}
