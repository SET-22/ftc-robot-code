package org.firstinspires.ftc.teamcode.decision;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

// ---------------------------------------------------------------------
// Laptop link: a background thread with one UDP socket.
//   Laptop -> Hub:  "H"  (hello, so the Hub learns the laptop's address)
//                   "A,seq,fwd,strafe,turn,fire"  (seq = the observation it answers)
//   Hub -> laptop:  "O,seq,t_ms,have_target,tx,ty,area,conf,
//                    shoot_fwd_err,shoot_left_err,shoot_head_err,vx,vy,balls,state"
// The OpMode only reads the latest accepted action; old or out-of-order ones are dropped.
// ---------------------------------------------------------------------
public class PolicyLink implements Runnable {

    public static class Action {
        public final long seq;
        public final double fwd, strafe, turn;
        public final boolean fire;
        public final double receivedMs;

        Action(long seq, double fwd, double strafe, double turn, boolean fire, double receivedMs) {
            this.seq = seq; this.fwd = fwd; this.strafe = strafe; this.turn = turn;
            this.fire = fire; this.receivedMs = receivedMs;
        }
    }

    private final int port;
    private DatagramSocket socket;
    private Thread thread;
    private volatile boolean running = false;
    private volatile InetSocketAddress laptop = null;
    private volatile String pendingObs = null;
    private volatile long lastObsSeq = 0;
    private volatile double lastPacketMs = -1e9;
    private Action latest = null;          // guarded by "this"
    private long rejected = 0;

    public PolicyLink(int port) { this.port = port; }

    public boolean start() {
        try {
            socket = new DatagramSocket(port);
            socket.setSoTimeout(5);
        } catch (IOException e) {
            return false;
        }
        running = true;
        thread = new Thread(this, "PolicyLink");
        thread.setDaemon(true);
        thread.start();
        return true;
    }

    public void stop() {
        running = false;
        if (socket != null) socket.close();
    }

    /** Called by the OpMode; the thread sends it (no networking on the loop thread). */
    public void sendObservation(String obs, long seq) {
        lastObsSeq = seq;
        pendingObs = obs;
    }

    /** Latest action if it is fresh and answers a recent observation, else null. */
    public synchronized Action latestAction(double nowMs, double timeoutMs, long minSeq) {
        if (latest == null) return null;
        if (nowMs - latest.receivedMs > timeoutMs) return null;
        if (latest.seq < minSeq) return null;
        return latest;
    }

    public String describe(double nowMs) {
        if (socket == null) return "port not open";
        if (laptop == null) return "waiting for laptop";
        double age = nowMs - lastPacketMs;
        return String.format(Locale.US, "%s (last %.0f ms ago, dropped %d)",
                age < 500 ? "connected" : "QUIET", age, rejected);
    }

    @Override
    public void run() {
        byte[] buf = new byte[256];
        while (running) {
            // ---- Receive (5 ms timeout so sending stays responsive) ----
            try {
                DatagramPacket p = new DatagramPacket(buf, buf.length);
                socket.receive(p);
                handle(new String(p.getData(), 0, p.getLength(), StandardCharsets.US_ASCII).trim(),
                        (InetSocketAddress) p.getSocketAddress());
            } catch (SocketTimeoutException ignored) {
                // nothing arrived
            } catch (IOException e) {
                if (!running) break;
            }

            // ---- Send the newest observation, if any ----
            String obs = pendingObs;
            InetSocketAddress to = laptop;
            if (obs != null && to != null) {
                pendingObs = null;
                try {
                    byte[] data = obs.getBytes(StandardCharsets.US_ASCII);
                    socket.send(new DatagramPacket(data, data.length, to));
                } catch (IOException ignored) {
                    // drop it; the next one comes in 33 ms
                }
            }
        }
    }

    private void handle(String msg, InetSocketAddress from) {
        laptop = from;
        lastPacketMs = System.nanoTime() / 1e6;
        if (!msg.startsWith("A,")) return; // hello or unknown
        String[] f = msg.split(",");
        if (f.length < 6) { rejected++; return; }
        try {
            long seq = Long.parseLong(f[1]);
            double fwd = Double.parseDouble(f[2]);
            double strafe = Double.parseDouble(f[3]);
            double turn = Double.parseDouble(f[4]);
            boolean fire = Integer.parseInt(f[5].trim()) != 0;
            if (Double.isNaN(fwd) || Double.isNaN(strafe) || Double.isNaN(turn)) { rejected++; return; }
            synchronized (this) {
                // Out of order, or answering an observation we never sent
                if ((latest != null && seq <= latest.seq) || seq > lastObsSeq) { rejected++; return; }
                latest = new Action(seq, fwd, strafe, turn, fire, lastPacketMs);
            }
        } catch (NumberFormatException e) {
            rejected++;
        }
    }
}
