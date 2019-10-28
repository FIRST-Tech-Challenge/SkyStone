package org.firstinspires.ftc.teamcode.teamcode.ZeroFiles;

import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;

public class ThreadCallable implements Callable<Integer> {

    private final int threadID;

    ThreadCallable(int threadID) {
        this.threadID = threadID;
    }

    @Override
    public Integer call() throws Exception {
        switch (threadID) {
            case 1: return getThreadA();
            case 2: return getThreadB();
            case 3: return getThreadC();
            default:
                throw new IllegalArgumentException("Unknown ThreadID: " + threadID);
        }
    }

    private int getThreadA() {
        sleep(700);
        return 1;
    }

    private int getThreadB() {
        sleep(500);
        return 2;
    }

    private int getThreadC() {
        sleep(900);
        return 2;
    }

    private void sleep(long MS) {
        try { Thread.sleep(MS); }
        catch (InterruptedException e) {}
    }

    public static void main(String[] args) throws ExecutionException, InterruptedException {
        new ThreadRun().executionParallel();
    }
}
