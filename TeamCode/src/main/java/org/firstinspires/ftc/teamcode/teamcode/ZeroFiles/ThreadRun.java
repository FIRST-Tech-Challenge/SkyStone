package org.firstinspires.ftc.teamcode.teamcode.ZeroFiles;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

public class ThreadRun {

    private final ExecutorService threadPool = Executors.newFixedThreadPool(3);
    private final int deca = 10;

    public void executionParallel() throws ExecutionException, InterruptedException {
        int report = 0;
        long timeMillis = System.currentTimeMillis();

        final ThreadCallable threadPrim = new ThreadCallable(1);
        final ThreadCallable threadSeco = new ThreadCallable(3);
        final ThreadCallable threadTert = new ThreadCallable(3);

        for (int i = 0; i < deca; i++) {

            Future<Integer> futurePrim = threadPool.submit(threadPrim);
            Future<Integer> futureSeco = threadPool.submit(threadSeco);
            Future<Integer> futureTert = threadPool.submit(threadTert);

            int v1 = futurePrim.get();
            int v2 = futureSeco.get();
            int v3 = futureTert.get();

            report += v1 + v2 + v3;
        }

        System.out.println("Report : " + report + "   Execution Time : " +
                (System.currentTimeMillis() - timeMillis) + " ms");
    }
}