package org.firstinspires.ftc.teamcode.roboticslibrary;

import android.util.Log;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Created by FIXIT on 16-10-02.
 */
public abstract class TaskHandler {

    private static String TAG = "TaskHandler";

    private static ExecutorService exec;

    private static HashMap<String, Future> futures = new HashMap<>();
    private static HashMap<String, Boolean> pauseLocks = new HashMap<>();

    public static void init() {
        exec = Executors.newCachedThreadPool();
    }//init

    /*
    ADD TASKS
     */

    //Note: tasks are named in this style: "{Name of Class Adding Task}.{TASK NAME IN UPPER CASE}"
    //e.g. Fermion.VEERCHECK or Fermion.WALLFOLLOW
    public static boolean addTask(String name, Runnable task) {
        if (!containsTask(name)) {
            futures.put(name, exec.submit(task));

            return true;
        }//if

        Log.e(TAG, "Attempted to add pre-existing task!");
        return false;
    }//addTask

    public static boolean addLoopedTask(String name, Runnable task) {

        if (!containsTask(name)) {
            pauseLocks.put(name, false);
        }//if

        return addTask(name, loop(name, task, 0));
    }//addLoopedTask

    public static boolean addLoopedTask(String name, Runnable task, int delay) {
        if (!containsTask(name)) {
            pauseLocks.put(name, false);
        }//if

        return addTask(name, loop(name, task, delay));
    }//addLoopedTask

    public static boolean addCountedTask(String name, Runnable task, int count){

        if (!containsTask(name)) {
            pauseLocks.put(name, false);
        }//if

        return addTask(name, count(name, task, count));
    }//addCountedTask

    public static boolean addDelayedTask(String name, final Runnable task, final int delay) {
        Runnable delayed = new Runnable() {
            @Override
            public void run() {
                try {
                    Thread.sleep(delay);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }//catch

                task.run();
            }//run
        };//delayed

        return addTask(name, delayed);
    }//addDelayedTask

    /*
    PAUSE & RESUME TASKS
    Note: at the moment, this only works for looped and counted tasks
     */
    public static boolean pauseTask(String name) {
        if (containsTask(name)) {
            pauseLocks.put(name, true);

            return true;
        }//if

        return false;
    }//pauseTask

    public static boolean resumeTask(String name) {
        if (containsTask(name)) {
            pauseLocks.put(name, false);

            return true;
        }//if

        return false;
    }//resumeTask

    /*
    SEARCH TASKS
     */

    public static boolean containsTask(String key){
        return futures.containsKey(key);
    }

    public static boolean containsTaskStartingWith(String prefix){
        for (Map.Entry<String, Future> entry : futures.entrySet()) {
            if (entry.getKey().startsWith(prefix)) {
                return true;
            }//if
        }//for

        return false;
    }//containsTaskStartingWith

    /*
    REMOVE TASKS
     */

    public static boolean removeTask(String name) {
        if (futures.containsKey(name)) {
            futures.get(name).cancel(true);
            futures.remove(name);
        }//if

        pauseLocks.remove(name);

        return futures.containsKey(name);
    }//removeTask

    public static void removeAllTasksWith(String prefix) {

        for (Map.Entry<String, Future> entry : futures.entrySet()) {
            if (entry.getKey().startsWith(prefix)) {
                entry.getValue().cancel(true);
            }//if
        }//for

    }//removeAllTasksWith

    public static void removeAllTasks() {
        Log.i(TAG, "removeAllTasks: ");
        for (Map.Entry<String, Future> future : futures.entrySet()) {
            future.getValue().cancel(true);
        }//for

        futures.clear();
        pauseLocks.clear();
    }//removeAllTasks

    /*
    RUNNABLE MODIFIERS
     */

    private static Runnable loop (final String pauseLockName, final Runnable r, final int delay) {

        return new Runnable() {
            @Override
            public void run() {
                while (true) {

                    while (pauseLocks.get(pauseLockName)) {
                        try {
                            Thread.sleep(1);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }//catch
                    }//while

                    r.run();

                    if (delay > 0) {
                        try {
                            Thread.sleep(delay);
                        } catch (InterruptedException e) {
                            break;
                        }//catch
                    }//if

                    if (Thread.currentThread().isInterrupted()) {
                        break;
                    }//if
                }//while
            }//run
        };

    }//loop

    private static Runnable count(final String pauseLockName, final Runnable r, final int count){
        return new Runnable() {
            @Override
            public void run() {
                for(int i = 0; i < count; i++){

                    while (pauseLocks.get(pauseLockName)) {
                        try {
                            Thread.sleep(1);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }//catch
                    }//while

                    r.run();
                }//for
            }
        };
    }//count

}
