package org.firstinspires.ftc.robotcontroller.moeglobal.opmodeloading;//package org.firstinspires.ftc.robotcontroller.moeglobal.opmodeloading;
//
//import android.app.Activity;
//import android.app.AlertDialog;
//import android.content.Context;
//import android.content.DialogInterface;
//import android.content.Intent;
//import android.net.Uri;
//import android.os.Environment;
//import android.support.v4.content.ContextCompat;
//import android.util.Pair;
//import com.qualcomm.robotcore.eventloop.EventLoopManager;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
//import dalvik.system.DexClassLoader;
//import dalvik.system.DexFile;
//import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
//import org.firstinspires.ftc.robotcontroller.moeglobal.Constants;
//import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
//import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMetaAndClass;
//import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMetaAndInstance;
//import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
//
//import java.io.File;
//import java.io.IOException;
//import java.lang.reflect.Field;
//import java.lang.reflect.InvocationTargetException;
//import java.util.*;
//
//import static org.firstinspires.ftc.robotcontroller.moeglobal.MOEValueListener.weakActivity;
//import static org.firstinspires.ftc.robotcontroller.moeglobal.MOEGlobalProcesses.storageRef;
//
//public class OldOpModeLoading {
//
//
//    private static final List<String> DefaultOpModeAsList = Collections.singletonList(OpModeManager.DEFAULT_OP_MODE_NAME);
//
//
//
//
//
//    public static void flushOpModes(String path) {
//        File path1 = new File(Environment.getExternalStorageDirectory() + "/firecode/" + path);
//        ArrayList<Pair<OpModeMeta, OpMode>> opModesFromFile = getOpModesFromFile(path1);
//        try {
//            registerOpModes(opModesFromFile);
//        } catch (NoSuchFieldException e) {
//            e.printStackTrace();
//        } catch (IllegalAccessException e) {
//            e.printStackTrace();
//        }
////        Log.e("length", String.valueOf(opModesFromFile.size()));
//    }
//
//    private static ArrayList<Pair<OpModeMeta, OpMode>> getOpModesFromFile(File dexAsFile) {
//        ArrayList<Pair<OpModeMeta, OpMode>> opModeMetas = new ArrayList<>();
//
//        try {
//            if (!dexAsFile.exists()) {
//                return opModeMetas;
//            }
//
//            @SuppressWarnings("deprecation")
//            DexFile dexFile = new DexFile(dexAsFile);
//            Enumeration<String> entries = dexFile.entries();
//
//            List<String> list = Collections.list(entries);
//
//            FtcRobotControllerActivity activity = weakActivity.get();
//            DexClassLoader classLoader = new DexClassLoader(dexAsFile.getAbsolutePath(), ContextCompat.getCodeCacheDir(activity).getAbsolutePath(), null, activity.getClassLoader());
//            if (!list.contains(Constants.masterClass)) {
//                throw new IllegalStateException("Master Class is missing: " + Constants.masterClass);
//            }
//            Class<?> masterClass = classLoader.loadClass(Constants.masterClass);
//            Field classes = masterClass.getField("classes");
//            Class opModeMeta = classLoader.loadClass(Constants.opModeMeta);
//            Field name = opModeMeta.getField("name");
//            Field group = opModeMeta.getField("group");
//            Field isAuton = opModeMeta.getField("isAuton");
//            Field path = opModeMeta.getField("path");
//            Object[] opModes = (Object[]) classes.get(null);
//            opModeMetas.ensureCapacity(opModes.length);
//            for (Object opModeSingle : opModes) {
//                String opModeName = (String) name.get(opModeSingle);
//                String opModeGroup = (String) group.get(opModeSingle);
//                String opModePath = (String) path.get(opModeSingle);
//                boolean opModeIsAuton = isAuton.getBoolean(opModeSingle);
//                OpModeMeta meta = new OpModeMeta(opModeName, opModeIsAuton ? OpModeMeta.Flavor.AUTONOMOUS : OpModeMeta.Flavor.TELEOP, opModeGroup);
//                OpMode actualOpMode = (OpMode) classLoader.loadClass(opModePath).newInstance();
//                opModeMetas.add(new Pair<>(meta, actualOpMode));
//            }
//        } catch (IOException | ClassNotFoundException | NoSuchFieldException | IllegalAccessException | InstantiationException e) {
//            e.printStackTrace();
//        }
//        return opModeMetas;
//    }
//
//    public enum CodeStatus {Internal, External}
//
//    public static CodeStatus currentStatus = CodeStatus.External;
//    public static File dexFilePath = new File(Environment.getExternalStorageDirectory().getAbsolutePath()
//            + "/FIRST/externalDex/classes.dex");
//
//    public static void refresh(Activity activity, EventLoopManager eventLoopManager) {
//        if (currentStatus == CodeStatus.Internal) {
//            internalCode(activity);
//
//        }
//    }
//
//    private static void internalCode(Activity context) {
//        showAlert(context);
//    }
//
////    public static void externalCode(Activity activity) {
////        ArrayList<Pair<OpModeMeta, OpMode>> opModesInMaster = getAllOpModesFromMaster(activity);
////        registerOpModes(opModesInMaster);
////    }
//
//    private static void registerOpModes(ArrayList<Pair<OpModeMeta, OpMode>> opModesInMaster) throws NoSuchFieldException, IllegalAccessException {
//        RegisteredOpModes instance = RegisteredOpModes.getInstance();
//        Field opModeClasses1 = instance.getClass().getDeclaredField("opModeClasses");
//        Field opModeInstances1 = instance.getClass().getDeclaredField("opModeInstances");
//        Field opModesLock1 = instance.getClass().getDeclaredField("opModesLock");
//        opModesLock1.setAccessible(true);
//        opModeClasses1.setAccessible(true);
//        opModeInstances1.setAccessible(true);
//        LinkedHashMap<String, OpModeMetaAndClass> opModeClasses = (LinkedHashMap<String, OpModeMetaAndClass>) opModeClasses1.get(instance);
//        LinkedHashMap<String, OpModeMetaAndInstance> opModesInstances = (LinkedHashMap<String, OpModeMetaAndInstance>) opModeInstances1.get(instance);
//        Object opModesLock = opModesLock1.get(instance);
//
//        synchronized (opModesLock) {
//            opModeClasses.keySet().retainAll(DefaultOpModeAsList);
//            opModesInstances.keySet().retainAll(DefaultOpModeAsList);
//
//            for (Pair<OpModeMeta, OpMode> opModeMetaOpModePair : opModesInMaster) {
//                opModeClasses.put(opModeMetaOpModePair.first.name, new OpModeMetaAndClass(opModeMetaOpModePair.first, (Class<OpMode>) opModeMetaOpModePair.second.getClass()));
//
//            }
//        }
//        weakActivity.get().eventLoop.refreshUI();
//
////        return;
//
//    }
//
//    private static String findDefault(LinkedHashMap<String, OpModeMetaAndClass> opModes) {
//        for (Map.Entry<String, OpModeMetaAndClass> entry : opModes.entrySet()) {
//            if (entry.getValue().meta.name.equals(OpModeManager.DEFAULT_OP_MODE_NAME)) {
//                return entry.getKey();
//            }
////            System.out.println(entry.getKey() + "/" + entry.getValue());
//        }
//        throw new IllegalStateException("why no default");
//    }
////        for(
////    Pair<OpModeMeta, OpMode> opModeMeta :opModesInMaster)
////
////    {
////        instance.register(opModeMeta.first, opModeMeta.second);
////    }
////        Firebase.registerOpModes(opModesInMaster);
//
//
////    private static ArrayList<Pair<OpModeMeta, OpMode>> getAllOpModesFromMaster(Activity activity) {
////        ArrayList<Pair<OpModeMeta, OpMode>> opModeMetas = new ArrayList<>();
////
////        try {
////            if (!dexFilePath.exists()) {
////                return opModeMetas;
////            }
////
////            @SuppressWarnings("deprecation")
////            DexFile dexFile = new DexFile(dexFilePath);
////            Enumeration<String> entries = dexFile.entries();
////
////            List<String> list = Collections.list(entries);
////
////            DexClassLoader classLoader = new DexClassLoader(dexFilePath.getAbsolutePath(), ContextCompat.getCodeCacheDir(activity).getAbsolutePath(), null, activity.getClassLoader());
////            if (!list.contains(Constants.masterClass)) {
////                throw new IllegalStateException("Master Class is missing: " + Constants.masterClass);
////            }
////            Class<?> masterClass = classLoader.loadClass(Constants.masterClass);
////            Field classes = masterClass.getField("classes");
////            Class opModeMeta = classLoader.loadClass(Constants.opModeMeta);
////            Field name = opModeMeta.getField("name");
////            Field group = opModeMeta.getField("group");
////            Field isAuton = opModeMeta.getField("isAuton");
////            Field path = opModeMeta.getField("path");
////            Object[] opModes = (Object[]) classes.get(null);
////            opModeMetas.ensureCapacity(opModes.length);
////            for (Object opModeSingle : opModes) {
////                String opModeName = (String) name.get(opModeSingle);
////                String opModeGroup = (String) group.get(opModeSingle);
////                String opModePath = (String) path.get(opModeSingle);
////                boolean opModeIsAuton = isAuton.getBoolean(opModeSingle);
////                OpModeMeta meta = new OpModeMeta(opModeName, opModeIsAuton ? OpModeMeta.Flavor.AUTONOMOUS : OpModeMeta.Flavor.TELEOP, opModeGroup);
////                OpMode actualOpMode = (OpMode) classLoader.loadClass(opModePath).newInstance();
////                opModeMetas.add(new Pair<>(meta, actualOpMode));
////            }
////        } catch (IOException | ClassNotFoundException | NoSuchFieldException | IllegalAccessException | InstantiationException e) {
////            e.printStackTrace();
////        }
////        return opModeMetas;
////    }
//
//
////    private boolean addAnnotatedOpMode(Class<OpMode> clazz)
////    {
////        if (clazz.isAnnotationPresent(teleop))
////        {
////            Annotation annotation = clazz.getAnnotation(teleop);
////            String groupName = ((TeleOp) annotation).group();
////            return addOpModeWithGroupName(clazz, OpModeMeta.Flavor.TELEOP, groupName);
////        }
////        else if (clazz.isAnnotationPresent(autonomous))
////        {
////            Annotation annotation = clazz.getAnnotation(autonomous);
////            String groupName = ((Autonomous) annotation).group();
////            return addOpModeWithGroupName(clazz, OpModeMeta.Flavor.AUTONOMOUS, groupName);
////        }
////        else
////            return false;
////    }
//
//
//    public static void verifyStatus(Context context) {
//        try {
//            Class<?> aClass = Class.forName("org.firstinspires.ftc.teamcode.external.pushcode.CodeStatus");
//            aClass.getMethod("status").invoke(null);
//            currentStatus = CodeStatus.Internal;
//        } catch (ClassNotFoundException | IllegalAccessException | NoSuchMethodException | InvocationTargetException ignored) {
//        }
//    }
//
//    private static void showAlert(final Activity activity) {
//        activity.runOnUiThread(new Runnable() {
//            @Override
//            public void run() {
//                new AlertDialog.Builder(activity)
//                        .setTitle("Gradle error")
//                        .setMessage("Gradle flavors got screwed up. Try changing build variant to noTeamCodeDebug")
//                        .setPositiveButton("How?", new DialogInterface.OnClickListener() {
//                            @Override
//                            public void onClick(DialogInterface dialog, int which) {
//                                Intent browserIntent = new Intent(Intent.ACTION_VIEW, Uri.parse("https://developer.android.com/studio/run#changing-variant"));
//                                activity.startActivity(browserIntent);
//                                System.exit(0);
//                            }
//                        })
//                        .setNegativeButton("Kill App", new DialogInterface.OnClickListener() {
//                            public void onClick(DialogInterface dialog, int which) {
//                                System.exit(0);
//                            }
//                        })
//                        .setOnDismissListener(
//                                new DialogInterface.OnDismissListener() {
//                                    @Override
//                                    public void onDismiss(DialogInterface dialog) {
//                                        System.exit(0);
//                                    }
//                                })
//                        .setIcon(android.R.drawable.ic_dialog_alert)
//                        .show();
//            }
//        });
//
//    }
//
//
//}
//
