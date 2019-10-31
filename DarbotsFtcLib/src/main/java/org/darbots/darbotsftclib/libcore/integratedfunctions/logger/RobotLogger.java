package org.darbots.darbotsftclib.libcore.integratedfunctions.logger;

import org.darbots.darbotsftclib.libcore.integratedfunctions.FTCFileIO;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.File;
import java.text.DateFormat;
import java.util.Calendar;
import java.util.List;

public class RobotLogger {
    public enum LogLevel{
        FATAL(5),
        ERROR(4),
        WARNING(3),
        INFO(2),
        DEBUG(1);

        private int value = 0;

        private LogLevel(int value) {    //    必须是private的，否则编译错误
            this.value = value;
        }

        public static LogLevel valueOf(int value) {    //    手写的从int到enum的转换函数
            switch (value) {
                case 1:
                    return DEBUG;
                case 2:
                    return INFO;
                case 3:
                    return WARNING;
                case 4:
                    return ERROR;
                case 5:
                    return FATAL;
                default:
                    return null;
            }
        }

        public int value() {
            return this.value;
        }
    }
    private JSONArray m_Array;
    private JSONObject m_CurrentLog;
    private JSONArray m_Logs;
    private String m_FileName;
    private boolean m_DebugOn;
    private LogLevel m_LowestLogLevel;
    public RobotLogger(String FileName){
        this.m_FileName = FileName;
        this.m_DebugOn = false;
        this.m_LowestLogLevel = LogLevel.INFO;
        this.__setupCurrentLog();
        this.__readLoggerFile();
    }
    public LogLevel getLowestLogLevel(){
        return this.m_LowestLogLevel;
    }
    public void setLowestLogLevel(LogLevel lowestLevel){
        this.m_LowestLogLevel = lowestLevel;
    }
    public boolean isDebugOn(){
        return this.m_DebugOn;
    }
    public void setDebugOn(boolean onDebug){
        this.m_DebugOn = onDebug;
    }
    protected void __setupCurrentLog(){
        m_CurrentLog = new JSONObject();
        m_Logs = new JSONArray();
        Calendar currentTime = Calendar.getInstance();
        long currentTimeLong = currentTime.getTime().getTime();
        m_CurrentLog.put("startTime", currentTimeLong);
        m_CurrentLog.put("runningOpMode",GlobalRegister.runningOpMode.getClass().getName());
        m_CurrentLog.put("logs",m_Logs);
    }
    protected void __readLoggerFile(){
        File logFile = FTCFileIO.getLogFolderFile(this.m_FileName);
        String logFileContent = FTCFileIO.readFile(logFile);
        if(logFileContent == null || logFileContent.isEmpty()){
            m_Array = new JSONArray();
        }else {
            JSONParser Parser = new JSONParser();
            try {
                Object ParsedContent = Parser.parse(logFileContent);
                if(ParsedContent instanceof JSONArray) {
                    m_Array = (JSONArray) ParsedContent;
                }else{
                    m_Array = new JSONArray();
                }
            } catch (ParseException e) {
                m_Array = new JSONArray();
            } catch (Exception e) {
                m_Array = new JSONArray();
            }
        }
        __checkCurrentLog();
    }
    protected void __checkCurrentLog(){
        if(!m_Array.contains(m_CurrentLog)){
            m_Array.add(m_CurrentLog);
        }
    }
    public String getFileName(){
        return m_FileName;
    }
    public void setFileName(String fileName){
        if(!m_FileName.equals(fileName)){
            this.m_FileName = fileName;
            __readLoggerFile();
        }
    }
    public List getAllLogs(){
        return this.m_Array;
    }
    public List getCurrentLogs(){
        return m_Logs;
    }
    public void addLog(String module, String caption, Object content, LogLevel logLevel){
        if(!isDebugOn()){
            return;
        }else if(logLevel.value() < this.m_LowestLogLevel.value()){
            return;
        }
        JSONObject logContent = new JSONObject();
        Calendar currentTime = Calendar.getInstance();
        long currentTimeLong =  currentTime.getTime().getTime();
        logContent.put("module",module);
        logContent.put("caption",caption);
        logContent.put("timeStamp",currentTimeLong);
        logContent.put("content",content);
        logContent.put("level",logLevel.value());
        m_Logs.add(logContent);
    }
    public void clearPreviousRunLogs(){
        m_Array.clear();
        __checkCurrentLog();
    }
    public void clearPreviousLogs(){
        m_Logs.clear();
    }
    public void saveLogsToFile(){
        if(!isDebugOn()){
            return;
        }
        File logFile = FTCFileIO.getLogFolderFile(m_FileName);
        String logFileContent = this.m_Array.toJSONString();
        FTCFileIO.writeFile(logFile,logFileContent);
    }
    public static void deleteAllLogs(){
        File[] logFiles = FTCFileIO.getLogFolder().listFiles();
        for(int i=0;i<logFiles.length;i++){
            if(logFiles[i].isFile())
                logFiles[i].delete();
        }
    }
}
