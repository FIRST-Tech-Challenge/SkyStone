package org.darbots.darbotsftclib.libcore.integratedfunctions;

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
    private JSONArray m_Array;
    private JSONObject m_CurrentLog;
    private JSONArray m_Logs;
    private String m_FileName;
    private boolean m_DebugOn;
    public RobotLogger(String FileName){
        this.m_FileName = FileName;
        this.m_DebugOn = false;
        this.__setupCurrentLog();
        this.__readLoggerFile();
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
        String currentTimeStr = DateFormat.getInstance().format(currentTime.getTime());
        m_CurrentLog.put("startTime", currentTimeStr);
        m_CurrentLog.put("runningOpMode",GlobalRegister.runningOpMode.getClass().getName());
        m_CurrentLog.put("logs",m_Logs);
    }
    protected void __readLoggerFile(){
        File logFile = FTCFileIO.getSettingFile(m_FileName);
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
    public List getLogs(){
        return m_Logs;
    }
    public void addLog(String module, String caption, String content){
        if(!isDebugOn()){
            return;
        }
        JSONObject logContent = new JSONObject();
        Calendar currentTime = Calendar.getInstance();
        String currentTimeStr = DateFormat.getInstance().format(currentTime.getTime());
        logContent.put("module",module);
        logContent.put("caption",caption);
        logContent.put("timeStamp",currentTimeStr);
        logContent.put("content",content);
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
        File logFile = FTCFileIO.getSettingFile(m_FileName);
        String logFileContent = this.m_Array.toJSONString();
        FTCFileIO.writeFile(logFile,logFileContent);
    }
}
