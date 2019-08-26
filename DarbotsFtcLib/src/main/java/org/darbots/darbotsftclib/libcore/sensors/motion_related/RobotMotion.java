/*
MIT License

Copyright (c) 2018 DarBots Collaborators

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

package org.darbots.darbotsftclib.libcore.sensors.motion_related;


import android.support.annotation.NonNull;

import org.darbots.darbotsftclib.libcore.internal.MotionTaskCallBackToMotorTaskCallBack;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.tasks.motor_tasks.RobotFixCountSpeedCtlTask;
import org.darbots.darbotsftclib.libcore.tasks.motor_tasks.RobotFixCountTask;
import org.darbots.darbotsftclib.libcore.templates.motion_related.RobotMotionTask;
import org.darbots.darbotsftclib.libcore.templates.motion_related.RobotMotionTaskCallBack;

public class RobotMotion {
    protected RobotMotorController m_MotorController;
    protected RobotWheel m_Wheel;
    public class FixedDistanceTask extends RobotFixCountTask implements RobotMotionTask {
        double m_Distance;
        public FixedDistanceTask(double Distance, double Speed, RobotMotionTaskCallBack TaskCallBack) {
            super(0, Speed, TaskCallBack == null ? null : new MotionTaskCallBackToMotorTaskCallBack(RobotMotion.this,TaskCallBack));
            super.setMotorController(RobotMotion.this.getMotorController());
            this.setDistance(Distance);
        }
        public FixedDistanceTask(FixedDistanceTask Task){
            super(Task);
            super.setMotorController(RobotMotion.this.getMotorController());
            this.m_Distance = Task.m_Distance;
        }
        public double getDistance(){
            return this.m_Distance;
        }
        public void setDistance(double Distance){
            this.m_Distance = Distance;
            double Revolution = Distance / RobotMotion.this.getRobotWheel().getCircumference();
            int Counts = (int) Math.round(Revolution * RobotMotion.this.getMotorController().getMotor().getMotorType().getCountsPerRev());
            super.setCounts(Counts);
        }

        @Override
        public RobotMotionTaskCallBack getMotionTaskCallBack(){
            if(super.getTaskCallBack() != null) {
                return ((MotionTaskCallBackToMotorTaskCallBack) super.getTaskCallBack()).getTaskCallBack();
            }else{
                return null;
            }
        }

        @Override
        public void setMotionTaskCallBack(RobotMotionTaskCallBack TaskCallBack) {
            if(TaskCallBack == null){
                super.setTaskCallBack(null);
            }else{
                if(super.getTaskCallBack() != null){
                    ((MotionTaskCallBackToMotorTaskCallBack) super.getTaskCallBack()).setTaskCallBack(TaskCallBack);
                }else{
                    super.setTaskCallBack(new MotionTaskCallBackToMotorTaskCallBack(RobotMotion.this,TaskCallBack));
                }
            }
        }
        @Override
        public String getTaskDetailString() {
            String result="TaskType: RobotFixedDistanceTask, ";
            result += "Distance: " + this.getDistance() + ", ";
            result += "Count: " + this.getCounts() + ", ";
            result += "Speed: " + this.getSpeed();
            return result;
        }
    }
    public class FixedDistanceSpeedCtlTask extends RobotFixCountSpeedCtlTask implements RobotMotionTask{
        double m_Distance;
        public FixedDistanceSpeedCtlTask(double Distance, double Speed, RobotMotionTaskCallBack TaskCallBack, boolean isCountCtl) {
            super(0, Speed, TaskCallBack == null ? null : new MotionTaskCallBackToMotorTaskCallBack(RobotMotion.this,TaskCallBack),isCountCtl);
            super.setMotorController(RobotMotion.this.getMotorController());
            this.setDistance(Distance);
        }
        public FixedDistanceSpeedCtlTask(FixedDistanceSpeedCtlTask Task){
            super(Task);
            super.setMotorController(RobotMotion.this.getMotorController());
            this.m_Distance = Task.m_Distance;
        }
        public double getDistance(){
            return this.m_Distance;
        }
        public void setDistance(double Distance){
            this.m_Distance = Distance;
            double Revolution = Distance / RobotMotion.this.getRobotWheel().getCircumference();
            int Counts = (int) Math.round(Revolution * RobotMotion.this.getMotorController().getMotor().getMotorType().getCountsPerRev());
            super.setCounts(Counts);
        }

        @Override
        public RobotMotionTaskCallBack getMotionTaskCallBack(){
            if(super.getTaskCallBack() != null) {
                return ((MotionTaskCallBackToMotorTaskCallBack) super.getTaskCallBack()).getTaskCallBack();
            }else{
                return null;
            }
        }

        @Override
        public void setMotionTaskCallBack(RobotMotionTaskCallBack TaskCallBack) {
            if(TaskCallBack == null){
                super.setTaskCallBack(null);
            }else{
                if(super.getTaskCallBack() != null){
                    ((MotionTaskCallBackToMotorTaskCallBack) super.getTaskCallBack()).setTaskCallBack(TaskCallBack);
                }else{
                    super.setTaskCallBack(new MotionTaskCallBackToMotorTaskCallBack(RobotMotion.this,TaskCallBack));
                }
            }
        }

        @Override
        public String getTaskDetailString() {
            String result="TaskType: RobotFixedDistanceSpeedCtlTask, ";
            result += "TimeInSeconds: " + this.getTimeInSeconds() + ", ";
            result += "Speed: " + this.getSpeed() + ", ";
            result += "Count: " + this.getCounts() + ", ";
            result += "Distance: " + this.getDistance() + ", ";
            result += "CountControl: " + (this.isCountCtl() ? "Enabled" : "Disabled");
            return result;
        }
    }
    public RobotMotion(@NonNull RobotMotorController MotorController, @NonNull RobotWheel RobotWheel){
        this.m_MotorController = MotorController;
        this.m_Wheel = RobotWheel;
    }
    public RobotMotorController getMotorController(){
        return this.m_MotorController;
    }
    public void setMotorController(@NonNull RobotMotorController MotorController){
        this.m_MotorController = MotorController;
    }
    public RobotWheel getRobotWheel(){
        return this.m_Wheel;
    }
    public void setRobotWheel(RobotWheel Wheel){
        this.m_Wheel = Wheel;
    }

}
