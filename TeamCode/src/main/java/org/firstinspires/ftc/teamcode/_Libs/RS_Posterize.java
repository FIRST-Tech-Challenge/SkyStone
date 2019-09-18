/*
 * Copyright (C) 2011 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.firstinspires.ftc.teamcode._Libs;

import android.app.Activity;
import android.content.Context;
import android.graphics.BitmapFactory;
import android.graphics.Bitmap;

import android.os.Build;
import android.renderscript.Allocation;
import android.renderscript.Element;
import android.renderscript.RSRuntimeException;
import android.renderscript.RenderScript;
import android.renderscript.Script;

import org.firstinspires.ftc.teamcode.R;


import java.util.HashMap;
import java.util.Map;

/**
 * extend the generated ScriptC_rs565 to get {@link Element#RGB_565(RenderScript)} allocations
 * past the checks in ScriptC_rs565#forEach_myKernel565(Allocation, Allocation)
 */
class ScriptC_PosterizeHue_rs565 extends ScriptC_PosterizeHue {
    private final Element elementRgb565;
    private Map<KernelID, Integer> knownIds = new HashMap<>();
    private Integer slotMyKernel565 = null;

    public ScriptC_PosterizeHue_rs565(RenderScript rs) {
        super(rs);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.JELLY_BEAN) {
            this.elementRgb565 = Element.RGB_565(rs);
        } else {
            this.elementRgb565 = null;
        }
    }

    /**
     * like ScriptC_rs565#forEach_myKernel565(Allocation, Allocation), but accepting allocations of
     * {@link Element#RGB_565(RenderScript)} instead of {@link Element#U16(RenderScript)}
     */
    public void forEach_myKernel565_Element_RGB_565(Allocation ain, Allocation aout) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.JELLY_BEAN) {
            Element inElement = ain.getElement();
            if (!inElement.isCompatible(elementRgb565)) {
                throw new RSRuntimeException("Type mismatch, !");
            }
            if (ain != aout) {
                Element outElement = aout.getElement();
                if (!outElement.isCompatible(elementRgb565)) {
                    throw new RuntimeException();
                }
            }
            if (slotMyKernel565 == null) {
                KernelID kernelID = getKernelID_myKernel565();
                slotMyKernel565 = knownIds.get(kernelID);
            }

            super.forEach(slotMyKernel565, ain, aout, null);
        }
    }

    /** memorize the slots that are private in superclass */
    @Override
    protected KernelID createKernelID(int slot, int sig, Element ein, android.renderscript.Element eout) {
        KernelID kernelID = super.createKernelID(slot, sig, ein, eout);
        knownIds.put(kernelID, slot);
        return kernelID;
    }
}

public class RS_Posterize {
 
    private RenderScript mRS;
    private ScriptC_PosterizeHue_rs565 mScript;

    public void createScript(Context act) {
        mRS = RenderScript.create(act);
        mScript = new ScriptC_PosterizeHue_rs565(mRS);
    }

    public void runScript(Bitmap inBm, Bitmap outBm)
    {
        Allocation inAllocation = Allocation.createFromBitmap(mRS, inBm,
                Allocation.MipmapControl.MIPMAP_NONE,
                Allocation.USAGE_SCRIPT);
        Allocation outAllocation = Allocation.createFromBitmap(mRS, outBm,
                Allocation.MipmapControl.MIPMAP_NONE,
                Allocation.USAGE_SCRIPT);

        mScript.forEach_myKernel565_Element_RGB_565(inAllocation, outAllocation);
        outAllocation.copyTo(outBm);
    }

    public void destroyScript() {
        // destroy the RenderScript context
        mRS.destroy();
    }

}
