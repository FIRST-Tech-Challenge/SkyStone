/*
 Copyright (c) 2019 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.corelib.metrics.sources;

import com.hfrobots.tnt.corelib.metrics.GaugeMetricSource;

import org.openftc.revextensions2.ExpansionHubEx;

import lombok.EqualsAndHashCode;

@EqualsAndHashCode
public class DcMotorCurrentMetricSource implements GaugeMetricSource {
    private final ExpansionHubEx expansionHubEx;

    private final int motorPort;

    private final String name;

    public DcMotorCurrentMetricSource(ExpansionHubEx expansionHubEx, int motorPort) {
        this.expansionHubEx = expansionHubEx;
        this.motorPort = motorPort;
        int moduleAddress = expansionHubEx.getStandardModule().getModuleAddress();

        name = String.format("hub_%d_dcm_curr_%d", moduleAddress, motorPort);

    }

    @Override
    public String getSampleName() {
        return name;
    }

    @Override
    public double getValue() {
        return expansionHubEx.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS, motorPort);
    }
}
