/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.SerialNumber;

@SuppressWarnings("WeakerAccess")
public class FtcRobotControllerActivity extends Activity {

  public FtcRobotControllerActivity() {
      mHandler = new Handler();
    }
  public static final String TAG = "RCActivity";
  DeviceInterfaceModule cdim0=null, cdim1=null;

  protected TextView textVoltage0, textVoltage1, textVoltage2, textVoltage3;
  protected TextView textVoltage4, textVoltage5, textVoltage6, textVoltage7;
  TextView[] voltageView;

  @Override
  protected void onNewIntent(Intent intent) {
    super.onNewIntent(intent);
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_ftc_controller);
    setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

    textVoltage0 = findViewById(R.id.textView0);
    textVoltage1 = findViewById(R.id.textView1);
    textVoltage2 = findViewById(R.id.textView2);
    textVoltage3 = findViewById(R.id.textView3);
    textVoltage4 = findViewById(R.id.textView4);
    textVoltage5 = findViewById(R.id.textView5);
    textVoltage6 = findViewById(R.id.textView6);
    textVoltage7 = findViewById(R.id.textView7);

    voltageView = new TextView[] {textVoltage0, textVoltage1, textVoltage2, textVoltage3, textVoltage4, textVoltage5, textVoltage6, textVoltage7};

    for (TextView textView : voltageView) {
      textView.setText("0V");
      textView.setTextColor(Color.BLACK);
      textView.setVisibility(View.VISIBLE);
    }

    cdim0 = connectToDIMusingSerial("AL00VCZJ");
    cdim1 = connectToDIMusingSerial("AL00VEYO");

    cdim0.setLED(0, true);
    cdim1.setLED(1, true);

    startUpdatingCDIM();
  }

  private boolean mActive;
  private final Handler mHandler;

  private void startUpdatingCDIM() {
    mActive = true;
    mHandler.post(updateCDIM);
  }

  private final Runnable updateCDIM = new Runnable() {
    @Override
    public void run() {
      if(mActive) {
        int channel = 0;
          for (TextView view : voltageView) {
            double raw_voltage = cdim0.getAnalogInputVoltage(channel);
            double corrected_voltage = Math.round(voltage_divider(raw_voltage) * 100.) / 100.;

            if (corrected_voltage < 10 || corrected_voltage > 18) {
              view.setTextColor(Color.RED);
            } else if (corrected_voltage > 13.8) {
              view.setTextColor(Color.GREEN);
            } else {
              view.setTextColor(Color.YELLOW);
            }
            String outputText = corrected_voltage + "V";
            view.setText(outputText);
            channel++;
          }
      }

      mHandler.postDelayed(updateCDIM, 5);
    }
  };

  private double voltage_divider(double measured_voltage) {
    return voltage_divider(measured_voltage, 1000000, 100000);
  }

  private double voltage_divider(double measured_voltage, double r_1, double r_2) {
    return ((measured_voltage) * (r_1 + r_2)) / r_2;
  }

  public DeviceInterfaceModule connectToDIMusingSerial(String serial_port) {
    DeviceInterfaceModule cdi;
    SyncdDevice.Manager synced_device_manager = new SyncdDevice.Manager() {
      @Override
      public void registerSyncdDevice(SyncdDevice device) {}

      @Override
      public void unregisterSyncdDevice(SyncdDevice device) {}
    };

    HardwareDeviceManager hardware_device_manager = new HardwareDeviceManager(this, synced_device_manager);
    int devicesConnected = 0;

    while(devicesConnected < 1) {
      try {
        devicesConnected = hardware_device_manager.scanForUsbDevices().size();
      } catch (RobotCoreException e) {
        devicesConnected = 0;
      }
    }

    try {
      cdi = hardware_device_manager.createDeviceInterfaceModule(SerialNumber.fromString(serial_port), "Test");
    } catch (RobotCoreException | InterruptedException e) {
      cdi = null;
    }
    return cdi;
  }

//
//    public void connect() {
//      System.out.println("---------------- Connecting ----------------");
//      try {
//        RobotUsbManager usbManager = new RobotUsbManagerFtdi();
//
//        List<SerialNumber> serialNumbers = usbManager.scanForDevices();
//
//        int len = serialNumbers.size();
//        for(SerialNumber serialNumber:serialNumbers) {
//          RobotUsbDevice device = ModernRoboticsUsbUtil.openRobotUsbDevice(false, usbManager, serialNumber);
//          System.out.println("HIHIHIH| ---------- " + device.toString());
//
//
//
////        ModernRoboticsUsbDeviceInterfaceModule module = new ModernRoboticsUsbDeviceInterfaceModule(this, serialNumber, device, usbManager);
//        }
//
//        textRobotStatus.setText("Devices: " + String.valueOf(len));
//        System.out.println("---------------- Connected to " + String.valueOf(len)+ " Devices ----------------");
//
//      } catch (Exception e) {
//        System.out.println(e.toString());
//        System.out.println("---------------- Failed to Connect ----------------");
//
//      }
//
//    }

  @Override
  protected void onStart() {
    super.onStart();
  }


  @Override
  protected void onStop() {
    // Note: this gets called even when the configuration editor is launched. That is, it gets
    // called surprisingly often. So, we don't actually do much here.
    super.onStop();
  }
}
