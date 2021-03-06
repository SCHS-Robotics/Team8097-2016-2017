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

import android.app.ActionBar;
import android.app.Activity;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.content.res.Configuration;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.ViewTreeObserver;
import android.webkit.WebView;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.TextView;

import com.google.blocks.ftcrobotcontroller.BlocksActivity;
import com.google.blocks.ftcrobotcontroller.ProgrammingModeActivity;
import com.google.blocks.ftcrobotcontroller.ProgrammingModeControllerImpl;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;
import com.qualcomm.ftccommon.AboutActivity;
import com.qualcomm.ftccommon.ClassManagerFactory;
import com.qualcomm.ftccommon.Device;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.FtcEventLoopIdle;
import com.qualcomm.ftccommon.FtcRobotControllerService;
import com.qualcomm.ftccommon.FtcRobotControllerService.FtcRobotControllerBinder;
import com.qualcomm.ftccommon.FtcRobotControllerSettingsActivity;
import com.qualcomm.ftccommon.LaunchActivityConstantsList;
import com.qualcomm.ftccommon.ProgrammingModeController;
import com.qualcomm.ftccommon.Restarter;

import org.firstinspires.ftc.ftccommon.external.SoundPlayingRobotMonitor;

import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.ftccommon.configuration.EditParameters;
import com.qualcomm.ftccommon.configuration.FtcLoadFileActivity;
import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.HardwareFactory;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.robocol.PeerAppRobotController;
import com.qualcomm.robotcore.util.Dimmer;
import com.qualcomm.robotcore.util.ImmersiveMode;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.wifi.NetworkConnectionFactory;
import com.qualcomm.robotcore.wifi.NetworkType;
import com.qualcomm.robotcore.wifi.WifiDirectAssistant;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.inspection.RcInspectionActivity;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

public class FtcRobotControllerActivity extends Activity { /*testing:*/ //implements CameraBridgeViewBase.CvCameraViewListener2 {

    public static final String TAG = "RCActivity";

    private static final int REQUEST_CONFIG_WIFI_CHANNEL = 1;
    private static final boolean USE_DEVICE_EMULATION = false;
    private static final int NUM_GAMEPADS = 2;

    public static final String NETWORK_TYPE_FILENAME = "ftc-network-type.txt";

    protected WifiManager.WifiLock wifiLock;
    protected RobotConfigFileManager cfgFileMgr;

    protected ProgrammingModeController programmingModeController;

    protected UpdateUI.Callback callback;
    protected Context context;
    protected Utility utility;
    protected AppUtil appUtil = AppUtil.getInstance();

    protected ImageButton buttonMenu;
    protected TextView textDeviceName;
    protected TextView textNetworkConnectionStatus;
    protected TextView textRobotStatus;
    protected TextView[] textGamepad = new TextView[NUM_GAMEPADS];
    protected TextView textOpMode;
    protected TextView textErrorMessage;
    protected ImmersiveMode immersion;

    public static JavaCameraView mOpenCvCameraView;

    public static TextView textDataLog;
    public static Button increaseExposureButt;
    public static Button decreaseExposureButt;
    public static TextView exposureText;
    public static int exposureValue;
    public static int exposureIncrement = 1;
    public final static int MIN_EXPOSURE = -12;
    public final static int MAX_EXPOSURE = 12;
    public static Button calibrateGroundButt;
    public static Button calibrateTapeButt;
    public static boolean calibrateTape;
    public static boolean calibrateGround;
    protected SharedPreferences preferences;
    public static SharedPreferences calibrationSP;
    public final static String CALIBRATE_SP = "org.firstinspires.ftc.robotcontroller.calibrate";

    protected UpdateUI updateUI;
    protected Dimmer dimmer;
    protected ViewGroup entireScreenLayout;

    protected FtcRobotControllerService controllerService;
    protected NetworkType networkType;

    protected FtcEventLoop eventLoop;
    protected Queue<UsbDevice> receivedUsbAttachmentNotifications;

    protected class RobotRestarter implements Restarter {

        public void requestRestart() {
            requestRobotRestart();
        }

    }

    protected ServiceConnection connection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            FtcRobotControllerBinder binder = (FtcRobotControllerBinder) service;
            onServiceBind(binder.getService());
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            RobotLog.vv(FtcRobotControllerService.TAG, "%s.controllerService=null", TAG);
            controllerService = null;
        }
    };

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);

        if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(intent.getAction())) {
            UsbDevice usbDevice = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
            if (usbDevice != null) {  // paranoia
                // We might get attachment notifications before the event loop is set up, so
                // we hold on to them and pass them along only when we're good and ready.
                if (receivedUsbAttachmentNotifications != null) { // *total* paranoia
                    receivedUsbAttachmentNotifications.add(usbDevice);
                    passReceivedUsbAttachmentsToEventLoop();
                }
            }
        }
    }

    protected void passReceivedUsbAttachmentsToEventLoop() {
        if (this.eventLoop != null) {
            for (; ; ) {
                UsbDevice usbDevice = receivedUsbAttachmentNotifications.poll();
                if (usbDevice == null)
                    break;
                this.eventLoop.onUsbDeviceAttached(usbDevice);
            }
        } else {
            // Paranoia: we don't want the pending list to grow without bound when we don't
            // (yet) have an event loop
            while (receivedUsbAttachmentNotifications.size() > 100) {
                receivedUsbAttachmentNotifications.poll();
            }
        }
    }

    HashMap<View, Integer> left = new HashMap<>();
    HashMap<View, Integer> right = new HashMap<>();
    HashMap<View, Integer> top = new HashMap<>();
    HashMap<View, Integer> bottom = new HashMap<>();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        RobotLog.vv(TAG, "onCreate()");

        receivedUsbAttachmentNotifications = new ConcurrentLinkedQueue<UsbDevice>();
        eventLoop = null;

        setContentView(R.layout.activity_ftc_controller);

        context = this;
        utility = new Utility(this);
        appUtil.setThisApp(new PeerAppRobotController(context));

        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        entireScreenLayout = (ViewGroup) findViewById(R.id.entire_screen);
        entireScreenLayout.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
            private int n1 = 0;
            private int n2 = 0;

            @Override
            public void onGlobalLayout() {
                if (FtcRobotControllerActivity.this.getResources().getConfiguration().orientation == Configuration.ORIENTATION_PORTRAIT) {
                    if (n1 == 1) {
                        saveLayout(entireScreenLayout);
                        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
                    }
                    n1++;
                } else if (FtcRobotControllerActivity.this.getResources().getConfiguration().orientation == Configuration.ORIENTATION_LANDSCAPE) {
                    if (n2 == 0) {
                        restoreLayout(entireScreenLayout);
                        entireScreenLayout.setPivotX((entireScreenLayout.getRight() - entireScreenLayout.getLeft()) / 2);
                        entireScreenLayout.setPivotY((entireScreenLayout.getRight() - entireScreenLayout.getLeft()) / 2);
                        entireScreenLayout.setRotation(-90);
                    } else if (n2 >= 1) {
                        restoreLayout(entireScreenLayout);
                    }
                    n2++;
                }
            }
        });

        buttonMenu = (ImageButton) findViewById(R.id.menu_buttons);
        buttonMenu.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                AppUtil.getInstance().openOptionsMenuFor(FtcRobotControllerActivity.this);
            }
        });

        BlocksOpMode.setActivityAndWebView(this, (WebView) findViewById(R.id.webViewBlocksRuntime));

        ClassManagerFactory.processClasses();
        cfgFileMgr = new RobotConfigFileManager(this);

        // Clean up 'dirty' status after a possible crash
        RobotConfigFile configFile = cfgFileMgr.getActiveConfig();
        if (configFile.isDirty()) {
            configFile.markClean();
            cfgFileMgr.setActiveConfig(false, configFile);
        }

        textDeviceName = (TextView) findViewById(R.id.textDeviceName);
        textNetworkConnectionStatus = (TextView) findViewById(R.id.textNetworkConnectionStatus);
        textRobotStatus = (TextView) findViewById(R.id.textRobotStatus);
        textOpMode = (TextView) findViewById(R.id.textOpMode);
        textErrorMessage = (TextView) findViewById(R.id.textErrorMessage);
        textGamepad[0] = (TextView) findViewById(R.id.textGamepad1);
        textGamepad[1] = (TextView) findViewById(R.id.textGamepad2);
        immersion = new ImmersiveMode(getWindow().getDecorView());
        dimmer = new Dimmer(this);
        dimmer.longBright();

        mOpenCvCameraView = (JavaCameraView) findViewById(R.id.cameraView);

        //testing
//        mOpenCvCameraView.setVisibility(View.VISIBLE);
//        mOpenCvCameraView.setCvCameraViewListener(this);
        //testing
        calibrationSP = getSharedPreferences(CALIBRATE_SP, MODE_PRIVATE);

        textDataLog = (TextView) findViewById(R.id.textDataLog);
        increaseExposureButt = (Button) findViewById(R.id.plusExposure);
        decreaseExposureButt = (Button) findViewById(R.id.minusExposure);
        exposureText = (TextView) findViewById(R.id.exposureText);
        exposureValue = calibrationSP.getInt("exposure", 0);
        exposureText.setText("Exposure: " + exposureValue);


        calibrateGroundButt = (Button) findViewById(R.id.calibrateGroundButt);
        calibrateTapeButt = (Button) findViewById(R.id.calibrateTapeButt);
        calibrateTapeButt.setText("Calibrate Tape\nF: " + (float) ((int) (calibrationSP.getFloat("frontTapeValue", -1000) * 100) / 100.0) + "  B: " + (float) ((int) (calibrationSP.getFloat("backTapeValue", -1000) * 100) / 100.0));
        calibrateGroundButt.setText("Calibrate Floor\nF: " + (float) ((int) (calibrationSP.getFloat("frontGroundValue", -1000) * 100) / 100.0) + "  B: " + (float) ((int) (calibrationSP.getFloat("backGroundValue", -1000) * 100) / 100.0));

        programmingModeController = new ProgrammingModeControllerImpl(
                this, (TextView) findViewById(R.id.textRemoteProgrammingMode));

        updateUI = createUpdateUI();
        callback = createUICallback(updateUI);

        PreferenceManager.setDefaultValues(this, R.xml.preferences, false);

        WifiManager wifiManager = (WifiManager) getSystemService(Context.WIFI_SERVICE);
        wifiLock = wifiManager.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, "");

        hittingMenuButtonBrightensScreen();

        if (USE_DEVICE_EMULATION) {
            HardwareFactory.enableDeviceEmulation();
        }

        // save 4MB of logcat to the SD card
        RobotLog.writeLogcatToDisk(this, 4 * 1024);
        wifiLock.acquire();
        callback.networkConnectionUpdate(WifiDirectAssistant.Event.DISCONNECTED);
        bindToService();
    }

    private void saveLayout(ViewGroup vg) {
        left.put(vg, vg.getLeft());
        right.put(vg, vg.getRight());
        top.put(vg, vg.getTop());
        bottom.put(vg, vg.getBottom());
        for (int i = 0; i < vg.getChildCount(); i++) {
            View v = vg.getChildAt(i);
            if (v instanceof ViewGroup) {
                saveLayout((ViewGroup) v);
            } else {
                if (v.getId() == R.id.fakeCameraView) {
                    int[] location = new int[2];
                    v.getLocationOnScreen(location);
                    int x = location[0];
                    int y = location[1];
                    top.put(v, x);
                    bottom.put(v, x + v.getWidth());
                    left.put(v, y);
                    right.put(v, y + v.getHeight());
                    v.setVisibility(View.GONE);
                } else if (v.getId() != R.id.cameraView) {
                    left.put(v, v.getLeft());
                    right.put(v, v.getRight());
                    top.put(v, v.getTop());
                    bottom.put(v, v.getBottom());
                }
            }
        }
    }

    private void restoreLayout(ViewGroup vg) {
        vg.setLeft(left.get(vg));
        vg.setRight(right.get(vg));
        vg.setTop(top.get(vg));
        vg.setBottom(bottom.get(vg));
        for (int i = 0; i < vg.getChildCount(); i++) {
            View v = vg.getChildAt(i);
            if (v instanceof ViewGroup) {
                restoreLayout((ViewGroup) v);
            } else {
                if (v.getId() == R.id.fakeCameraView) {
                    mOpenCvCameraView.setLeft(left.get(v));
                    mOpenCvCameraView.setRight(right.get(v));
                    mOpenCvCameraView.setTop(top.get(v));
                    mOpenCvCameraView.setBottom(bottom.get(v));
                } else if (v.getId() != R.id.cameraView) {
                    v.setLeft(left.get(v));
                    v.setRight(right.get(v));
                    v.setTop(top.get(v));
                    v.setBottom(bottom.get(v));
                }
            }
        }
    }

    protected UpdateUI createUpdateUI() {
        Restarter restarter = new RobotRestarter();
        UpdateUI result = new UpdateUI(this, dimmer);
        result.setRestarter(restarter);
        result.setTextViews(textNetworkConnectionStatus, textRobotStatus, textGamepad, textOpMode, textErrorMessage, textDeviceName);
        return result;
    }

    protected UpdateUI.Callback createUICallback(UpdateUI updateUI) {
        UpdateUI.Callback result = updateUI.new Callback();
        result.setStateMonitor(new SoundPlayingRobotMonitor());
        return result;
    }

    @Override
    protected void onStart() {
        super.onStart();
        RobotLog.vv(TAG, "onStart()");

        // Undo the effects of shutdownRobot() that we might have done in onStop()
        updateUIAndRequestRobotSetup();

        cfgFileMgr.getActiveConfigAndUpdateUI();

        entireScreenLayout.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                dimmer.handleDimTimer();
                return false;
            }
        });
    }

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };

    @Override
    protected void onResume() {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        RobotLog.vv(TAG, "onResume()");
        readNetworkType(NETWORK_TYPE_FILENAME);
    }

    @Override
    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        RobotLog.vv(TAG, "onPause()");
        if (programmingModeController.isActive()) {
            programmingModeController.stopProgrammingMode();
        }
    }

    @Override
    protected void onStop() {
        // Note: this gets called even when the configuration editor is launched. That is, it gets
        // called surprisingly often.
        super.onStop();
        RobotLog.vv(TAG, "onStop()");

        // We *do* shutdown the robot even when we go into configuration editing
        controllerService.shutdownRobot();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        RobotLog.vv(TAG, "onDestroy()");
        unbindFromService();
        wifiLock.release();
        RobotLog.cancelWriteLogcatToDisk(this);
    }

    protected void bindToService() {
        readNetworkType(NETWORK_TYPE_FILENAME);
        Intent intent = new Intent(this, FtcRobotControllerService.class);
        intent.putExtra(NetworkConnectionFactory.NETWORK_CONNECTION_TYPE, networkType);
        bindService(intent, connection, Context.BIND_AUTO_CREATE);
    }

    protected void unbindFromService() {
        if (controllerService != null) {
            unbindService(connection);
        }
    }

    public void writeNetworkTypeFile(String fileName, String fileContents) {
        ReadWriteFile.writeFile(AppUtil.FIRST_FOLDER, fileName, fileContents);
    }

    protected void readNetworkType(String fileName) {
        NetworkType defaultNetworkType;
        File directory = RobotConfigFileManager.CONFIG_FILES_DIR;
        File networkTypeFile = new File(directory, fileName);
        if (!networkTypeFile.exists()) {
            if (Build.MODEL.equals(Device.MODEL_410C)) {
                defaultNetworkType = NetworkType.SOFTAP;
            } else {
                defaultNetworkType = NetworkType.WIFIDIRECT;
            }
            writeNetworkTypeFile(NETWORK_TYPE_FILENAME, defaultNetworkType.toString());
        }

        String fileContents = readFile(networkTypeFile);
        networkType = NetworkConnectionFactory.getTypeFromString(fileContents);
        programmingModeController.setCurrentNetworkType(networkType);
    }

    private String readFile(File file) {
        return ReadWriteFile.readFile(file);
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        // When the window loses focus (e.g., the action overflow is shown),
        // cancel any pending hide action. When the window gains focus,
        // hide the system UI.
        if (hasFocus) {
            if (ImmersiveMode.apiOver19()) {
                // Immersive flag only works on API 19 and above.
                immersion.hideSystemUI();
            }
        } else {
            immersion.cancelSystemUIHide();
        }
    }


    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        mOpenCvCameraView.disableView();
        getMenuInflater().inflate(R.menu.ftc_robot_controller, menu);
        return true;
    }

    @Override
    public void onOptionsMenuClosed(Menu menu) {
        super.onOptionsMenuClosed(menu);
        mOpenCvCameraView.enableView();
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();

        if (id == R.id.action_programming_mode) {
            if (cfgFileMgr.getActiveConfig().isNoConfig()) {
                // Tell the user they must configure the robot before starting programming mode.
                AppUtil.getInstance().showToast(
                        context, context.getString(R.string.toastConfigureRobotBeforeProgrammingMode));
            } else {
                Intent programmingModeIntent = new Intent(ProgrammingModeActivity.launchIntent);
                programmingModeIntent.putExtra(
                        LaunchActivityConstantsList.PROGRAMMING_MODE_ACTIVITY_NETWORK_TYPE, networkType);
                startActivity(programmingModeIntent);
            }
            return true;
        } else if (id == R.id.action_inspection_mode) {
            Intent inspectionModeIntent = new Intent(RcInspectionActivity.rcLaunchIntent);
            startActivity(inspectionModeIntent);
            return true;
        } else if (id == R.id.action_blocks) {
            Intent blocksIntent = new Intent(BlocksActivity.launchIntent);
            startActivity(blocksIntent);
            return true;
        } else if (id == R.id.action_restart_robot) {
            dimmer.handleDimTimer();
            AppUtil.getInstance().showToast(context, context.getString(R.string.toastRestartingRobot));
            requestRobotRestart();
            return true;
        } else if (id == R.id.action_configure_robot) {
            EditParameters parameters = new EditParameters();
            Intent intentConfigure = new Intent(FtcLoadFileActivity.launchIntent);
            parameters.putIntent(intentConfigure);
            startActivityForResult(intentConfigure, LaunchActivityConstantsList.FTC_CONFIGURE_REQUEST_CODE_ROBOT_CONTROLLER);
        } else if (id == R.id.action_settings) {
            Intent settingsIntent = new Intent(FtcRobotControllerSettingsActivity.launchIntent);
            startActivityForResult(settingsIntent, LaunchActivityConstantsList.FTC_CONFIGURE_REQUEST_CODE_ROBOT_CONTROLLER);
            return true;
        } else if (id == R.id.action_about) {
            Intent intent = new Intent(AboutActivity.launchIntent);
            intent.putExtra(LaunchActivityConstantsList.ABOUT_ACTIVITY_CONNECTION_TYPE, networkType);
            startActivity(intent);
            return true;
        } else if (id == R.id.action_exit_app) {
            finish();
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    @Override
    public void onConfigurationChanged(Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
        // don't destroy assets on screen rotation
    }

    @Override
    protected void onActivityResult(int request, int result, Intent intent) {
        if (request == REQUEST_CONFIG_WIFI_CHANNEL) {
            if (result == RESULT_OK) {
                AppUtil.getInstance().showToast(context, context.getString(R.string.toastWifiConfigurationComplete));
            }
        }
        if (request == LaunchActivityConstantsList.FTC_CONFIGURE_REQUEST_CODE_ROBOT_CONTROLLER) {
            // We always do a refresh, whether it was a cancel or an OK, for robustness
            cfgFileMgr.getActiveConfigAndUpdateUI();
        }
    }

    public void onServiceBind(FtcRobotControllerService service) {
        RobotLog.vv(FtcRobotControllerService.TAG, "%s.controllerService=bound", TAG);
        controllerService = service;
        updateUI.setControllerService(controllerService);

        updateUIAndRequestRobotSetup();
    }

    private void updateUIAndRequestRobotSetup() {
        if (controllerService != null) {
            callback.networkConnectionUpdate(controllerService.getNetworkConnectionStatus());
            callback.updateRobotStatus(controllerService.getRobotStatus());
            requestRobotSetup();
        }
    }

    private void requestRobotSetup() {
        if (controllerService == null) return;

        HardwareFactory factory;
        RobotConfigFile file = cfgFileMgr.getActiveConfigAndUpdateUI();
        HardwareFactory hardwareFactory = new HardwareFactory(context);
        hardwareFactory.setXmlPullParser(file.getXml());
        factory = hardwareFactory;

        eventLoop = new FtcEventLoop(factory, createOpModeRegister(), callback, this, programmingModeController);
        FtcEventLoopIdle idleLoop = new FtcEventLoopIdle(factory, callback, this, programmingModeController);

        controllerService.setCallback(callback);
        controllerService.setupRobot(eventLoop, idleLoop);

        passReceivedUsbAttachmentsToEventLoop();
    }

    protected OpModeRegister createOpModeRegister() {
        return new FtcOpModeRegister();
    }

    private void requestRobotShutdown() {
        if (controllerService == null) return;
        controllerService.shutdownRobot();
    }

    private void requestRobotRestart() {
        requestRobotShutdown();
        requestRobotSetup();
    }

    protected void hittingMenuButtonBrightensScreen() {
        ActionBar actionBar = getActionBar();
        if (actionBar != null) {
            actionBar.addOnMenuVisibilityListener(new ActionBar.OnMenuVisibilityListener() {
                @Override
                public void onMenuVisibilityChanged(boolean isVisible) {
                    if (isVisible) {
                        dimmer.handleDimTimer();
                    }
                }
            });
        }
    }

    public void plusExposure(View v) {
        exposureValue = Math.min(exposureValue + exposureIncrement, MAX_EXPOSURE);
        SharedPreferences.Editor editor = FtcRobotControllerActivity.calibrationSP.edit();
        editor.putInt("exposure", exposureValue);
        editor.apply();
        exposureText.setText("Exposure: " + exposureValue);
    }

    public void minusExposure(View v) {
        exposureValue = Math.max(exposureValue - exposureIncrement, MIN_EXPOSURE);
        SharedPreferences.Editor editor = FtcRobotControllerActivity.calibrationSP.edit();
        editor.putInt("exposure", exposureValue);
        editor.apply();
        exposureText.setText("Exposure: " + exposureValue);
    }

    public final static Handler setExposureButtonsClickable = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            increaseExposureButt.setClickable(true);
            decreaseExposureButt.setClickable(true);
        }
    };

    public final static Handler setExposureButtonsUnclickable = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            increaseExposureButt.setClickable(false);
            decreaseExposureButt.setClickable(false);
        }
    };

    public void calibrateTape(View v) {
        calibrateTape = true;
    }

    public void calibrateGround(View v) {
        calibrateGround = true;
    }

    public final static Handler setButtonsClickable = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            calibrateGroundButt.setClickable(true);
            calibrateTapeButt.setClickable(true);
        }
    };

    public final static Handler setButtonsUnclickable = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            calibrateGroundButt.setClickable(false);
            calibrateTapeButt.setClickable(false);
        }
    };

    public final static Handler setGroundText = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            float front = (float) (msg.arg1 / 100.0);
            float back = (float) (msg.arg2 / 100.0);
            calibrateGroundButt.setText("Calibrate Floor\nF: " + front + "  B: " + back);
        }
    };

    public final static Handler setTapeText = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            float front = (float) (msg.arg1 / 100.0);
            float back = (float) (msg.arg2 / 100.0);
            calibrateTapeButt.setText("Calibrate Tape\nF: " + front + "  B: " + back);
        }
    };

    public final static Handler logData = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            textDataLog.setText((String) msg.obj);
        }
    };

    public final static Handler turnOnCameraView = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            mOpenCvCameraView.setVisibility(View.VISIBLE);
            textDataLog.setVisibility(View.GONE);
        }
    };


    //testing
//    private boolean mIsColorSelected = false;
//    private Mat mRgba;
//    private Scalar mBlobColorRgba;
//    private Scalar mBlobColorHsv;
//    private ColorBlobDetector mDetector;
//    private Mat mSpectrum;
//    private Size SPECTRUM_SIZE;
//    private Scalar CONTOUR_COLOR;
//
//    public void onCameraViewStarted(int width, int height) {
//        mRgba = new Mat(height, width, CvType.CV_8UC4);
//        mDetector = new ColorBlobDetector();
//        mSpectrum = new Mat();
//        mBlobColorRgba = new Scalar(255);
//        mBlobColorHsv = new Scalar(255);
//        SPECTRUM_SIZE = new Size(200, 64);
//        CONTOUR_COLOR = new Scalar(0, 0, 255, 255);//for red, CONTOUR_COLOR = new Scalar(255, 0, 0, 255); for blue
//
//        mBlobColorHsv = new Scalar(0, 255, 255);//red, mBlobColorHsv = new Scalar(170, 255, 255) is blue
//        mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv);
//
//        Log.i(TAG, "rgba color: (" + mBlobColorRgba.val[0] + ", " + mBlobColorRgba.val[1] +
//                ", " + mBlobColorRgba.val[2] + ", " + mBlobColorRgba.val[3] + ")");
//
//        mDetector.setHsvColor(mBlobColorHsv);
//
//        Imgproc.resize(mDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);
//
//        mIsColorSelected = true;
//    }
//
//    public void onCameraViewStopped() {
//        mRgba.release();
//    }
//
//    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
//        mRgba = inputFrame.rgba();
//
//        if (mIsColorSelected) {
//            mDetector.process(mRgba);
//            List<MatOfPoint> contours = mDetector.getContours();
//            Log.e(TAG, "Contours count: " + contours.size());
//            Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);
//
//            Mat colorLabel = mRgba.submat(4, 68, 4, 68);
//            colorLabel.setTo(mBlobColorRgba);
//
//            Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
//            mSpectrum.copyTo(spectrumLabel);
//
//            double maxWidth = 0;
//            double maxHeight = 0;
//            for (MatOfPoint contour : mDetector.getContours()) {
//                double left = contour.toArray()[0].x;
//                double right = contour.toArray()[0].x;
//                double top = contour.toArray()[0].y;
//                double bottom = contour.toArray()[0].y;
//                for (Point p : contour.toArray()) {
//                    left = p.x < left ? p.x : left;
//                    right = p.x > right ? p.x : right;
//                    top = p.y < top ? p.y : top;
//                    bottom = p.y > bottom ? p.y : bottom;
//                }
//                double width = right - left;
//                double height = bottom - top;
//                maxWidth = width > maxWidth ? width : maxWidth;
//                maxHeight = height > maxHeight ? height : maxHeight;
//            }
//            Log.i("contourtest", "width: " + maxWidth + ", height: " + maxHeight);
//        }
//
//        return mRgba;
//    }
//
//    private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
//        Mat pointMatRgba = new Mat();
//        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
//        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);
//
//        return new Scalar(pointMatRgba.get(0, 0));
//    }
//
//    static class ColorBlobDetector {
//        // Lower and Upper bounds for range checking in HSV color space
//        private Scalar mLowerBound = new Scalar(0);
//        private Scalar mUpperBound = new Scalar(0);
//        // Minimum contour area in percent for contours filtering
//        private static double mMinContourArea = 0.1;
//        // Color radius for range checking in HSV color space
//        private Scalar mColorRadius = new Scalar(30, 100, 100, 0);
//        private Mat mSpectrum = new Mat();
//        private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();
//
//        // Cache
//        Mat mPyrDownMat = new Mat();
//        Mat mHsvMat = new Mat();
//        Mat mMask = new Mat();
//        Mat mMask1 = new Mat();
//        Mat mMask2 = new Mat();
//        Mat mDilatedMask = new Mat();
//        Mat mHierarchy = new Mat();
//
//        public void setColorRadius(Scalar radius) {
//            mColorRadius = radius;
//        }
//
//        public void setHsvColor(Scalar hsvColor) {
//            double minH = hsvColor.val[0] - mColorRadius.val[0]; //(hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0] - mColorRadius.val[0] : 0;
//            double maxH = hsvColor.val[0] + mColorRadius.val[0]; //(hsvColor.val[0] + mColorRadius.val[0] <= 255) ? hsvColor.val[0] + mColorRadius.val[0] : 255;
//
//            mLowerBound.val[0] = minH;
//            mUpperBound.val[0] = maxH;
//
//            mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
//            mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];
//
//            mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
//            mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];
//
//            mLowerBound.val[3] = 0;
//            mUpperBound.val[3] = 255;
//
//            Mat spectrumHsv = new Mat(1, (int) (maxH - minH), CvType.CV_8UC3);
//
//            for (int j = 0; j < maxH - minH; j++) {
//                int minHPlusJ = (int) (minH + j);
//                if (minHPlusJ < 0) {
//                    minHPlusJ += 256;
//                }
//                byte[] tmp = {(byte) minHPlusJ, (byte) 255, (byte) 255};
//                spectrumHsv.put(0, j, tmp);
//            }
//
//            Imgproc.cvtColor(spectrumHsv, mSpectrum, Imgproc.COLOR_HSV2RGB_FULL, 4);
//        }
//
//        public Mat getSpectrum() {
//            return mSpectrum;
//        }
//
//        public void setMinContourArea(double area) {
//            mMinContourArea = area;
//        }
//
//        public void process(Mat rgbaImage) {
//            Imgproc.pyrDown(rgbaImage, mPyrDownMat);
//            Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);
//
//            Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);
//
//            if (mLowerBound.val[0] < 0) {
//                Scalar lowBound = new Scalar(mLowerBound.val[0] + 256, mLowerBound.val[1], mLowerBound.val[2], mLowerBound.val[3]);
//                Scalar minBound = new Scalar(0, mLowerBound.val[1], mLowerBound.val[2], mLowerBound.val[3]);
//                Scalar maxBound = new Scalar(255, mUpperBound.val[1], mUpperBound.val[2], mUpperBound.val[3]);
//                Core.inRange(mHsvMat, minBound, mUpperBound, mMask1);
//                Core.inRange(mHsvMat, lowBound, maxBound, mMask2);
//                Core.bitwise_or(mMask2, mMask1, mMask);
//            } else if (mUpperBound.val[0] > 255) {
//                Scalar highBound = new Scalar(mUpperBound.val[0] - 256, mUpperBound.val[1], mUpperBound.val[2], mUpperBound.val[3]);
//                Scalar minBound = new Scalar(0, mLowerBound.val[1], mLowerBound.val[2], mLowerBound.val[3]);
//                Scalar maxBound = new Scalar(255, mUpperBound.val[1], mUpperBound.val[2], mUpperBound.val[3]);
//                Core.inRange(mHsvMat, minBound, highBound, mMask1);
//                Core.inRange(mHsvMat, mLowerBound, maxBound, mMask2);
//                Core.bitwise_or(mMask2, mMask1, mMask);
//            } else {
//                Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
//            }
//            Imgproc.dilate(mMask, mDilatedMask, new Mat());
//
//            List<MatOfPoint> contours = new ArrayList<>();
//
//            Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            // Find max contour area
//            double maxArea = 0;
//            Iterator<MatOfPoint> each = contours.iterator();
//            while (each.hasNext()) {
//                MatOfPoint wrapper = each.next();
//                double area = Imgproc.contourArea(wrapper);
//                if (area > maxArea)
//                    maxArea = area;
//            }
//
//            // Filter contours by area and resize to fit the original image size
//            mContours.clear();
//            each = contours.iterator();
//            while (each.hasNext()) {
//                MatOfPoint contour = each.next();
//                if (Imgproc.contourArea(contour) > mMinContourArea * maxArea) {
//                    Core.multiply(contour, new Scalar(4, 4), contour);
//                    mContours.add(contour);
//                }
//            }
//        }
//
//        public List<MatOfPoint> getContours() {
//            return mContours;
//        }
//    }
}
