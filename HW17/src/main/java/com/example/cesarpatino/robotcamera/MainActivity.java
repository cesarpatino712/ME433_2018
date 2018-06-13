package com.example.cesarpatino.robotcamera;

// libraries

import android.Manifest;
import android.app.Activity;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.View;
import android.view.WindowManager;
import android.widget.TextView;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;


import com.hoho.android.usbserial.driver.CdcAcmSerialDriver;
import com.hoho.android.usbserial.driver.ProbeTable;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static android.graphics.Color.rgb;



public class MainActivity extends Activity implements TextureView.SurfaceTextureListener {
    private UsbManager manager;
    private UsbSerialPort sPort;
    private final ExecutorService mExecutor = Executors.newSingleThreadExecutor();
    private SerialInputOutputManager mSerialIoManager;
    SeekBar myControl;
    TextView myTextView;
    private Camera mCamera;
    private TextureView mTextureView;
    private SurfaceView mSurfaceView;
    private SurfaceHolder mSurfaceHolder;
    private Bitmap bmp = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
    private Canvas canvas = new Canvas(bmp);
    private Paint paint1 = new Paint();
    private TextView mTextView;


    static long prevtime = 0; // for FPS calculation
    static int thresh = 0; //threshold

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON); // keeps the screen from turning off
        myControl = (SeekBar) findViewById(R.id.seek1);

        myTextView = (TextView) findViewById(R.id.textView01);
        myTextView.setText("Thresh");

        mTextView = (TextView) findViewById(R.id.cameraStatus);

        setMyControlListener();

        // see if the app has permission to use the camera
        //ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.CAMERA}, 1);
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            mSurfaceView = (SurfaceView) findViewById(R.id.surfaceview);
            mSurfaceHolder = mSurfaceView.getHolder();

            mTextureView = (TextureView) findViewById(R.id.textureview);
            mTextureView.setSurfaceTextureListener(this);

            // set the paintbrush for writing text on the image
            paint1.setColor(0xffff0000); // red
            paint1.setTextSize(24);

            mTextView.setText("started camera");
        } else {
            mTextView.setText("no camera permissions");
        }

        manager = (UsbManager) getSystemService(Context.USB_SERVICE);

    }

    private void setMyControlListener() {
        myControl.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            int progressChanged = 0;

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged = progress;
                thresh = progressChanged;
                myTextView.setText("Thresh value is: "+progress);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }

    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        mCamera = Camera.open();
        Camera.Parameters parameters = mCamera.getParameters();
        parameters.setPreviewSize(640, 480);
        parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY); // no autofocusing
        parameters.setAutoExposureLock(true); // keep the white balance constant
        mCamera.setParameters(parameters);
        mCamera.setDisplayOrientation(90); // rotate to portrait mode

        try {
            mCamera.setPreviewTexture(surface);
            mCamera.startPreview();
        } catch (IOException ioe) {
            // Something bad happened
        }
    }

    private final SerialInputOutputManager.Listener mListener =
            new SerialInputOutputManager.Listener() {
                @Override
                public void onRunError(Exception e) {

                }

                @Override
                public void onNewData(final byte[] data) {
                    MainActivity.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            MainActivity.this.updateReceivedData(data);
                        }
                    });
                }
            };

    @Override
    protected void onPause(){
        super.onPause();
        stopIoManager();
        if(sPort != null){
            try{
                sPort.close();
            } catch (IOException e){ }
            sPort = null;
        }
        finish();
    }

    @Override
    protected void onResume() {
        super.onResume();

        ProbeTable customTable = new ProbeTable();
        customTable.addProduct(0x04D8,0x000A, CdcAcmSerialDriver.class);
        UsbSerialProber prober = new UsbSerialProber(customTable);

        final List<UsbSerialDriver> availableDrivers = prober.findAllDrivers(manager);

        if(availableDrivers.isEmpty()) {
            //check
            return;
        }

        UsbSerialDriver driver = availableDrivers.get(0);
        sPort = driver.getPorts().get(0);

        if (sPort == null){
            //check
        }else{
            final UsbManager usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
            UsbDeviceConnection connection = usbManager.openDevice(driver.getDevice());
            if (connection == null){
                //check
                PendingIntent pi = PendingIntent.getBroadcast(this, 0, new Intent("com.android.example.USB_PERMISSION"), 0);
                usbManager.requestPermission(driver.getDevice(), pi);
                return;
            }

            try {
                sPort.open(connection);
                sPort.setParameters(9600, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);

            }catch (IOException e) {
                //check
                try{
                    sPort.close();
                } catch (IOException e1) { }
                sPort = null;
                return;
            }
        }
        onDeviceStateChange();
    }

    private void stopIoManager(){
        if(mSerialIoManager != null) {
            mSerialIoManager.stop();
            mSerialIoManager = null;
        }
    }

    private void startIoManager() {
        if(sPort != null){
            mSerialIoManager = new SerialInputOutputManager(sPort, mListener);
            mExecutor.submit(mSerialIoManager);
        }
    }

    private void onDeviceStateChange(){
        stopIoManager();
        startIoManager();
    }

    private void updateReceivedData(byte[] data) {
        //do something with received data

        //for displaying:
        String rxString = null;
        //try {
           // rxString = new String(data, "UTF-8"); // put the data you got into a string
            //myTextView3.append(rxString);
            //myScrollView.fullScroll(View.FOCUS_DOWN);
        //} catch (UnsupportedEncodingException e) {
        //    e.printStackTrace();
        //}
    }
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, Camera does all the work for us
    }

    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        mCamera.stopPreview();
        mCamera.release();
        return true;
    }

    // the important function
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // every time there is a new Camera preview frame
        mTextureView.getBitmap(bmp);

        final Canvas c = mSurfaceHolder.lockCanvas();
        /*if (c != null) {
            //int thresh = 0; // comparison value
            int thresh = 0;
            int[] pixels = new int[bmp.getWidth()]; // pixels[] is the RGBA data
            //int[] startY = new int [480]; // which row in the bitmap to analyze to read
            int startY = 100;
            int count_COM = 0;
            int count = 0;
            int COM = 0;
            //for(int j = 0; j <  startY.length; j ++) {
            bmp.getPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);

            // in the row, see if there is more green than red
            for (int i = 0; i < bmp.getWidth(); i++) {
                if ((green(pixels[i]) - red(pixels[i])) > thresh) {
                    pixels[i] = rgb(0, 255, 0); // over write the pixel with pure green
                    count_COM = count_COM + i;
                    count = count + 1;
                } else {
                    pixels[i] = rgb(0, 0, 0);
                }

            }

            */
        if (c != null) {
            int thresh = 0; // comparison value
            int[] pixels = new int[bmp.getWidth()]; // pixels[] is the RGBA data
            int startY = 200; // which row in the bitmap to analyze to read
            int count_COM = 0;
            int count = 0;
            int COM = 0;
            bmp.getPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);

            // in the row, see if there is more green than red
            for (int i = 0; i < bmp.getWidth(); i++) {
                if ((red(pixels[i]) - green(pixels[i])) > thresh) {
                    pixels[i] = rgb(255, 0, 0); // over write the pixel with pure green
                    count_COM = count_COM + i;
                    count = count + 1;
                }
            }

            // update the row
            bmp.setPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);


            if (count > 5) {

                COM = count_COM / count;
            }
            else{
                COM = bmp.getWidth()/2;
            }

            String sendString = String.valueOf(COM) + '\n';
            try {
                sPort.write(sendString.getBytes(), 10); // 10 is the timeout
            } catch (IOException e) {
            }


            // draw a circle at some position
            int pos = 50;
            canvas.drawCircle(pos, 240, 5, paint1); // x position, y position, diameter, color

            // write the pos as text
            canvas.drawText("COM = " + COM, 10, 200, paint1);
            //canvas.drawText("r = " + pos, 10, 200, paint1);
            c.drawBitmap(bmp, 0, 0, null);
            mSurfaceHolder.unlockCanvasAndPost(c);

            // calculate the FPS to see how fast the code is running
            long nowtime = System.currentTimeMillis();
            long diff = nowtime - prevtime;
            mTextView.setText("FPS " + 1000 / diff);
            prevtime = nowtime;
        }
    }
