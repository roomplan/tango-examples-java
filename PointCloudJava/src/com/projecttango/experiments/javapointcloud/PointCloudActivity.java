/*
 * Copyright 2014 Google Inc. All Rights Reserved.
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

package com.projecttango.experiments.javapointcloud;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.Tango.OnTangoUpdateListener;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;

import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager.NameNotFoundException;
import android.opengl.GLSurfaceView;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import java.io.BufferedOutputStream;
import java.io.BufferedWriter;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.concurrent.Semaphore;

/**
 * Main Activity class for the Point Cloud Sample. Handles the connection to the
 * {@link Tango} service and propagation of Tango XyzIj data to OpenGL and
 * Layout views. OpenGL rendering logic is delegated to the {@link PCrenderer}
 * class.
 */
public class PointCloudActivity extends Activity implements OnClickListener {

	private static final String TAG = PointCloudActivity.class.getSimpleName();
	private static final int SECS_TO_MILLISECS = 1000;
	private Tango mTango;
	private TangoConfig mConfig;

	private PCRenderer mRenderer;
	private GLSurfaceView mGLView;

	private TextView mDeltaTextView;
	private TextView mPoseCountTextView;
	private TextView mPoseTextView;
	private TextView mQuatTextView;
	private TextView mPoseStatusTextView;
	private TextView mTangoEventTextView;
	private TextView mPointCountTextView;
	private TextView mTangoServiceVersionTextView;
	private TextView mApplicationVersionTextView;
	private TextView mAverageZTextView;
	private TextView mFrequencyTextView;

	private Button mFirstPersonButton;
	private Button mThirdPersonButton;
	private Button mTopDownButton;

	private int count;
	private int mPreviousPoseStatus;
	private int mPointCount;
	private float mDeltaTime;
	private float mPosePreviousTimeStamp;
	private float mXyIjPreviousTimeStamp;
	private float mCurrentTimeStamp;
	private float mPointCloudFrameDelta;
	private String mServiceVersion;
	private boolean mIsTangoServiceConnected;
	private TangoPoseData mPose;
	private static final int UPDATE_INTERVAL_MS = 100;

	public static Object poseLock = new Object();
	public static Object depthLock = new Object();

	// roomplan

	private static final String MainDir = Environment
			.getExternalStorageDirectory().getAbsolutePath() + "/PCD/";
	public static final String SaveDir = MainDir+"PCLData/";
	public static final String mFilenameSS2DPose = SaveDir + "SS2Dev.txt";
	// flag is set, if user wants to record a pointcloud
	private boolean mTimeToTakePointCloud = false;
	// number of poses
	private int mNumPoseCount=0;

	private Semaphore mutex_on_mIsRecording;
	private String mLastPointCloudFilename;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_jpoint_cloud);
		setTitle(R.string.app_name);

		mPoseTextView = (TextView) findViewById(R.id.pose);
		mQuatTextView = (TextView) findViewById(R.id.quat);
		mPoseCountTextView = (TextView) findViewById(R.id.posecount);
		mDeltaTextView = (TextView) findViewById(R.id.deltatime);
		mTangoEventTextView = (TextView) findViewById(R.id.tangoevent);
		mPoseStatusTextView = (TextView) findViewById(R.id.status);
		mPointCountTextView = (TextView) findViewById(R.id.pointCount);
		mTangoServiceVersionTextView = (TextView) findViewById(R.id.version);
		mApplicationVersionTextView = (TextView) findViewById(R.id.appversion);
		mAverageZTextView = (TextView) findViewById(R.id.averageZ);
		mFrequencyTextView = (TextView) findViewById(R.id.frameDelta);

		mFirstPersonButton = (Button) findViewById(R.id.first_person_button);
		mFirstPersonButton.setOnClickListener(this);
		mThirdPersonButton = (Button) findViewById(R.id.third_person_button);
		mThirdPersonButton.setOnClickListener(this);
		mTopDownButton = (Button) findViewById(R.id.top_down_button);
		mTopDownButton.setOnClickListener(this);
		mTango = new Tango(this);
		mConfig = new TangoConfig();
		mConfig = mTango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT);
		mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);

		int maxDepthPoints = mConfig.getInt("max_point_cloud_elements");
		mRenderer = new PCRenderer(maxDepthPoints);
		mGLView = (GLSurfaceView) findViewById(R.id.gl_surface_view);
		mGLView.setEGLContextClientVersion(2);
		mGLView.setRenderer(mRenderer);

		PackageInfo packageInfo;
		try {
			packageInfo = this.getPackageManager().getPackageInfo(
					this.getPackageName(), 0);
			mApplicationVersionTextView.setText(packageInfo.versionName);
		} catch (NameNotFoundException e) {
			e.printStackTrace();
		}

		// Display the version of Tango Service
		mServiceVersion = mConfig.getString("tango_service_library_version");
		mTangoServiceVersionTextView.setText(mServiceVersion);
		mIsTangoServiceConnected = false;
		startUIThread();
		
		createSavingDirectory();		
		mutex_on_mIsRecording = new Semaphore(1, true);
	}

	@Override
	protected void onPause() {
		super.onPause();
		try {
			mTango.disconnect();
			mIsTangoServiceConnected = false;
		} catch (TangoErrorException e) {
			Toast.makeText(getApplicationContext(), R.string.TangoError,
					Toast.LENGTH_SHORT).show();
		}
	}

	@Override
	protected void onResume() {
		super.onResume();
		if (!mIsTangoServiceConnected) {
			startActivityForResult(
					Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_MOTION_TRACKING),
					Tango.TANGO_INTENT_ACTIVITYCODE);
		}
		Log.i(TAG, "onResumed");
	}

	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		// Check which request we're responding to
		if (requestCode == Tango.TANGO_INTENT_ACTIVITYCODE) {
			Log.i(TAG, "Triggered");
			// Make sure the request was successful
			if (resultCode == RESULT_CANCELED) {
				Toast.makeText(this, R.string.motiontrackingpermission,
						Toast.LENGTH_LONG).show();
				finish();
				return;
			}
			try {
				setTangoListeners();
			} catch (TangoErrorException e) {
				Toast.makeText(this, R.string.TangoError, Toast.LENGTH_SHORT)
						.show();
			} catch (SecurityException e) {
				Toast.makeText(getApplicationContext(),
						R.string.motiontrackingpermission, Toast.LENGTH_SHORT)
						.show();
			}
			try {
				mTango.connect(mConfig);
				mIsTangoServiceConnected = true;
			} catch (TangoOutOfDateException e) {
				Toast.makeText(getApplicationContext(),
						R.string.TangoOutOfDateException, Toast.LENGTH_SHORT)
						.show();
			} catch (TangoErrorException e) {
				Toast.makeText(getApplicationContext(), R.string.TangoError,
						Toast.LENGTH_SHORT).show();
			}
			setUpExtrinsics();
		}
	}

	@Override
	protected void onDestroy() {
		super.onDestroy();
	}

	@Override
	public void onClick(View v) {
		switch (v.getId()) {
		case R.id.first_person_button:
			mRenderer.setFirstPersonView();
			break;
		case R.id.third_person_button:
			mRenderer.setThirdPersonView();
			break;
		case R.id.top_down_button:
			mRenderer.setTopDownView();
			break;
		default:
			Log.w(TAG, "Unrecognized button click.");
			return;
		}
	}

	@Override
	public boolean onTouchEvent(MotionEvent event) {
		return mRenderer.onTouchEvent(event);
	}

	private void setUpExtrinsics() {
		// Set device to imu matrix in Model Matrix Calculator.
		TangoPoseData device2IMUPose = new TangoPoseData();
		TangoCoordinateFramePair framePair = new TangoCoordinateFramePair();
		framePair.baseFrame = TangoPoseData.COORDINATE_FRAME_IMU;
		framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_DEVICE;
		try {
			device2IMUPose = mTango.getPoseAtTime(0.0, framePair);
		} catch (TangoErrorException e) {
			Toast.makeText(getApplicationContext(), R.string.TangoError,
					Toast.LENGTH_SHORT).show();
		}
		mRenderer.getModelMatCalculator().SetDevice2IMUMatrix(
				device2IMUPose.getTranslationAsFloats(),
				device2IMUPose.getRotationAsFloats());

		// Set color camera to imu matrix in Model Matrix Calculator.
		TangoPoseData color2IMUPose = new TangoPoseData();

		framePair.baseFrame = TangoPoseData.COORDINATE_FRAME_IMU;
		framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_CAMERA_COLOR;
		try {
			color2IMUPose = mTango.getPoseAtTime(0.0, framePair);
		} catch (TangoErrorException e) {
			Toast.makeText(getApplicationContext(), R.string.TangoError,
					Toast.LENGTH_SHORT).show();
		}
		mRenderer.getModelMatCalculator().SetColorCamera2IMUMatrix(
				color2IMUPose.getTranslationAsFloats(),
				color2IMUPose.getRotationAsFloats());
	}

	private void setTangoListeners() {
		// Configure the Tango coordinate frame pair
		final ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();
		framePairs.add(new TangoCoordinateFramePair(
				TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
				TangoPoseData.COORDINATE_FRAME_DEVICE));
		// Listen for new Tango data
		mTango.connectListener(framePairs, new OnTangoUpdateListener() {

			@Override
			public void onPoseAvailable(final TangoPoseData pose) {
				// Make sure to have atomic access to Tango Pose Data so that
				// render loop doesn't interfere while Pose call back is
				// updating
				// the data.
				synchronized (poseLock) {
					mPose = pose;
					// Calculate the delta time from previous pose.
					mDeltaTime = (float) (pose.timestamp - mPosePreviousTimeStamp)
							* SECS_TO_MILLISECS;
					mPosePreviousTimeStamp = (float) pose.timestamp;
					if (mPreviousPoseStatus != pose.statusCode) {
						count = 0;
					}
					count++;
					mPreviousPoseStatus = pose.statusCode;
					if (!mRenderer.isValid()) {
						return;
					}
					mRenderer.getModelMatCalculator().updateModelMatrix(
							pose.getTranslationAsFloats(),
							pose.getRotationAsFloats());
					mRenderer.updateViewMatrix();
				}
			}

			@Override
			public void onXyzIjAvailable(final TangoXyzIjData xyzIj) {
				// Make sure to have atomic access to TangoXyzIjData so that
				// render loop doesn't interfere while onXYZijAvailable callback
				// is updating
				// the point cloud data.

				synchronized (depthLock) {
					mCurrentTimeStamp = (float) xyzIj.timestamp;
					mPointCloudFrameDelta = (mCurrentTimeStamp - mXyIjPreviousTimeStamp)
							* SECS_TO_MILLISECS;
					mXyIjPreviousTimeStamp = mCurrentTimeStamp;
					try {
						TangoPoseData pointCloudPose = mTango.getPoseAtTime(
								mCurrentTimeStamp, framePairs.get(0));
						mPointCount = xyzIj.xyzCount;
						if (!mRenderer.isValid()) {
							return;
						}
						if ((pointCloudPose.statusCode == TangoPoseData.POSE_VALID)) {
							if (mTimeToTakePointCloud) {
								mTimeToTakePointCloud = false;
								writeCloudAndPoseToFile(xyzIj, pointCloudPose,
										mNumPoseCount);
							}
						}

						mRenderer.getPointCloud().UpdatePoints(xyzIj.xyz);
						mRenderer
								.getModelMatCalculator()
								.updatePointCloudModelMatrix(
										pointCloudPose.getTranslationAsFloats(),
										pointCloudPose.getRotationAsFloats());
						mRenderer.getPointCloud().setModelMatrix(
								mRenderer.getModelMatCalculator()
										.getPointCloudModelMatrixCopy());
					} catch (TangoErrorException e) {
						Toast.makeText(getApplicationContext(),
								R.string.TangoError, Toast.LENGTH_SHORT).show();
					} catch (TangoInvalidException e) {
						Toast.makeText(getApplicationContext(),
								R.string.TangoError, Toast.LENGTH_SHORT).show();
					}
				}
			}

			@Override
			public void onTangoEvent(final TangoEvent event) {
				runOnUiThread(new Runnable() {
					@Override
					public void run() {
						mTangoEventTextView.setText(event.eventKey + ": "
								+ event.eventValue);
					}
				});
			}

			@Override
			public void onFrameAvailable(int cameraId) {
				// We are not using onFrameAvailable for this application.
			}
		});
	}

	protected void writeCloudAndPoseToFile(final TangoXyzIjData xyzIj,
			final TangoPoseData pointCloudPoseSS2Dev, final int poseCount) {

		// Background task for writing to file
		class SendCommandTask extends AsyncTask<Void, Void, Boolean> {

			@Override
			protected Boolean doInBackground(Void... params) {

				try {
					mutex_on_mIsRecording.acquire();
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				try {
					createSavingDirectory();
					writePointCloudToFile(xyzIj, poseCount);
					writeSS2DevicePoseToFile(pointCloudPoseSS2Dev, poseCount);
					mutex_on_mIsRecording.release();
				} catch (Exception e) {
					mutex_on_mIsRecording.release();
					return false;
				}
				mutex_on_mIsRecording.release();
				return true;
			}

			@Override
			protected void onPostExecute(Boolean done) {
				
				if (done) {
					int numberPlanes = getCountPlanesByPCL(mLastPointCloudFilename);
					String result=getResources().getString(R.string.found_planes)+Integer.toString(numberPlanes);
					Toast.makeText(getApplicationContext(), result,
							Toast.LENGTH_SHORT).show();

				} else {
					Toast.makeText(getApplicationContext(),
							R.string.failed_to_save + " :" + mFilenameSS2DPose+" and "+mLastPointCloudFilename,
							Toast.LENGTH_SHORT).show();
				}
			}
		}
		new SendCommandTask().execute();

	}

	// Write POSE SS2Device
	private boolean writeSS2DevicePoseToFile(
			final TangoPoseData pointCloudPoseSS2Dev,
			final int poseCount) {
		float accuracy = pointCloudPoseSS2Dev.accuracy;
		int confidence = pointCloudPoseSS2Dev.confidence;
		DecimalFormat threeDec = new DecimalFormat("0.000");
		String translationStringADF2Dev = threeDec
				.format(pointCloudPoseSS2Dev.translation[0])
				+ " "
				+ threeDec.format(pointCloudPoseSS2Dev.translation[1])
				+ " "
				+ threeDec.format(pointCloudPoseSS2Dev.translation[2]) + " ";
		String quaternionStringADF2Dev = threeDec
				.format(pointCloudPoseSS2Dev.rotation[0])
				+ " "
				+ threeDec.format(pointCloudPoseSS2Dev.rotation[1])
				+ " "
				+ threeDec.format(pointCloudPoseSS2Dev.rotation[2])
				+ " "
				+ threeDec.format(pointCloudPoseSS2Dev.rotation[3]) + " ";

		String confidenceString = Integer.toString(confidence) + " ";
		String accuracyString = Float.toString(accuracy) + " "; // oneDec.format(confidence);
		String cloudIndexString = Integer.toString(poseCount) + " ";
	

		// Save pose to file
		try {

			File poseFile = new File(mFilenameSS2DPose);

			BufferedWriter bW;

			bW = new BufferedWriter(new FileWriter(poseFile, true));
			bW.write(cloudIndexString);
			bW.write(translationStringADF2Dev);
			bW.write(quaternionStringADF2Dev);
			bW.write(accuracyString);
			bW.write(confidenceString);
			bW.newLine();
			bW.flush();
			bW.close();
			return true;

		} catch (IOException e) {
			e.printStackTrace();

		} catch (TangoErrorException e) {

		}
		return true;
	}

	// This function writes the XYZ points to pcd files in binary
	private void writePointCloudToFile(TangoXyzIjData xyzIj, int poseNumber) {

		// XXXXXXXXXXXXXXXXXXXApi changes
		File dir = new File(SaveDir);
		mLastPointCloudFilename = String.format("%06d", poseNumber) + "-cloud.pcd";

		File file = new File(dir, mLastPointCloudFilename);

		final byte[] buffer = new byte[xyzIj.xyzCount * 3 * 4];
		FileInputStream fileStream = new FileInputStream(
				xyzIj.xyzParcelFileDescriptor.getFileDescriptor());
		try {
			fileStream.read(buffer, xyzIj.xyzParcelFileDescriptorOffset,
					buffer.length);
			fileStream.close();
		} catch (IOException e) {
			e.printStackTrace();
		}

		ByteBuffer myBuffer = ByteBuffer.allocate(xyzIj.xyzCount * 3 * 4);
		myBuffer.order(ByteOrder.BIG_ENDIAN);
		myBuffer.put(buffer, xyzIj.xyzParcelFileDescriptorOffset,
				myBuffer.capacity());

		// xyzIj.xyz=myBuffer.asFloatBuffer();

		try {

			DataOutputStream out = new DataOutputStream(
					new BufferedOutputStream(new FileOutputStream(file)));

			out.write(("# .PCD v0.7 - Point Cloud Data file format\n"
					+ "VERSION 0.7\n" + "FIELDS x y z\n" + "SIZE 4 4 4\n"
					+ "TYPE F F F\n" + "COUNT 1 1 1\n" + "WIDTH "
					+ xyzIj.xyzCount + "\n" + "HEIGHT 1\n"
					+ "VIEWPOINT 0 0 0 1 0 0 0\n" + "POINTS " + xyzIj.xyzCount
					+ "\n" + "DATA binary\n").getBytes());
			Log.d(TAG, "Anzahl an Punkten" + Integer.toString(xyzIj.xyzCount));

			for (int i = 0; i < xyzIj.xyzCount; i++) {
				// Log.d(TAG, Float.toString(xyzIj.xyz.get((3 * i ))));
				out.writeFloat(myBuffer.getFloat((3 * i) * 4));
				out.writeFloat(myBuffer.getFloat((3 * i + 1) * 4));
				out.writeFloat(myBuffer.getFloat((3 * i + 2) * 4));
			}
			// FindPlanesRANSACPC(myBuffer);
			out.close();
			mNumPoseCount++;
			

		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void createSavingDirectory() {
		File mainDir = new File(MainDir);
		if (!mainDir.exists()) {
			boolean created = mainDir.mkdir();
			if (created) {
				Log.i(TAG, "Folder: \"" + MainDir + "\" created\n");
			}
		}
		File dir = new File(SaveDir);
		if (!dir.exists()) {
			boolean created = dir.mkdir();
			if (created) {
				Log.i(TAG, "Folder: \"" + SaveDir + "\" created\n");
			}
		}

	}

	/**
	 * Create a separate thread to update Log information on UI at the specified
	 * interval of UPDATE_INTERVAL_MS. This function also makes sure to have
	 * access to the mPose atomically.
	 */
	private void startUIThread() {
		new Thread(new Runnable() {
			final DecimalFormat threeDec = new DecimalFormat("0.000");

			@Override
			public void run() {
				while (true) {
					try {
						Thread.sleep(UPDATE_INTERVAL_MS);
						// Update the UI with TangoPose information
						runOnUiThread(new Runnable() {
							@Override
							public void run() {
								synchronized (poseLock) {
									if (mPose == null) {
										return;
									}
									String translationString = "["
											+ threeDec
													.format(mPose.translation[0])
											+ ", "
											+ threeDec
													.format(mPose.translation[1])
											+ ", "
											+ threeDec
													.format(mPose.translation[2])
											+ "] ";
									String quaternionString = "["
											+ threeDec
													.format(mPose.rotation[0])
											+ ", "
											+ threeDec
													.format(mPose.rotation[1])
											+ ", "
											+ threeDec
													.format(mPose.rotation[2])
											+ ", "
											+ threeDec
													.format(mPose.rotation[3])
											+ "] ";

									// Display pose data on screen in TextViews
									mPoseTextView.setText(translationString);
									mQuatTextView.setText(quaternionString);
									mPoseCountTextView.setText(Integer
											.toString(count));
									mDeltaTextView.setText(threeDec
											.format(mDeltaTime));
									if (mPose.statusCode == TangoPoseData.POSE_VALID) {
										mPoseStatusTextView
												.setText(R.string.pose_valid);
									} else if (mPose.statusCode == TangoPoseData.POSE_INVALID) {
										mPoseStatusTextView
												.setText(R.string.pose_invalid);
									} else if (mPose.statusCode == TangoPoseData.POSE_INITIALIZING) {
										mPoseStatusTextView
												.setText(R.string.pose_initializing);
									} else if (mPose.statusCode == TangoPoseData.POSE_UNKNOWN) {
										mPoseStatusTextView
												.setText(R.string.pose_unknown);
									}
								}
								synchronized (depthLock) {
									// Display number of points in the point
									// cloud
									mPointCountTextView.setText(Integer
											.toString(mPointCount));
									mFrequencyTextView.setText(""
											+ threeDec
													.format(mPointCloudFrameDelta));
									mAverageZTextView.setText(""
											+ threeDec.format(mRenderer
													.getPointCloud()
													.getAverageZ()));
								}
							}
						});
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
		}).start();
	}

	public void onPCLBtnClicked(View v) {
		if (v.getId() == R.id.pclButton) {
			mTimeToTakePointCloud=true;
		}
	}

	static {
		System.loadLibrary("pcl_lib");
	}

	public native int greetingsFromPCL();
	public native int getCountPlanesByPCL(String filename);
}
