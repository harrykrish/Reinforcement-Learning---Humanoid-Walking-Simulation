using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO;
using Microsoft.Kinect;
using System.Windows.Media.Media3D;
using System.Diagnostics;

namespace PraxisProjectIuK
{
	public partial class MainWindow : Window
	{
		// Timer for the record start
		static System.Windows.Forms.Timer myTimer = new System.Windows.Forms.Timer();
		// bool variables for the record method
		private bool drawstart = true;
		private bool drawstop = false;
		private bool writefiles = false;
		// stopwatch for the time of each frame
		Stopwatch stopwatch = new Stopwatch();
		// all variables for the kinectskeleton.trc file
		#region OutputFile
		public string pathFileType { get; set; }
		public string top2 { get; set; }
		public string coordinateSystem { get; set; }
		public string filename { get; set; }

		public string dataRate { get; set; }
		public string cameraRate { get; set; }
		public string numFrames { get; set; }
		public string numMarkers { get; set; }
		public string units { get; set; }
		public string origDataRate { get; set; }
		public string origDataStartFrame { get; set; }
		public string origNumFrames { get; set; }

		public string dataRateValue { get; set; }
		public string cameraRateValue { get; set; }
		public string numFrameValue { get; set; }
		public string numMarkersValue { get; set; }
		public string unitsValue { get; set; }
		public string origDataRateValue { get; set; }
		public string origDataStartFrameValue { get; set; }
		public string origNumFramesValue { get; set; }

		public string frame { get; set; }
		public string time { get; set; }
		public string neck { get; set; }
		public string shoulder_C { get; set; }
		public string shoulderRight { get; set; }
		public string shoulderLeft { get; set; }
		public string elbowRight { get; set; }
		public string elbowLeft { get; set; }
		public string rightWrist { get; set; }
		public string leftWrist { get; set; }
		public string hipCenter { get; set; }
		public string hipRight { get; set; }
		public string hipLeft { get; set; }
		public string kneeRight { get; set; }
		public string kneeLeft { get; set; }
		public string ankleRight { get; set; }
		public string ankleLeft { get; set; }
		public string footRight { get; set; }
		public string footLeft { get; set; }

		public string frameNumber { get; set; }
		public int counter { get; set; }
		public double timechecker { get; set; }
		public string neckX { get; set; }
		public string neckY { get; set; }
		public string neckZ { get; set; }

		public string shoulderCX { get; set; }
		public string shoulderCY { get; set; }
		public string shoulderCZ { get; set; }

		public string shoulderRX { get; set; }
		public string shoulderRY { get; set; }
		public string shoulderRZ { get; set; }

		public string shoulderLX { get; set; }
		public string shoulderLY { get; set; }
		public string shoulderLZ { get; set; }

		public string rightElbowX { get; set; }
		public string rightElbowY { get; set; }
		public string rightElbowZ { get; set; }

		public string leftElbowX { get; set; }
		public string leftElbowY { get; set; }
		public string leftElbowZ { get; set; }

		public string rightWristX { get; set; }
		public string rightWristY { get; set; }
		public string rightWristZ { get; set; }

		public string leftWristX { get; set; }
		public string leftWristY { get; set; }
		public string leftWristZ { get; set; }

		public string HipPX { get; set; }
		public string HipPY { get; set; }
		public string HipPZ { get; set; }

		public string HipRightX { get; set; }
		public string HipRightY { get; set; }
		public string HipRightZ { get; set; }

		public string HipLeftX { get; set; }
		public string HipLeftY { get; set; }
		public string HipLeftZ { get; set; }

		public string KneeRightX { get; set; }
		public string KneeRightY { get; set; }
		public string KneeRightZ { get; set; }

		public string KneeLeftX { get; set; }
		public string KneeLeftY { get; set; }
		public string KneeLeftZ { get; set; }

		public string AnkleRightX { get; set; }
		public string AnkleRightY { get; set; }
		public string AnkleRightZ { get; set; }

		public string AnkleLeftX { get; set; }
		public string AnkleLeftY { get; set; }
		public string AnkleLeftZ { get; set; }

		public string FootRightX { get; set; }
		public string FootRightY { get; set; }
		public string FootRightZ { get; set; }

		public string FootLeftX { get; set; }
		public string FootLeftY { get; set; }
		public string FootLeftZ { get; set; }

		#endregion
		// bool variables for the instructions in the gui
		#region Bool instructions
		public bool beginning = true;
		public bool start = false;
		public bool recording = false;
		#endregion

		#region Color and body stream

		// kinectSensor for initialisiation later
		private KinectSensor kinectSensor = null;
		// Array with all tracked Bodys
		IList&lt;Body&gt; _bodies;
// All Data Streams
private MultiSourceFrameReader msfr;
		// DrawingGroup and Image for the bitmap creation
		private DrawingGroup drawingGroup;
		private DrawingImage imageSource;
		// CoordinateMapper for mapping later on
		private CoordinateMapper coordinateMapper = null;

		#endregion

		public MainWindow()
		{
			#region Formating Dots
			// Change from , to .
			System.Globalization.CultureInfo customCulture = (System.Globalization.CultureInfo)System.Threading.Thread.CurrentThread.CurrentCulture.Clone();
			customCulture.NumberFormat.NumberDecimalSeparator = ".";
			System.Threading.Thread.CurrentThread.CurrentCulture = customCulture;
			#endregion
			#region COLOR + SKELETON STREAM
			// Create new DrawingGroup
			drawingGroup = new DrawingGroup();
			// Create an image source that we can use in our image control
			imageSource = new DrawingImage(this.drawingGroup);
			// kinectSensor
			kinectSensor = KinectSensor.GetDefault();
			// Open SoureFrameReader with Color and Body Stream
			msfr = kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Body);
			// Start Eventhandler with this Color and Body Data
			msfr.MultiSourceFrameArrived += msfr_MultiSourceFrameArrived;
			// CoordinateMapper for this kinect for the later mapping of Dots for the Joint Position
			coordinateMapper = this.kinectSensor.CoordinateMapper;
			// Start kinectSensor
			kinectSensor.Open();
			// Initialize all
			InitializeComponent();
			#endregion
		}

		public void msfr_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
		{

			var reference = e.FrameReference.AcquireFrame();
			using (var frame = reference.ColorFrameReference.AcquireFrame())
			{
				if (frame != null)
				{
					camera.Source = frame.ToBitmap();
				}
			}
			using (var frame = reference.BodyFrameReference.AcquireFrame())
			{
				if (frame != null)
				{
					using (DrawingContext dc = this.drawingGroup.Open())
					{
						canvas.Children.Clear();
						_bodies = new Body[frame.BodyFrameSource.BodyCount];
						frame.GetAndRefreshBodyData(_bodies);
						foreach (var body in _bodies)
						{
							if (body.IsTracked)
							{
								// COORDINATE MAPPING
								foreach (Joint joint in body.Joints.Values)
								{
									if (joint.TrackingState == TrackingState.Tracked)
									{
										// 3D space point
										CameraSpacePoint jointPosition = joint.Position;
										// 2D space point
										Point point1 = new Point();

										ColorSpacePoint colorPoint = kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(jointPosition);
										point1.X = float.IsInfinity(colorPoint.X) ? 0 : colorPoint.X;
										point1.Y = float.IsInfinity(colorPoint.Y) ? 0 : colorPoint.Y;

										// Draw
										Ellipse ellipse = new Ellipse
										{
											Fill = Brushes.Red,
											Width = 30,
											Height = 30
										};
										Canvas.SetLeft(ellipse, point1.X - ellipse.Width / 2);
										Canvas.SetTop(ellipse, point1.Y - ellipse.Height / 2);
										canvas.Children.Add(ellipse);
									}
								}
							}
						}
					}
				}
			}

			using (var frame = reference.BodyFrameReference.AcquireFrame())
			{
				if (frame != null)
				{
					if (beginning == true & amp; &amp; start == false & amp; &amp; recording == false)
{
						var uri = new Uri("pack://siteoforigin:,,,/Images/Beginning.png");
						var bitmap = new BitmapImage(uri);
						instructions.Source = bitmap;
					}
					if (beginning == true & amp; &amp; start == true & amp; &amp; recording == false)
{
						var uri = new Uri("pack://siteoforigin:,,,/Images/Start.png");
						var bitmap = new BitmapImage(uri);
						instructions.Source = bitmap;
					}
					if (beginning == true & amp; &amp; start == true & amp; &amp; recording == true)
{
						var uri = new Uri("pack://siteoforigin:,,,/Images/Recordfenster.png");
						var bitmap = new BitmapImage(uri);
						instructions.Source = bitmap;
					}

					frame.GetAndRefreshBodyData(_bodies);

					if (writefiles)
					{
						System.Windows.Threading.DispatcherTimer dispatcherTimer = new System.Windows.Threading.DispatcherTimer();
						dispatcherTimer.Tick += new EventHandler(dispatcherTimer_Tick);
						dispatcherTimer.Interval = new TimeSpan(0, 0, 5);
						dispatcherTimer.Start();
					}
					foreach (var body in _bodies)
					{
						if (body.IsTracked)
						{
							foreach (Joint joint in body.Joints.Values)
							{

								if (joint.TrackingState == TrackingState.Tracked)
								{

									if (drawstart)
									{
										DrawStartArea(body, JointType.HipRight);
									}
									if (drawstop)
									{
										DrawStopArea(body, JointType.SpineShoulder);
									}
									bool startRecord = IsHandOverStart(body);
									bool stopRecord = IsHandOverStop(body);
									record(body);
									if (startRecord)
									{
										counter = 0;
										drawstart = false;
										drawstop = true;
										writefiles = true;
										writeTopping();
										stopwatch.Start();
										start = true;
									}
									if (stopRecord)
									{
										drawstart = true;
										drawstop = false;
										stopwatch.Stop();
										writefiles = false;
										var instance = new ChangeTrc();
										instance.changeExtensions();
										Application.Current.Shutdown();
										break;
									}
								}
							}
						}
					}
				}
			}
		}

		static void lineChanger(string newText, string fileName, int line_to_edit)
		{
			string[] arrLine = File.ReadAllLines(fileName);
			arrLine[line_to_edit - 1] = newText;
			File.WriteAllLines(fileName, arrLine);
		}

		private void writeTopping()
		{
			pathFileType = "PathFileType";
			top2 = "4";
			coordinateSystem = "(X/Y/Z)";
			filename = "kinectSkeleton.trc";

			dataRate = "DataRate";
			cameraRate = "CameraRate";
			numFrames = "NumFrames";
			numMarkers = "NumMarkers";
			units = "Units";
			origDataRate = "OrigDataRate";
			origDataStartFrame = "OrigDataStartFrame";
			origNumFrames = "OrigNumFrames";

			dataRateValue = "60.00";
			cameraRateValue = "60.00";

			numMarkersValue = "17";
			unitsValue = "mm";
			origDataRateValue = "60.00";
			origDataStartFrameValue = "1";
			origNumFramesValue = "300";
			frame = "#Frame";
			time = "Time";

			neck = "Neck";
			shoulder_C = "ShoulderMid";
			shoulderRight = "R.Shoulder";
			shoulderLeft = "L.Shoulder";
			elbowRight = "R.Elbow";
			elbowLeft = "L.Elbow";
			rightWrist = "R.Wrist";
			leftWrist = "L.Wrist";
			hipCenter = "HipCenter";
			hipRight = "R.Hip";
			hipLeft = "L.Hip";
			kneeRight = "R.Knee";
			kneeLeft = "L.Knee";
			ankleRight = "R.Ankle";
			ankleLeft = "L.Ankle";
			footRight = "R.Foot";
			footLeft = "L.Foot";

			System.IO.File.WriteAllText(@"Output/kinectSkeleton.txt", "");
			using (System.IO.StreamWriter file =
			new System.IO.StreamWriter(@"Output/kinectSkeleton.txt", true))
			{
				file.WriteLine(pathFileType + "\t" + top2 + "\t" + coordinateSystem + "\t" + filename);
				file.WriteLine(dataRate + "\t" + cameraRate + "\t" + numFrames + "\t" + numMarkers + "\t" + units + "\t" + origDataRate + "\t" + origDataStartFrame + "\t" + origNumFrames);
				file.WriteLine(dataRateValue + "\t" + cameraRateValue + "\t" + numFrameValue + "\t" + numMarkersValue + "\t" + unitsValue + "\t" + origDataRateValue + "\t" + origDataStartFrameValue + "\t" + origNumFramesValue);
				file.WriteLine(frame + "\t" + time + "\t" + neck + "\t" + "\t" + "\t" + shoulder_C + "\t" + "\t" + "\t" + shoulderRight + "\t" + "\t" + "\t" + shoulderLeft + "\t" + "\t" + "\t" + elbowRight + "\t" + "\t" + "\t" + elbowLeft
				+ "\t" + "\t" + "\t" + rightWrist + "\t" + "\t" + "\t" + leftWrist + "\t" + "\t" + "\t" + hipCenter + "\t" + "\t" + "\t" + hipRight + "\t" + "\t" + "\t" + hipLeft
				+ "\t" + "\t" + "\t" + kneeRight + "\t" + "\t" + "\t" + kneeLeft + "\t" + "\t" + "\t" + ankleRight + "\t" + "\t" + "\t" + ankleLeft + "\t" + "\t" + "\t" + footRight + "\t" + "\t" + "\t" + footLeft);
				file.WriteLine("\t" + "\t" + "X1" + "\t" + "Y1" + "\t" + "Z1" + "\t" + "X2" + "\t" + "Y2" + "\t" + "Z2" + "\t" + "X3" + "\t" + "Y3" + "\t" + "Z3" + "\t" + "X4" + "\t" + "Y4" + "\t" + "Z4" + "\t" + "X5" + "\t" + "Y5" + "\t" + "Z5" + "\t" + "X6" + "\t" + "Y6" + "\t" + "Z6" + "\t" + "X7" + "\t" + "Y7" + "\t" + "Z7" + "\t" + "X8" + "\t" + "Y8" + "\t" + "Z8" + "\t" + "X9" + "\t" + "Y9" + "\t" + "Z10" + "\t" + "X10" + "\t" + "Y10" + "\t" + "Z10" + "\t" + "X11" + "\t" + "Y11" + "\t" + "Z11" + "\t" + "X12" + "\t" + "Y12" + "\t" + "Z12" + "\t" + "X13" + "\t" + "Y13" + "\t" + "Z13" + "\t" + "X14" + "\t" + "Y14" + "\t" + "Z14" + "\t" + "X15" + "\t" + "Y15" + "\t" + "Z15");
				file.WriteLine("");
			}
		}

		private void dispatcherTimer_Tick(object sender, EventArgs e)
		{
			recording = true;
			var elapsedTime = stopwatch.ElapsedMilliseconds;
			double elapsedTimeFloat = Convert.ToDouble(elapsedTime);
			double elapsedTimeDoubleToSeconds = elapsedTimeFloat / 1000;
			string elapsedTimeOutput = string.Format("{0:0.00}", elapsedTimeDoubleToSeconds);
			int counterFinal = counter;
			counterFinal += 1;
			string counterToString = counterFinal.ToString();
			numFrameValue = counterToString;
			stopwatch.Start();

			if (elapsedTimeDoubleToSeconds != timechecker)
			{
				using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"Output/kinectSkeleton.txt", true))
				{

					counter++;
					file.WriteLine(counter + "\t" + elapsedTimeOutput + "0000" + "\t" + neckX + "\t" + neckY + "\t" + neckZ + "\t" + shoulderCX + "\t" + shoulderCY + "\t" + shoulderCZ + "\t" + shoulderRX + "\t" + shoulderRY + "\t" + shoulderRZ + "\t" + shoulderLX + "\t" + shoulderLY + "\t" + shoulderLZ +
					"\t" + rightElbowX + "\t" + rightElbowY + "\t" + rightElbowZ + "\t" + leftElbowX + "\t" + leftElbowY + "\t" + leftElbowZ + "\t" + rightWristX + "\t" + rightWristY + "\t" + rightWristZ + "\t" +
					leftWristX + "\t" + leftWristY + "\t" + leftWristZ + "\t" + HipPX + "\t" + HipPY + "\t" + HipPZ + "\t" + HipRightX + "\t" + HipRightY + "\t" + &nbsp; HipRightZ + "\t" + &nbsp; HipLeftX + "\t" + &nbsp; HipLeftY + "\t" + &nbsp; HipLeftZ
						+ "\t" + KneeRightX + "\t" + KneeRightY + "\t" + KneeRightZ + "\t" + KneeLeftX + "\t" + KneeLeftY + "\t" + KneeLeftZ + "\t" + AnkleRightX + "\t" + AnkleRightY + "\t" + AnkleRightZ + "\t" + AnkleLeftX + "\t" + AnkleLeftY + "\t" + AnkleLeftZ
						+ "\t" + FootRightX + "\t" + FootRightY + "\t" + FootRightZ + "\t" + FootLeftX + "\t" + FootLeftY + "\t" + FootLeftZ);
					timechecker = elapsedTimeDoubleToSeconds;

				}
				lineChanger(dataRateValue + "\t" + cameraRateValue + "\t" + numFrameValue + "\t" + numMarkersValue + "\t" + unitsValue + "\t" + origDataRateValue + "\t" + origDataStartFrameValue + "\t" + origNumFramesValue, @"Output/kinectSkeleton.txt", 3);
			}
		}

		private void record(Body body)
		{
			#region source
			Vector3D sourceP = new Vector3D(body.Joints[JointType.SpineBase].Position.X, body.Joints[JointType.FootRight].Position.Y, body.Joints[JointType.SpineBase].Position.Z);
			#endregion
			#region neck
			var neckPX = body.Joints[JointType.Neck].Position.X - sourceP.X;
			var neckPY = body.Joints[JointType.Neck].Position.Y - sourceP.Y;
			var neckPZ = body.Joints[JointType.Neck].Position.Z - sourceP.Z;

			var rotateNeckPX = neckPX * Math.Cos(-90) + neckPZ * Math.Sin(-90);
			var rotateNeckPY = neckPY;
			var rotateNeckPZ = neckPX * -Math.Sin(-90) + neckPZ * Math.Cos(-90);

			double neckXToDouble = Convert.ToDouble(rotateNeckPX);
			double neckYYToDouble = Convert.ToDouble(rotateNeckPY);
			double neckZToDouble = Convert.ToDouble(rotateNeckPZ);

			double neckXToMillimeters = neckXToDouble * 1000;
			double neckYToMillimeters = neckYYToDouble * 1000;
			double neckZToMillimeters = neckZToDouble * 1000;

			neckXToMillimeters = Math.Round(neckXToMillimeters, 6, MidpointRounding.ToEven);
			neckYToMillimeters = Math.Round(neckYToMillimeters, 6, MidpointRounding.ToEven);
			neckZToMillimeters = Math.Round(neckZToMillimeters, 6, MidpointRounding.ToEven);

			neckX = string.Format("{0:0.000000}", neckXToMillimeters);
			neckY = string.Format("{0:0.000000}", neckYToMillimeters);
			neckZ = string.Format("{0:0.000000}", neckZToMillimeters);

			#endregion
			#region shoulderC
			var shoulderCenterX = body.Joints[JointType.SpineShoulder].Position.X - sourceP.X;
			var shoulderCenterY = body.Joints[JointType.SpineShoulder].Position.Y - sourceP.Y;
			var shoulderCenterZ = body.Joints[JointType.SpineShoulder].Position.Z - sourceP.Z;

			var rotateshoulderCenterX = shoulderCenterX * Math.Cos(-90) + shoulderCenterZ * Math.Sin(-90);
			var rotateshoulderCenterY = shoulderCenterY;
			var rotateshoulderCenterZ = shoulderCenterX * -Math.Sin(-90) + shoulderCenterZ * Math.Cos(-90);

			double shoulderCenterXToDouble = Convert.ToDouble(rotateshoulderCenterX);
			double shoulderCenterYToDouble = Convert.ToDouble(rotateshoulderCenterY);
			double shoulderCenterZToDouble = Convert.ToDouble(rotateshoulderCenterZ);

			double shoulderCenterXToMillimeters = shoulderCenterXToDouble * 1000;
			double shoulderCenterYToMillimeters = shoulderCenterYToDouble * 1000;
			double shoulderCenterZToMillimeters = shoulderCenterZToDouble * 1000;

			shoulderCenterXToMillimeters = Math.Round(shoulderCenterXToMillimeters, 4, MidpointRounding.ToEven);
			shoulderCenterYToMillimeters = Math.Round(shoulderCenterYToMillimeters, 4, MidpointRounding.ToEven);
			shoulderCenterZToMillimeters = Math.Round(shoulderCenterZToMillimeters, 4, MidpointRounding.ToEven);

			shoulderCX = shoulderCenterXToMillimeters.ToString();
			shoulderCY = shoulderCenterYToMillimeters.ToString();
			shoulderCZ = shoulderCenterZToMillimeters.ToString();
			#endregion
			#region shoulderR

			var shoulderRightX = body.Joints[JointType.ShoulderRight].Position.X - sourceP.X;
			var shoulderRightY = body.Joints[JointType.ShoulderRight].Position.Y - sourceP.Y;
			var shoulderRightZ = body.Joints[JointType.ShoulderRight].Position.Z - sourceP.Z;

			var rotateshoulderRightX = shoulderRightX * Math.Cos(-90) + shoulderRightZ * Math.Sin(-90);
			var rotateshoulderRightY = shoulderRightY;
			var rotateshoulderRightZ = shoulderRightX * -Math.Sin(-90) + shoulderRightZ * Math.Cos(-90);

			double shoulderRightXToDouble = Convert.ToDouble(rotateshoulderRightX);
			double shoulderRightYToDouble = Convert.ToDouble(rotateshoulderRightY);
			double shoulderRightZToDouble = Convert.ToDouble(rotateshoulderRightZ);

			double shoulderRightXToMillimeters = shoulderRightXToDouble * 1000;
			double shoulderRightYToMillimeters = shoulderRightYToDouble * 1000;
			double shoulderRightZToMillimeters = shoulderRightZToDouble * 1000;

			shoulderRightXToMillimeters = Math.Round(shoulderRightXToMillimeters, 4, MidpointRounding.ToEven);
			shoulderRightYToMillimeters = Math.Round(shoulderRightYToMillimeters, 4, MidpointRounding.ToEven);
			shoulderRightZToMillimeters = Math.Round(shoulderRightZToMillimeters, 4, MidpointRounding.ToEven);

			shoulderRX = shoulderRightXToMillimeters.ToString();
			shoulderRY = shoulderRightYToMillimeters.ToString();
			shoulderRZ = shoulderRightZToMillimeters.ToString();

			#endregion
			#region shoulderL
			var shoulderLeftX = body.Joints[JointType.ShoulderLeft].Position.X - sourceP.X;
			var shoulderLeftY = body.Joints[JointType.ShoulderLeft].Position.Y - sourceP.Y;
			var shoulderLeftZ = body.Joints[JointType.ShoulderLeft].Position.Z - sourceP.Z;

			var rotateshoulderLeftX = shoulderLeftX * Math.Cos(-90) + shoulderLeftZ * Math.Sin(-90);
			var rotateshoulderLeftY = shoulderLeftY;
			var rotateshoulderLeftZ = shoulderLeftX * -Math.Sin(-90) + shoulderLeftZ * Math.Cos(-90);

			double shoulderLeftXToDouble = Convert.ToDouble(rotateshoulderLeftX);
			double shoulderLeftYToDouble = Convert.ToDouble(rotateshoulderLeftY);
			double shoulderLeftZToDouble = Convert.ToDouble(rotateshoulderLeftZ);

			double shoulderLeftXToMillimeters = shoulderLeftXToDouble * 1000;
			double shoulderLeftYToMillimeters = shoulderLeftYToDouble * 1000;
			double shoulderLeftZToMillimeters = shoulderLeftZToDouble * 1000;

			shoulderLeftXToMillimeters = Math.Round(shoulderLeftXToMillimeters, 4, MidpointRounding.ToEven);
			shoulderLeftYToMillimeters = Math.Round(shoulderLeftYToMillimeters, 4, MidpointRounding.ToEven);
			shoulderLeftZToMillimeters = Math.Round(shoulderLeftZToMillimeters, 4, MidpointRounding.ToEven);

			shoulderLX = shoulderLeftXToMillimeters.ToString();
			shoulderLY = shoulderLeftYToMillimeters.ToString();
			shoulderLZ = shoulderLeftZToMillimeters.ToString();
			#endregion
			#region elbowR

			var elbowRightX = body.Joints[JointType.ElbowRight].Position.X - sourceP.X;
			var elbowRightY = body.Joints[JointType.ElbowRight].Position.Y - sourceP.Y;
			var elbowRightZ = body.Joints[JointType.ElbowRight].Position.Z - sourceP.Z;

			var rotateelbowRightX = elbowRightX * Math.Cos(-90) + elbowRightZ * Math.Sin(-90);
			var rotateelbowRightY = elbowRightY;
			var rotateelbowRightZ = elbowRightX * -Math.Sin(-90) + elbowRightZ * Math.Cos(-90);

			double elbowRightXToDouble = Convert.ToDouble(rotateelbowRightX);
			double elbowRightYToDouble = Convert.ToDouble(rotateelbowRightY);
			double elbowRightZToDouble = Convert.ToDouble(rotateelbowRightZ);

			double elbowRightXToMillimeters = elbowRightXToDouble * 1000;
			double elbowRightYToMillimeters = elbowRightYToDouble * 1000;
			double elbowRightZToMillimeters = elbowRightZToDouble * 1000;

			elbowRightXToMillimeters = Math.Round(elbowRightXToMillimeters, 4, MidpointRounding.ToEven);
			elbowRightYToMillimeters = Math.Round(elbowRightYToMillimeters, 4, MidpointRounding.ToEven);
			elbowRightZToMillimeters = Math.Round(elbowRightZToMillimeters, 4, MidpointRounding.ToEven);

			rightElbowX = elbowRightXToMillimeters.ToString();
			rightElbowY = elbowRightYToMillimeters.ToString();
			rightElbowZ = elbowRightZToMillimeters.ToString();

			#endregion
			#region elbowL
			var elbowLeftX = body.Joints[JointType.ElbowLeft].Position.X - sourceP.X;
			var elbowLeftY = body.Joints[JointType.ElbowLeft].Position.Y - sourceP.Y;
			var elbowLeftZ = body.Joints[JointType.ElbowLeft].Position.Z - sourceP.Z;

			var rotateelbowLeftX = elbowLeftX * Math.Cos(-90) + elbowLeftZ * Math.Sin(-90);
			var rotateelbowLeftY = elbowLeftY;
			var rotateelbowLeftZ = elbowLeftX * -Math.Sin(-90) + elbowLeftZ * Math.Cos(-90);

			double elbowLeftXToDouble = Convert.ToDouble(rotateelbowLeftX);
			double elbowLeftYToDouble = Convert.ToDouble(rotateelbowLeftY);
			double elbowLeftZToDouble = Convert.ToDouble(rotateelbowLeftZ);

			double elbowLeftXToMillimeters = elbowLeftXToDouble * 1000;
			double elbowLeftYToMillimeters = elbowLeftYToDouble * 1000;
			double elbowLeftZToMillimeters = elbowLeftZToDouble * 1000;

			elbowLeftXToMillimeters = Math.Round(elbowLeftXToMillimeters, 4, MidpointRounding.ToEven);
			elbowLeftYToMillimeters = Math.Round(elbowLeftYToMillimeters, 4, MidpointRounding.ToEven);
			elbowLeftZToMillimeters = Math.Round(elbowLeftZToMillimeters, 4, MidpointRounding.ToEven);

			leftElbowX = elbowLeftXToMillimeters.ToString();
			leftElbowY = elbowLeftYToMillimeters.ToString();
			leftElbowZ = elbowLeftZToMillimeters.ToString();
			#endregion
			#region wristR
			var ankleRightX = body.Joints[JointType.HandRight].Position.X - sourceP.X;
			var ankleRightY = body.Joints[JointType.HandRight].Position.Y - sourceP.Y;
			var ankleRightZ = body.Joints[JointType.HandRight].Position.Z - sourceP.Z;

			var rotateankleRightX = ankleRightX * Math.Cos(-90) + ankleRightZ * Math.Sin(-90);
			var rotateankleRightY = ankleRightY;
			var rotateankleRightZ = ankleRightX * -Math.Sin(-90) + ankleRightZ * Math.Cos(-90);

			double ankleRightXToDouble = Convert.ToDouble(rotateankleRightX);
			double ankleRightYToDouble = Convert.ToDouble(rotateankleRightY);
			double ankleRightZToDouble = Convert.ToDouble(rotateankleRightZ);

			double ankleRightXToMillimeters = ankleRightXToDouble * 1000;
			double ankleRightYToMillimeters = ankleRightYToDouble * 1000;
			double ankleRightZToMillimeters = ankleRightZToDouble * 1000;

			ankleRightXToMillimeters = Math.Round(ankleRightXToMillimeters, 4, MidpointRounding.ToEven);
			ankleRightYToMillimeters = Math.Round(ankleRightYToMillimeters, 4, MidpointRounding.ToEven);
			ankleRightZToMillimeters = Math.Round(ankleRightZToMillimeters, 4, MidpointRounding.ToEven);

			rightWristX = ankleRightXToMillimeters.ToString();
			rightWristY = ankleRightYToMillimeters.ToString();
			rightWristZ = ankleRightZToMillimeters.ToString();
			#endregion
			#region wristL
			var ankleLeftX = body.Joints[JointType.HandLeft].Position.X - sourceP.X;
			var ankleLeftY = body.Joints[JointType.HandLeft].Position.Y - sourceP.Y;
			var ankleLeftZ = body.Joints[JointType.HandLeft].Position.Z - sourceP.Z;

			var rotateankleLeftX = ankleLeftX * Math.Cos(-90) + ankleLeftZ * Math.Sin(-90);
			var rotateankleLeftY = ankleLeftY;
			var rotateankleLeftZ = ankleLeftX * -Math.Sin(-90) + ankleLeftZ * Math.Cos(-90);

			double ankleLeftXToDouble = Convert.ToDouble(rotateankleLeftX);
			double ankleLeftYToDouble = Convert.ToDouble(rotateankleLeftY);
			double ankleLeftZToDouble = Convert.ToDouble(rotateankleLeftZ);

			double ankleLeftXToMillimeters = ankleLeftXToDouble * 1000;
			double ankleLeftYToMillimeters = ankleLeftYToDouble * 1000;
			double ankleLeftZToMillimeters = ankleLeftZToDouble * 1000;

			ankleLeftXToMillimeters = Math.Round(ankleLeftXToMillimeters, 4, MidpointRounding.ToEven);
			ankleLeftYToMillimeters = Math.Round(ankleLeftYToMillimeters, 4, MidpointRounding.ToEven);
			ankleLeftZToMillimeters = Math.Round(ankleLeftZToMillimeters, 4, MidpointRounding.ToEven);

			leftWristX = ankleLeftXToMillimeters.ToString();
			leftWristY = ankleLeftYToMillimeters.ToString();
			leftWristZ = ankleLeftZToMillimeters.ToString();
			#endregion
			#region HipC
			var HipCPX = body.Joints[JointType.SpineBase].Position.X - sourceP.X;
			var HipCPY = body.Joints[JointType.SpineBase].Position.Y - sourceP.Y;
			var HipCPZ = body.Joints[JointType.SpineBase].Position.Z - sourceP.Z;

			var rotateHipCPX = HipCPX * Math.Cos(-90) + HipCPZ * Math.Sin(-90);
			var rotateHipCPY = HipCPY;
			var rotateHipCPZ = HipCPX * -Math.Sin(-90) + HipCPZ * Math.Cos(-90);

			double HipCPXToDouble = Convert.ToDouble(rotateHipCPX);
			double HipCPYToDouble = Convert.ToDouble(rotateHipCPY);
			double HipCPZToDouble = Convert.ToDouble(rotateHipCPZ);

			double HipCPXToMillimeters = HipCPXToDouble * 1000;
			double HipCPYToMillimeters = HipCPYToDouble * 1000;
			double HipCPZToMillimeters = HipCPZToDouble * 1000;

			HipCPXToMillimeters = Math.Round(HipCPXToMillimeters, 4, MidpointRounding.ToEven);
			HipCPYToMillimeters = Math.Round(HipCPYToMillimeters, 4, MidpointRounding.ToEven);
			HipCPZToMillimeters = Math.Round(HipCPZToMillimeters, 4, MidpointRounding.ToEven);

			HipPX = string.Format("{0:0.000000}", HipCPXToMillimeters);
			HipPY = string.Format("{0:0.000000}", HipCPYToMillimeters);
			HipPZ = string.Format("{0:0.000000}", HipCPZToMillimeters);
			#endregion
			#region HipR

			var HipRX = body.Joints[JointType.HipRight].Position.X - sourceP.X;
			var HipRY = body.Joints[JointType.HipRight].Position.Y - sourceP.Y;
			var HipRZ = body.Joints[JointType.HipRight].Position.Z - sourceP.Z;

			var rotateHipRX = HipRX * Math.Cos(-90) + HipRZ * Math.Sin(-90);
			var rotateHipRY = HipRY;
			var rotateHipRZ = HipRX * -Math.Sin(-90) + HipRZ * Math.Cos(-90);

			double HipRXToDouble = Convert.ToDouble(rotateHipRX);
			double HipRYToDouble = Convert.ToDouble(rotateHipRY);
			double HipRZToDouble = Convert.ToDouble(rotateHipRZ);

			double HipRXToMillimeters = HipRXToDouble * 1000;
			double HipRYToMillimeters = HipRYToDouble * 1000;
			double HipRZToMillimeters = HipRZToDouble * 1000;

			HipRXToMillimeters = Math.Round(HipRXToMillimeters, 4, MidpointRounding.ToEven);
			HipRYToMillimeters = Math.Round(HipRYToMillimeters, 4, MidpointRounding.ToEven);
			HipRZToMillimeters = Math.Round(HipRZToMillimeters, 4, MidpointRounding.ToEven);

			HipRightX = HipRXToMillimeters.ToString();
			HipRightY = HipRYToMillimeters.ToString();
			HipRightZ = HipRZToMillimeters.ToString();
			#endregion
			#region HipL
			var HipLX = body.Joints[JointType.HipLeft].Position.X - sourceP.X;
			var HipLY = body.Joints[JointType.HipLeft].Position.Y - sourceP.Y;
			var HipLZ = body.Joints[JointType.HipLeft].Position.Z - sourceP.Z;

			var rotateHipLX = HipLX * Math.Cos(-90) + HipLZ * Math.Sin(-90);
			var rotateHipLY = HipLY;
			var rotateHipLZ = HipLX * -Math.Sin(-90) + HipLZ * Math.Cos(-90);

			double HipLXToDouble = Convert.ToDouble(rotateHipLX);
			double HipLYToDouble = Convert.ToDouble(rotateHipLY);
			double HipLZToDouble = Convert.ToDouble(rotateHipLZ);

			double HipLXToMillimeters = HipLXToDouble * 1000;
			double HipLYToMillimeters = HipLYToDouble * 1000;
			double HipLZToMillimeters = HipLZToDouble * 1000;

			HipLXToMillimeters = Math.Round(HipLXToMillimeters, 4, MidpointRounding.ToEven);
			HipLYToMillimeters = Math.Round(HipLYToMillimeters, 4, MidpointRounding.ToEven);
			HipLZToMillimeters = Math.Round(HipLZToMillimeters, 4, MidpointRounding.ToEven);

			HipLeftX = HipLXToMillimeters.ToString();
			HipLeftY = HipLYToMillimeters.ToString();
			HipLeftZ = HipLZToMillimeters.ToString();
			#endregion
			#region kneeR
			var KneeRX = body.Joints[JointType.KneeRight].Position.X - sourceP.X;
			var KneeRY = body.Joints[JointType.KneeRight].Position.Y - sourceP.Y;
			var KneeRZ = body.Joints[JointType.KneeRight].Position.Z - sourceP.Z;

			var rotateKneeRX = KneeRX * Math.Cos(-90) + KneeRZ * Math.Sin(-90);
			var rotateKneeRY = KneeRY;
			var rotateKneeRZ = KneeRX * -Math.Sin(-90) + KneeRZ * Math.Cos(-90);

			double KneeRXToDouble = Convert.ToDouble(rotateKneeRX);
			double KneeRYToDouble = Convert.ToDouble(rotateKneeRY);
			double KneeRZToDouble = Convert.ToDouble(rotateKneeRZ);

			double KneeRXToMillimeters = KneeRXToDouble * 1000;
			double KneeRYToMillimeters = KneeRYToDouble * 1000;
			double KneeRZToMillimeters = KneeRZToDouble * 1000;

			KneeRXToMillimeters = Math.Round(KneeRXToMillimeters, 4, MidpointRounding.ToEven);
			KneeRYToMillimeters = Math.Round(KneeRYToMillimeters, 4, MidpointRounding.ToEven);
			KneeRZToMillimeters = Math.Round(KneeRZToMillimeters, 4, MidpointRounding.ToEven);

			KneeRightX = KneeRXToMillimeters.ToString();
			KneeRightY = KneeRYToMillimeters.ToString();
			KneeRightZ = KneeRZToMillimeters.ToString();
			#endregion
			#region kneeL
			var KneeLX = body.Joints[JointType.KneeLeft].Position.X - sourceP.X;
			var KneeLY = body.Joints[JointType.KneeLeft].Position.Y - sourceP.Y;
			var KneeLZ = body.Joints[JointType.KneeLeft].Position.Z - sourceP.Z;

			var rotateKneeLX = KneeLX * Math.Cos(-90) + KneeLZ * Math.Sin(-90);
			var rotateKneeLY = KneeLY;
			var rotateKneeLZ = KneeLX * -Math.Sin(-90) + KneeLZ * Math.Cos(-90);

			double KneeLXToDouble = Convert.ToDouble(rotateKneeLX);
			double KneeLYToDouble = Convert.ToDouble(rotateKneeLY);
			double KneeLZToDouble = Convert.ToDouble(rotateKneeLZ);

			double KneeLXToMillimeters = KneeLXToDouble * 1000;
			double KneeLYToMillimeters = KneeLYToDouble * 1000;
			double KneeLZToMillimeters = KneeLZToDouble * 1000;

			KneeLXToMillimeters = Math.Round(KneeLXToMillimeters, 4, MidpointRounding.ToEven);
			KneeLYToMillimeters = Math.Round(KneeLYToMillimeters, 4, MidpointRounding.ToEven);
			KneeLZToMillimeters = Math.Round(KneeLZToMillimeters, 4, MidpointRounding.ToEven);

			KneeLeftX = KneeLXToMillimeters.ToString();
			KneeLeftY = KneeLYToMillimeters.ToString();
			KneeLeftZ = KneeLZToMillimeters.ToString();
			#endregion
			#region ankleR
			var AnkleRX = body.Joints[JointType.AnkleRight].Position.X - sourceP.X;
			var AnkleRY = body.Joints[JointType.AnkleRight].Position.Y - sourceP.Y;
			var AnkleRZ = body.Joints[JointType.AnkleRight].Position.Z - sourceP.Z;

			var rotateAnkleRX = AnkleRX * Math.Cos(-90) + AnkleRZ * Math.Sin(-90);
			var rotateAnkleRY = AnkleRY;
			var rotateAnkleRZ = AnkleRX * -Math.Sin(-90) + AnkleRZ * Math.Cos(-90);

			double AnkleRXToDouble = Convert.ToDouble(rotateAnkleRX);
			double AnkleRYToDouble = Convert.ToDouble(rotateAnkleRY);
			double AnkleRZToDouble = Convert.ToDouble(rotateAnkleRZ);

			double AnkleRXToMillimeters = AnkleRXToDouble * 1000;
			double AnkleRYToMillimeters = AnkleRYToDouble * 1000;
			double AnkleRZToMillimeters = AnkleRZToDouble * 1000;

			AnkleRXToMillimeters = Math.Round(AnkleRXToMillimeters, 4, MidpointRounding.ToEven);
			AnkleRYToMillimeters = Math.Round(AnkleRYToMillimeters, 4, MidpointRounding.ToEven);
			AnkleRZToMillimeters = Math.Round(AnkleRZToMillimeters, 4, MidpointRounding.ToEven);

			AnkleRightX = AnkleRXToMillimeters.ToString();
			AnkleRightY = AnkleRYToMillimeters.ToString();
			AnkleRightZ = AnkleRZToMillimeters.ToString();
			#endregion
			#region ankleL
			var AnkleLX = body.Joints[JointType.AnkleLeft].Position.X - sourceP.X;
			var AnkleLY = body.Joints[JointType.AnkleLeft].Position.Y - sourceP.Y;
			var AnkleLZ = body.Joints[JointType.AnkleLeft].Position.Z - sourceP.Z;

			var rotateAnkleLX = AnkleLX * Math.Cos(-90) + AnkleLZ * Math.Sin(-90);
			var rotateAnkleLY = AnkleLY;
			var rotateAnkleLZ = AnkleLX * -Math.Sin(-90) + AnkleLZ * Math.Cos(-90);

			double AnkleLXToDouble = Convert.ToDouble(rotateAnkleLX);
			double AnkleLYToDouble = Convert.ToDouble(rotateAnkleLY);
			double AnkleLZToDouble = Convert.ToDouble(rotateAnkleLZ);

			double AnkleLXToMillimeters = AnkleLXToDouble * 1000;
			double AnkleLYToMillimeters = AnkleLYToDouble * 1000;
			double AnkleLZToMillimeters = AnkleLZToDouble * 1000;

			AnkleLXToMillimeters = Math.Round(AnkleLXToMillimeters, 4, MidpointRounding.ToEven);
			AnkleLYToMillimeters = Math.Round(AnkleLYToMillimeters, 4, MidpointRounding.ToEven);
			AnkleLZToMillimeters = Math.Round(AnkleLZToMillimeters, 4, MidpointRounding.ToEven);

			AnkleLeftX = AnkleLXToMillimeters.ToString();
			AnkleLeftY = AnkleLYToMillimeters.ToString();
			AnkleLeftZ = AnkleLZToMillimeters.ToString();
			#endregion
			#region footR
			var FootRX = body.Joints[JointType.FootRight].Position.X - sourceP.X;
			var FootRY = body.Joints[JointType.FootRight].Position.Y - sourceP.Y;
			var FootRZ = body.Joints[JointType.FootRight].Position.Z - sourceP.Z;

			var rotateFootRX = FootRX * Math.Cos(-90) + FootRZ * Math.Sin(-90);
			var rotateFootRY = FootRY;
			var rotateFootRZ = FootRX * -Math.Sin(-90) + FootRZ * Math.Cos(-90);

			double FootRXToDouble = Convert.ToDouble(rotateFootRX);
			double FootRYToDouble = Convert.ToDouble(rotateFootRY);
			double FootRZToDouble = Convert.ToDouble(rotateFootRZ);

			double FootRXToMillimeters = FootRXToDouble * 1000;
			double FootRYToMillimeters = FootRYToDouble * 1000;
			double FootRZToMillimeters = FootRZToDouble * 1000;

			FootRXToMillimeters = Math.Round(FootRXToMillimeters, 4, MidpointRounding.ToEven);
			FootRYToMillimeters = Math.Round(FootRYToMillimeters, 4, MidpointRounding.ToEven);
			FootRZToMillimeters = Math.Round(FootRZToMillimeters, 4, MidpointRounding.ToEven);

			FootRightX = FootRXToMillimeters.ToString();
			FootRightY = FootRYToMillimeters.ToString();
			FootRightZ = FootRZToMillimeters.ToString();
			#endregion
			#region footL
			var FootLX = body.Joints[JointType.FootLeft].Position.X - sourceP.X;
			var FootLY = body.Joints[JointType.FootLeft].Position.Y - sourceP.Y;
			var FootLZ = body.Joints[JointType.FootLeft].Position.Z - sourceP.Z;

			var rotateFootLX = FootLX * Math.Cos(-90) + FootLZ * Math.Sin(-90);
			var rotateFootLY = FootLY;
			var rotateFootLZ = FootLX * -Math.Sin(-90) + FootLZ * Math.Cos(-90);

			double FootLXToDouble = Convert.ToDouble(rotateFootLX);
			double FootLYToDouble = Convert.ToDouble(rotateFootLY);
			double FootLZToDouble = Convert.ToDouble(rotateFootLZ);

			double FootLXToMillimeters = FootLXToDouble * 1000;
			double FootLYToMillimeters = FootLYToDouble * 1000;
			double FootLZToMillimeters = FootLZToDouble * 1000;

			FootLXToMillimeters = Math.Round(FootLXToMillimeters, 4, MidpointRounding.ToEven);
			FootLYToMillimeters = Math.Round(FootLYToMillimeters, 4, MidpointRounding.ToEven);
			FootLZToMillimeters = Math.Round(FootLZToMillimeters, 4, MidpointRounding.ToEven);

			FootLeftX = FootLXToMillimeters.ToString();
			FootLeftY = FootLYToMillimeters.ToString();
			FootLeftZ = FootLZToMillimeters.ToString();
			#endregion

		}

		private void DrawStopArea(Body body, JointType jointType)
		{
			Joint headjoint = body.Joints[JointType.SpineShoulder];
			CameraSpacePoint jointPosition = headjoint.Position;
			Point point2 = new Point();
			ColorSpacePoint colorPoint = kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(jointPosition);
			point2.X = float.IsInfinity(colorPoint.X) ? 0 : colorPoint.X;
			point2.Y = float.IsInfinity(colorPoint.Y) ? 0 : colorPoint.Y;
			Ellipse ellipse = new Ellipse
			{
				Stroke = Brushes.Green,
				StrokeThickness = 4,
				Width = 100,
				Height = 100,
			};
			Canvas.SetLeft(ellipse, (point2.X + 700) - ellipse.Width / 2);
			Canvas.SetTop(ellipse, (point2.Y) - ellipse.Height / 2);
			canvas.Children.Add(ellipse);
		}

		private void DrawStartArea(Body body, JointType jointType)
		{
			Joint spineMid = body.Joints[JointType.SpineMid];
			CameraSpacePoint startArea = spineMid.Position;
			Point point2 = new Point();
			ColorSpacePoint colorPoint = kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(startArea);
			point2.X = float.IsInfinity(colorPoint.X) ? 0 : colorPoint.X;
			point2.Y = float.IsInfinity(colorPoint.Y) ? 0 : colorPoint.Y;
			Ellipse ellipse = new Ellipse
			{
				Stroke = Brushes.Green,
				StrokeThickness = 4,
				Width = 100,
				Height = 100,
			};
			Canvas.SetLeft(ellipse, (point2.X) - ellipse.Width / 2);
			Canvas.SetTop(ellipse, (point2.Y) - ellipse.Height / 2);
			canvas.Children.Add(ellipse);
		}

		private bool IsHandOverStart(Body body)
		{
			var spineBaseX = body.Joints[JointType.SpineMid].Position.X;
			var spineBaseY = body.Joints[JointType.SpineMid].Position.Y;

			var handX = body.Joints[JointType.HandRight].Position.X;
			var handY = body.Joints[JointType.HandRight].Position.Y;

			double handXdouble = Convert.ToDouble(handX);
			double spineBaseXdouble = Convert.ToDouble(spineBaseX);
			double handYdouble = Convert.ToDouble(handY);
			double spineBaseYdouble = Convert.ToDouble(spineBaseY);

			double handXdoubletoeven = Math.Round(handXdouble, 1, MidpointRounding.ToEven); // Rounds to even
			double spineBaseXdoubletoeven = Math.Round(spineBaseXdouble, 1, MidpointRounding.ToEven); // Rounds to even
			double handYdoubletoeven = Math.Round(handYdouble, 1, MidpointRounding.ToEven); // Rounds to even
			double spineBaseYdoubletoeven = Math.Round(spineBaseYdouble, 1, MidpointRounding.ToEven); // Rounds to even

			bool isAshighAsSpineMid = handXdoubletoeven == spineBaseXdoubletoeven & amp; &amp; handYdoubletoeven == spineBaseYdoubletoeven;
			return isAshighAsSpineMid;
		}

		private bool IsHandOverStop(Body body)
		{
			var shoulderRightX = body.Joints[JointType.ShoulderRight].Position.X + 0.5;
			var shoulderRightY = body.Joints[JointType.ShoulderRight].Position.Y;

			var handX = body.Joints[JointType.HandRight].Position.X;
			var handY = body.Joints[JointType.HandRight].Position.Y;

			double handXdouble = Convert.ToDouble(handX);
			double shoulderRightXdouble = Convert.ToDouble(shoulderRightX);
			double handYdouble = Convert.ToDouble(handY);
			double shoulderRightYdouble = Convert.ToDouble(shoulderRightY);

			double handXdoubletoeven = Math.Round(handXdouble, 1, MidpointRounding.ToEven); // Rounds to even
			double shoulderRightXdoubletoeven = Math.Round(shoulderRightXdouble, 1, MidpointRounding.ToEven); // Rounds to even
			double handYdoubletoeven = Math.Round(handYdouble, 1, MidpointRounding.ToEven); // Rounds to even
			double shoulderRightYdoubletoeven = Math.Round(shoulderRightYdouble, 1, MidpointRounding.ToEven); // Rounds to even

			bool isOverStop = handXdoubletoeven == shoulderRightXdoubletoeven & amp; &amp; handYdoubletoeven == shoulderRightYdoubletoeven;
			return isOverStop;
		}
	}
}