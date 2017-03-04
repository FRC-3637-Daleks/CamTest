#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <WPILib.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <GripPipeline.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
class Robot: public frc::IterativeRobot {

private:


START_ROBOT_CLASS(Robot)
	cs::UsbCamera *usbCamera0;
	cs::MjpegServer *mjpegServer0;
	cs::CvSink *cvSink0;
	cs::CvSource cvSource0, cvSource1;
	cv::Mat source;
	grip::GripPipeline gp;
	int contourArea1, contourArea2;
	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		usbCamera0  = new cs::UsbCamera("USB Camera 0", 0);
		usbCamera0->SetResolution(320,240);
		usbCamera0->SetBrightness(150);
		usbCamera0->SetExposureManual(.75);
		contourArea1 = 0;
		contourArea2 = 0;


		mjpegServer0 = new cs::MjpegServer("serve_USB Camera 0", 1181);
		cvSink0      = new cs::CvSink("opencv_USB Camera 0");
		mjpegServer0->SetSource(*usbCamera0);
		cvSink0->SetSource(*usbCamera0);
		cvSource0 = CameraServer::GetInstance()->PutVideo("Output", 640, 480);
		cvSource1 = CameraServer::GetInstance()->PutVideo("processed", 640, 480);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {

		double tolerance = .2;
		//
		std::vector<std::vector<cv::Point>> *dContours;
		if (cvSink0->GrabFrame(source) != 0){
			gp.process(source);
			dContours = gp.getfindContoursOutput();

		}
		cv::Rect r1;
		cv::Rect r2;

		if (dContours->size()>1){
			unsigned int i = 0;
			while (i<dContours->size()){

				if (i%2==0){
					r1 = boundingRect(dContours->at(i));
					frc::SmartDashboard::PutNumber("Area1", r1.height*r1.width);
					frc::SmartDashboard::PutNumber("Midpoint1_Y", r1.br().y-(r1.height/2));
					frc::SmartDashboard::PutNumber("Midpoint1_X", r1.br().x-(r1.width/2));
					if ((std::abs((r1.height/r1.width)-(2.5)))/2.5 <= tolerance ){
						cv::rectangle(source, r1, cv::Scalar(225,0,0), 1, 8, 0);
					}

				}
				if (i%2==1){
					r2 = boundingRect(dContours->at(i));
					frc::SmartDashboard::PutNumber("Area2", r2.height*r2.width);
					frc::SmartDashboard::PutNumber("Midpoint2_Y", r2.br().y-(r2.height/2));
					frc::SmartDashboard::PutNumber("Midpoint2_X", r2.br().x-(r2.width/2));
					if ((std::abs((r2.height/r2.width)-(2.5)))/2.5 <= tolerance ){
						cv::rectangle(source, r2, cv::Scalar(225,0,0), 1, 8, 0);
					}
				}
				i++;
			}
			if(r1.br().x-(r1.width/2) + r2.br().x-(r2.width/2) / 2 > 360){
				frc::SmartDashboard::PutString("Direction", "Go Left");

			}
			else{
				frc::SmartDashboard::PutString("Direction", "Go Right");
			}


			}
			else if(dContours->size()>0){
				cv::Rect bb = boundingRect(dContours->at(0));
				if(bb.br().x-(bb.width/2) > 160){
					frc::SmartDashboard::PutString("Direction", "Go Left");
				}
				else{
					frc::SmartDashboard::PutString("Direction", "Go Right");
				}
			}

			cvSource1.PutFrame(source);





		frc::SmartDashboard::PutNumber("Contours", dContours->size());
		frc::SmartDashboard::PutNumber("Brightness", usbCamera0->GetBrightness());
		frc::SmartDashboard::PutNumber("Mat output", cvSink0->GrabFrame(source));
		frc::SmartDashboard::PutString("source", cvSink0->GetSource().GetName());


	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;

};

START_ROBOT_CLASS(Robot)
