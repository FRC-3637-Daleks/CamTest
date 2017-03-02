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
		usbCamera0->SetResolution(640,480);
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
		cvSink0->GrabFrame(source);
		//
		std::vector<std::vector<cv::Point>> *dContours;
		if (cvSink0->GrabFrame(source) != 0){
			gp.process(source);
			cv::Mat processed;

			processed = cv::Mat::zeros(source.size(),CV_8U);
			std::vector<std::vector<cv::Point>> *dalekContours = gp.getfindContoursOutput();
			dContours = dalekContours;
			for(unsigned int i= 0; i < dalekContours->size(); i++)
			{
				for(unsigned int j= 0; j < dalekContours->at(i).size();j++) // run until j < contours[i].size();
					{
						processed.at<int>(dalekContours->at(i).at(j)) = 1;

					}
			}

			cvSource0.PutFrame(source);
			cvSource1.PutFrame(processed);
		}

		frc::SmartDashboard::PutNumber("Contours", dContours->size());

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
