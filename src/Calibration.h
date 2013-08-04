#pragma once

#include "ofMain.h"

#include "CameraParam.h"
#include "ChessBoard.h"
#include "Utils.h"

#define DBG(v) cout << #v << ": " << v << endl;

class Calibration
{
public:
	
	void add(const ChessBoard& o) { chess_boards.push_back(o); }
	void clear() { chess_boards.clear(); }
	
	size_t size() const { return chess_boards.size(); }
	const ChessBoard& operator[](size_t idx) const { return chess_boards[idx]; }
	
	void drawChessboards() const
	{
		for (int i = 0; i < chess_boards.size(); i++)
		{
			float d = ofMap(i, 0, chess_boards.size(), 0, 255);
			const ChessBoard &o = chess_boards[i];
			o.drawExtrinsic(i, ofColor::fromHsb(d, 255, 255));
		}
	}
	
	void begin() const { camera_param.begin(); }
	void end() const { camera_param.end(); }
	
	void draw() const
	{
		ofPushMatrix();
		camera_param.beginExtrinsic();
		camera_param.drawFrustum();
		drawChessboards();
		camera_param.endExtrinsics();
		ofPopMatrix();
	}
	
	void updateCalibration()
	{
		rms = updateCameraParams();
	}
	
	float getRMS() const { return rms; }
	
	CameraParam& getCameraParam() { return camera_param; }
	const CameraParam& getCameraParam() const { return camera_param; }
	
protected:
	
	vector<ChessBoard> chess_boards;
	
	cv::Mat dist_coeffs;
	CameraParam camera_param;
	
	float rms;
	
	float updateCameraParams()
	{
		if (chess_boards.empty())
		{
			ofLogError("Calibration") << "No chessboard added";
			return rms = NAN;
		}
		
		vector<vector<cv::Point3f> > object_points;
		vector<vector<cv::Point2f> > image_points;
		
		for (int i = 0; i < chess_boards.size(); i++)
		{
			const vector<cv::Point3f>& o = chess_boards[i].getObjectPoints();
			object_points.push_back(o);
			
			const vector<cv::Point2f>& m = chess_boards[i].getCorners();
			image_points.push_back(m);
		}
		
		cv::Size image_size = chess_boards[0].getImageSize();
		cv::Mat camera_matrix = cv::Mat::ones(3, 3, CV_64F);
		
		dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
		
		vector<cv::Mat> rvecs;
		vector<cv::Mat> tvecs;
		
		int flags = 0;
		float rms = cv::calibrateCamera(object_points,
										image_points,
										image_size,
										camera_matrix,
										dist_coeffs,
										rvecs,
										tvecs);
		
		// update camera param
		for (int i = 0; i < chess_boards.size(); i++)
		{
			ChessBoard &o = chess_boards[i];
			o.param = CameraParam(CameraParam::Intrinsics(camera_matrix, image_size),
								  CameraParam::Extrinsic(rvecs[i], tvecs[i]));
		}
		
		camera_param = CameraParam(CameraParam::Intrinsics(camera_matrix, image_size), CameraParam::Extrinsic());
		
		return rms;
	}
	
	void updateReprojectionErrors()
	{
		// TODO: https://github.com/kylemcdonald/ofxCv/blob/master/libs/ofxCv/src/Calibration.cpp#L408
	}
};
