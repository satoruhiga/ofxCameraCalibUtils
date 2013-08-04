#pragma once

#include "ofMain.h"

#include "ofxCv.h"

#include "CameraParam.h"

class Calibration;

class ChessBoard
{
	friend class Calibration;
	
public:
	
	// TODO: generate chessboard image
	
	ChessBoard(int pattern_size_x = 7, int pattern_size_y = 10, float square_size = 25) : pattern_size(pattern_size_x, pattern_size_y), square_size(square_size), use_fast_check(false), valid(false)
	{
		object_points.clear();
		
		for (int x = 0; x < pattern_size.height; x++)
			for (int y = 0; y < pattern_size.width; y++)
				object_points.push_back(cv::Point3f(x, y, 0) * square_size);
		
		for (int i = 0; i < object_points.size(); i++)
		{
			float d = ofMap(i, 0, object_points.size(), 0, 255);
			corner_colors.push_back(ofColor::fromHsb(d, 255, 255));
		}
		
		board_size.width = (float)(pattern_size_y - 1) * square_size;
		board_size.height = (float)(pattern_size_x - 1) * square_size;
	}
	
	bool find(string path)
	{
		ofPixels pix;
		if (!ofLoadImage(pix, path))
		{
			ofLogError("ChessBoard") << "invalid image: " << path;
			return false;
		}
		return find(pix);
	}
	
	bool find(const ofPixels &pix)
	{
		corners.clear();
		
		image = pix;
		
		if (pix.getImageType() == OF_IMAGE_COLOR)
			ofxCv::convertColor(image, mat, CV_RGB2GRAY);
		else if (pix.getImageType() == OF_IMAGE_COLOR_ALPHA)
			ofxCv::convertColor(image, mat, CV_RGBA2GRAY);
		else
			ofxCv::copy(image, mat);
		
		int chessFlags = CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE;
		
		if (use_fast_check)
			chessFlags |= CV_CALIB_CB_FAST_CHECK;
		
		if (!cv::findChessboardCorners(mat, pattern_size, corners, chessFlags))
		{
			valid = false;
			return false;
		}
		
		cv::cornerSubPix(mat, corners, cv::Size(11, 11), cv::Size(-1, -1),  cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
		
		valid = true;
		return true;
	}
	
	void draw()
	{
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glDisable(GL_DEPTH_TEST);
		
		ofPushStyle();
		
		ofSetRectMode(OF_RECTMODE_CORNER);
		image.draw(0, 0);
		
		ofSetLineWidth(2);
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i < corners.size(); i++)
		{
			const cv::Point2f &p = corners[i];
			ofSetColor(corner_colors[i]);
			glVertex2f(p.x, p.y);
		}
		glEnd();
		
		for (int i = 0; i < corners.size(); i++)
		{
			const cv::Point2f &p = corners[i];
			ofSetColor(corner_colors[i]);
			ofCircle(p.x, p.y, 3);
			
			ofDrawBitmapString(ofToString(i), p.x + 5, p.y);
		}
		
		ofSetRectMode(OF_RECTMODE_CORNER);
		ofSetColor(255);
		
		ofPopStyle();
		
		glPopAttrib();
	}
	
	void drawExtrinsic(int id, ofColor color) const
	{
		const ofMatrix4x4& m = param.getExtrinsic().getModelViewMatrix();
		
		ofPushStyle();
		{
			ofPushMatrix();
			ofMultMatrix(m);
			
			ofSetColor(color);
			ofSetRectMode(OF_RECTMODE_CORNER);
			ofNoFill();
			ofRect(0, 0, board_size.width, board_size.height);
			
			ofDrawAxis(square_size * 2);
			
			ofDrawBitmapString(ofToString(id), 0, 0);
			
			ofPopMatrix();
		}
		ofPopStyle();
	}
	
	void enableFastCheck(bool b = true) { use_fast_check = b; }
	
	bool isValid() const { return valid; }
	
	const vector<cv::Point2f>& getCorners() const { return corners; }
	const vector<cv::Point3f>& getObjectPoints() const { return object_points; }
	
	cv::Size getImageSize() { return cv::Size(image.getWidth(), image.getHeight()); }
	float getSquareSize() const { return square_size; }
	
	const CameraParam& getCameraParam() const { return param; }
	
	ofImage& getImage() { return image; }
	
protected:
	
	ofImage image;
	
	cv::Mat mat;
	cv::Size pattern_size;
	
	float square_size;
	cv::Size2f board_size;
	
	bool use_fast_check;
	bool valid;
	
	vector<ofColor> corner_colors;
	vector<cv::Point3f> object_points;
	vector<cv::Point2f> corners;
	
	CameraParam param;
};
