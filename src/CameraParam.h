#pragma once

#include "ofMain.h"

#include "ofxCv.h"

static const cv::Size2f SENSOR_SIZE_35MM = cv::Size2f(36, 24);
static const cv::Size2f SENSOR_SIZE_APS_C = cv::Size2f(23.4, 16.7);

static const cv::Size2f SENSOR_SIZE_KINECT_COLOR = cv::Size2f(3.58, 2.87); // MT9M112
static const cv::Size2f SENSOR_SIZE_KINECT_IR = cv::Size2f(6.83, 5.45); // MT9M001

class CameraParam
{
public:
	
	struct Frustum
	{
		float left, right, bottom, top;
		float near, far;
		
		inline ofMatrix4x4 getMatrix() const
		{
			ofMatrix4x4 m;
			m.makeFrustumMatrix(left, right, bottom, top, near, far);
			return m;
		}
	};

	class Intrinsics
	{
		cv::Mat camera_matrix;
		cv::Size image_size;
		
		cv::Point2d principal_point, fov;
		double aspect ,focal_length;
		
		bool inited;
		
	public:
		
		Intrinsics();
		Intrinsics(cv::Mat camera_matrix, cv::Size image_size);
		Intrinsics(float fovY, cv::Size image_size, cv::Point2d principalPoint = cv::Point2d(-1, -1));
		
		// use for calculate the focal length (near clip distance in opengl)
		// if both value of width and height are 0, may use pixel unit
		void setSensorSize(float width_in_mm = 0, float height_in_mm = 0);
		void setSensorSize(cv::Size2f sensor_size);
		
		inline const cv::Size& getImageSize() const { return image_size; }
		inline float getWidth() const { return image_size.width; }
		inline float getHeight() const { return image_size.height; }
		
		inline const cv::Point2d& getFov() const { return fov; }
		inline float getFovX() const { return fov.x; }
		inline float getFovY() const { return fov.y; }
		
		inline const cv::Point2d& getPrincipalPoint() const { return principal_point; }
		
		inline float getFocalLength() const { return focal_length; }
		inline float getAspect() const { return aspect; }
		
		inline const cv::Mat& getCameraMatrix() const { return camera_matrix; }
		
		Frustum getFrustumData(float near, float far) const;

		ofMatrix4x4 getProjectionMatrix(float near, float far) const;

		void draw(float near, float far) const;

	protected:
		
		cv::Size2f sensor_size;
		void updateParams();
	};
	
	class Extrinsic
	{
		ofMatrix4x4 mat;
		
	public:
		
		Extrinsic() { mat.makeIdentityMatrix(); }
		Extrinsic(const cv::Mat& rotation, const cv::Mat& translation);
		
		void setPosition(const ofVec3f& v) { mat.setTranslation(v); }
		void setRotation(const ofQuaternion& q) { mat.setRotate(q); }
		
		const ofMatrix4x4& getModelViewMatrix() const { return mat; }
	};
	
	CameraParam() {}
	CameraParam(const Intrinsics& intrinsics, const Extrinsic& extrinsic);
	CameraParam(const CameraParam& copy);

	Intrinsics& getIntrinsics() { return intrinsics; }
	const Intrinsics& getIntrinsics() const { return intrinsics; }
	Extrinsic& getExtrinsic() { return extrinsic; }
	const Extrinsic& getExtrinsic() const { return extrinsic; }
	
	void begin() const;
	void end() const;
	
	void beginIntrinsics() const;
	void endIntrinsics() const;
	
	void beginExtrinsic() const;
	void endExtrinsics() const;
	
	void drawFrustum() const;
	
	void setFar(float far_) { far = far_; }

	void setSensorSize(float width_in_mm = 0, float height_in_mm = 0);
	void setSensorSize(cv::Size2f sensor_size);
	
protected:
	
	mutable float near, far;
	
	Intrinsics intrinsics;
	Extrinsic extrinsic;
	
	void updateClip() const;
};
