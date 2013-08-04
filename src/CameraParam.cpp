#include "CameraParam.h"

#pragma mark - Intrinsics

CameraParam::Intrinsics::Intrinsics() : inited(false) {}

CameraParam::Intrinsics::Intrinsics(cv::Mat camera_matrix, cv::Size image_size) : camera_matrix(camera_matrix), image_size(image_size), sensor_size(0, 0)
{
	updateParams();
	
	inited = true;
}

CameraParam::Intrinsics::Intrinsics(float fovY, cv::Size image_size, cv::Point2d principal_point_) : image_size(image_size), sensor_size(0, 0)
{
	if (principal_point_ == cv::Point2d(-1, -1))
		principal_point_ = cv::Point2d(image_size.width / 2, image_size.height / 2);
	
	principal_point = principal_point_;
	
	float f = (image_size.height / 2) * tan(ofDegToRad(90 - fovY / 2));
	camera_matrix = (cv::Mat_<double>(3, 3) <<
					f, 0, principal_point.x,
					0, f, principal_point.y,
					0, 0, 1);
	
	updateParams();
	
	inited = true;
}

void CameraParam::Intrinsics::setSensorSize(float width_in_mm, float height_in_mm)
{
	sensor_size.width = width_in_mm;
	sensor_size.height = height_in_mm;
	
	if (inited)
	{
		updateParams();
	}
}

void CameraParam::Intrinsics::setSensorSize(cv::Size2f sensor_size_)
{
	setSensorSize(sensor_size_.width, sensor_size_.height);
}

CameraParam::Frustum CameraParam::Intrinsics::getFrustumData(float near, float far) const
{
	const float w = image_size.width;
	const float h = image_size.height;
	const float fx = camera_matrix.at<double>(0, 0);
	const float fy = camera_matrix.at<double>(1, 1);
	const float s = camera_matrix.at<double>(0, 1);
	const float cx = camera_matrix.at<double>(0, 2);
	const float cy = camera_matrix.at<double>(1, 2);

	Frustum ret = {
		near * (-cx) / fx, near * (w - cx) / fx,
		near * (cy - h) / fy, near * (cy) / fy,
		near, far
	};
	
	return ret;
}

ofMatrix4x4 CameraParam::Intrinsics::getProjectionMatrix(float near, float far) const
{
	return getFrustumData(near, far).getMatrix();
}

void CameraParam::Intrinsics::updateParams()
{
	cv::calibrationMatrixValues(camera_matrix,
								image_size,
								sensor_size.width,
								sensor_size.height,
								fov.x,
								fov.y,
								focal_length,
								principal_point,
								aspect);
}

void CameraParam::Intrinsics::draw(float near, float far) const
{
	if (!inited) return;
	
	if (near == 0) near = 0.0001;
	
	const Frustum f = getFrustumData(near, far);
	
	const float s = f.far / f.near;
	
	const ofVec3f n0(f.left, f.top, -f.near);
	const ofVec3f n1(f.right, f.top, -f.near);
	const ofVec3f n2(f.right, f.bottom, -f.near);
	const ofVec3f n3(f.left, f.bottom, -f.near);
	
	const ofVec3f f0 = n0 * s;
	const ofVec3f f1 = n1 * s;
	const ofVec3f f2 = n2 * s;
	const ofVec3f f3 = n3 * s;
	
	const ofColor c = ofGetStyle().color;
	const ofVec3f zero(0, 0, 0);
	const ofVec3f focal_distance(0, 0, -near);
	
	ofPushStyle();
	ofEnableAlphaBlending();
	
	ofNoFill();
	
	ofSetColor(c, 200);
	
	ofLine(n0, n1);
	ofLine(n1, n2);
	ofLine(n2, n3);
	ofLine(n3, n0);
	
	ofLine(f0, f1);
	ofLine(f1, f2);
	ofLine(f2, f3);
	ofLine(f3, f0);
	
	ofLine(n0, f0);
	ofLine(n1, f1);
	ofLine(n2, f2);
	ofLine(n3, f3);
	
	ofDrawAxis(near * 0.25);
	
	ofSetColor(c, 60);
	
	ofLine(zero, n0);
	ofLine(zero, n1);
	ofLine(zero, n2);
	ofLine(zero, n3);
	
	ofLine(zero, focal_distance);
	
	ofSetColor(c, 200);
	
	ofDrawBitmapString("f:" + ofToString(getFocalLength(), 2), zero.getMiddle(focal_distance));
	
	ofPopStyle();
}

#pragma mark - Extrinsic

CameraParam::Extrinsic::Extrinsic(const cv::Mat& rotation, const cv::Mat& translation)
{
	cv::Mat rot3x3;
	if(rotation.rows == 3 && rotation.cols == 3) {
		rot3x3 = rotation;
	} else {
		cv::Rodrigues(rotation, rot3x3);
	}
	
	const double* rm = rot3x3.ptr<double>(0);
	const double* tm = translation.ptr<double>(0);
	
	mat.makeIdentityMatrix();
	mat.set(rm[0], rm[1], rm[2], 0,
			rm[3], rm[4], rm[5], 0,
			rm[6], rm[7], rm[8], 0,
			tm[0], tm[1], tm[2], 1);
	
	// convert coordinate system opencv to opengl
	mat.postMultScale(1, -1, -1);
}

#pragma mark - CameraParam

CameraParam::CameraParam(const Intrinsics& intrinsics, const Extrinsic& extrinsic)
: intrinsics(intrinsics), extrinsic(extrinsic), far(5000)
{
	updateClip();
}

CameraParam::CameraParam(const CameraParam& copy)
{
	intrinsics = copy.intrinsics;
	extrinsic = copy.extrinsic;
	far = copy.far;
}

void CameraParam::begin() const
{
	beginIntrinsics();
	beginExtrinsic();
}

void CameraParam::end() const
{
	endExtrinsics();
	endIntrinsics();
}

void CameraParam::beginIntrinsics() const
{
	updateClip();
	
	ofPushView();
	ofViewport(0, 0, intrinsics.getWidth(), intrinsics.getHeight());
	
	ofPushMatrix(); // GL_PROJECTION
	glMatrixMode(GL_PROJECTION);
	
	ofLoadMatrix(intrinsics.getProjectionMatrix(near, far));
	
	ofPushMatrix(); // GL_MODELVIEW
	glMatrixMode(GL_MODELVIEW);
	
	glLoadIdentity();
}

void CameraParam::endIntrinsics() const
{
	ofPopMatrix(); // GL_MODELVIEW
	ofPopMatrix(); // GL_PROJECTION
	
	ofPopView();
}

void CameraParam::beginExtrinsic() const
{
	ofMatrix4x4 m;
	m.makeInvertOf(extrinsic.getModelViewMatrix());
	ofMultMatrix(m);
}

void CameraParam::endExtrinsics() const
{
}

void CameraParam::drawFrustum() const
{
	intrinsics.draw(near, far);
}

void CameraParam::setSensorSize(float width_in_mm, float height_in_mm)
{
	intrinsics.setSensorSize(width_in_mm, height_in_mm);
	updateClip();
}

void CameraParam::setSensorSize(cv::Size2f sensor_size)
{
	intrinsics.setSensorSize(sensor_size);
	updateClip();
}

void CameraParam::updateClip() const
{
	near = intrinsics.getFocalLength();
}
