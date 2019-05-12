#ifndef CAMERA_PARAMETERS_H
#define CAMERA_PARAMETERS_H

class FOVParameters
{
private:
  float _DepthHorizontalFOV;
  float _DepthVerticalFOV;
  float _ColorHorizontalFOV;
  float _ColorVerticalFOV;

public:
  void SetDepthHorizontalFOV (float DepthHorizontalFOV)
  {
    _DepthHorizontalFOV = DepthHorizontalFOV;
  }
  void SetDepthVerticalFOV (float DepthVerticalFOV)
  {
    _DepthVerticalFOV = DepthVerticalFOV;
  }
  void SetColorHorizontalFOV (float ColorHorizontalFOV)
  {
    _ColorHorizontalFOV = ColorHorizontalFOV;
  }
  void SetColorVerticalFOV (float ColorVerticalFOV)
  {
    _ColorVerticalFOV = ColorVerticalFOV;
  }

  float GetDepthHorizontalFOV ()
  {
    return _DepthHorizontalFOV;
  }
  float GetDepthVerticalFOV ()
  {
    return _DepthVerticalFOV;
  }
  float GetColorHorizontalFOV ()
  {
    return _ColorHorizontalFOV;
  }
  float GetColorVerticalFOV ()
  {
    return _ColorVerticalFOV;
  }
};

class IntrinsicParameters
{
private:
  // Focal lengths
  float _Fx;
  float _Fy;
  // Principle points
  float _Cx;
  float _Cy;
public:
  void SetFx (float Fx)
  {
    _Fx = Fx;
  }
  void SetFy (float Fy)
  {
    _Fy = Fy;
  }
  void SetCx (float Cx)
  {
    _Fx = Cx;
  }
  void SetCy (float Cy)
  {
    _Fy = Cy;
  }

  float GetFx ()
  {
    return _Fx;
  }
  float GetFy ()
  {
    return _Fy;
  }
  float GetCx ()
  {
    return _Cx;
  }
  float GetCy ()
  {
    return _Cy;
  }
};

class KinectV1Parameters
{
private:
  float _RefPixelSize;
  float _RefDistance;

public:
  void SetRefPixelSize (float RefPixelSize)
  {
    _RefPixelSize = RefPixelSize;
  }
  void SetRefDistance (float RefDistance)
  {
    _RefDistance = RefDistance;
  }

  float GetRefPixelSize ()
  {
    return _RefPixelSize;
  }
  float GetRefDistance ()
  {
    return _RefDistance;
  }
};

class CameraParameters
{
public:
  CameraParameters ():_pFOVParameters (NULL), _pIntrinsicParameters (NULL),
    _pKinectV1Parameters (NULL), _CameraParamsType (CAMERA_PARAMETER_INVALID)
  {
  }

  CameraParametersType GetCameraParameterType ()
  {
    return _CameraParamsType;
  }
  FOVParameters *GetFovParametersInstance ()
  {
    return _pFOVParameters;
  }
  IntrinsicParameters *GetIntrinsicParametersInstance ()
  {
    return _pIntrinsicParameters;
  }
  KinectV1Parameters *GetKinectV1ParametersInstance ()
  {
    return _pKinectV1Parameters;
  }

  void SetFovParametersInstance (FOVParameters * pFOVParameters)
  {
    _pFOVParameters = pFOVParameters;
    _CameraParamsType = CAMERA_FOV_PARAMETERS;
  }

  void SetIntrinsicParametersInstance (IntrinsicParameters *
				       pIntrinsicParameters)
  {
    _pIntrinsicParameters = pIntrinsicParameters;
    _CameraParamsType = CAMERA_INTRINSIC_PARAMETERS;
  }

  void SetKinectV1ParametersInstance (KinectV1Parameters *
				      pKinectV1Parameters)
  {
    _pKinectV1Parameters = pKinectV1Parameters;
    _CameraParamsType = CAMERA_KINECTV1_PARAMETERS;
  }

private:
  CameraParametersType _CameraParamsType;
  FOVParameters *_pFOVParameters;
  IntrinsicParameters *_pIntrinsicParameters;
  KinectV1Parameters *_pKinectV1Parameters;
};

#endif
