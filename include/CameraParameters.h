#ifndef CAMERA_PARAMETERS_H
#define CAMERA_PARAMETERS_H

class CameraParameters {
	public:
		CameraParameters() : _pFOVParameters(NULL), _pIntrinsicParameters(NULL), _CameraParamsType(CAMERA_PARAMETER_INVALID) {
		}

		CameraParametersType GetCameraParameterType() 			{ return _CameraParamsType; 	}
		FOVParameters* 		 GetFovParametersInstance() 		{ return _pFOVParameters; 		}
		IntrinsicParameters* GetIntrinsicParametersInstance() 	{ return _pIntrinsicParameters; }

		void SetFovParametersInstance(FOVParameters* pFOVParameters) {
			_pFOVParameters = pFOVParameters;
			_CameraParamsType = CAMERA_FOV_PARAMETERS;
		}

		void SetIntrinsicParametersInstance(IntrinsicParameters* pIntrinsicParameters) {
			_pIntrinsicParameters = pIntrinsicParameters;
			_CameraParamsType = CAMERA_INTRINSIC_PARAMETERS;
		}

	private:
		CameraParametersType 	_CameraParamsType;
		FOVParameters* 			_pFOVParameters;
		IntrinsicParameters* 	_pIntrinsicParameters;
};

#endif