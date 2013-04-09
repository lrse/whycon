% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1440.830369637999183 ; 1441.839953024453507 ];

%-- Principal point:
cc = [ 596.022018435307587 ; 454.268567820455871 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.029570924296297 ; -0.103154693291318 ; -0.002139052547737 ; -0.003427999493844 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 4.496900132051064 ; 4.468125995525728 ];

%-- Principal point uncertainty:
cc_error = [ 4.109940271859303 ; 4.225289592655966 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.007334321721380 ; 0.023800782923474 ; 0.000916274564201 ; 0.000881083059960 ; 0.000000000000000 ];

%-- Image size:
nx = 1280;
ny = 960;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 21;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 1.965232e+00 ; 1.894424e+00 ; -3.882639e-01 ];
Tc_1  = [ -2.284163e+02 ; -9.909792e+01 ; 6.846695e+02 ];
omc_error_1 = [ 2.086428e-03 ; 2.571471e-03 ; 4.514308e-03 ];
Tc_error_1  = [ 1.945916e+00 ; 2.035721e+00 ; 2.292743e+00 ];

%-- Image #2:
omc_2 = [ 1.985544e+00 ; 2.095450e+00 ; -7.296084e-01 ];
Tc_2  = [ -4.953605e+01 ; -1.023503e+02 ; 7.042085e+02 ];
omc_error_2 = [ 2.087172e-03 ; 2.549865e-03 ; 4.503174e-03 ];
Tc_error_2  = [ 1.990272e+00 ; 2.034546e+00 ; 2.089968e+00 ];

%-- Image #3:
omc_3 = [ NaN ; NaN ; NaN ];
Tc_3  = [ NaN ; NaN ; NaN ];
omc_error_3 = [ NaN ; NaN ; NaN ];
Tc_error_3  = [ NaN ; NaN ; NaN ];

%-- Image #4:
omc_4 = [ -1.693709e+00 ; -1.737059e+00 ; 9.921082e-01 ];
Tc_4  = [ 1.123982e+02 ; -1.345023e+02 ; 9.084551e+02 ];
omc_error_4 = [ 2.731000e-03 ; 2.150338e-03 ; 3.976800e-03 ];
Tc_error_4  = [ 2.609675e+00 ; 2.657985e+00 ; 2.593760e+00 ];

%-- Image #5:
omc_5 = [ 2.180382e+00 ; 2.127946e+00 ; -2.322868e-01 ];
Tc_5  = [ -2.099129e+02 ; -1.334061e+02 ; 6.416531e+02 ];
omc_error_5 = [ 2.245100e-03 ; 2.645618e-03 ; 5.297554e-03 ];
Tc_error_5  = [ 1.826832e+00 ; 1.897773e+00 ; 2.279487e+00 ];

%-- Image #6:
omc_6 = [ 2.143753e+00 ; 2.068939e+00 ; -1.116266e-01 ];
Tc_6  = [ -7.555702e+01 ; -1.336128e+02 ; 5.911731e+02 ];
omc_error_6 = [ 2.639978e-03 ; 2.234584e-03 ; 4.876345e-03 ];
Tc_error_6  = [ 1.697143e+00 ; 1.717513e+00 ; 1.982913e+00 ];

%-- Image #7:
omc_7 = [ NaN ; NaN ; NaN ];
Tc_7  = [ NaN ; NaN ; NaN ];
omc_error_7 = [ NaN ; NaN ; NaN ];
Tc_error_7  = [ NaN ; NaN ; NaN ];

%-- Image #8:
omc_8 = [ NaN ; NaN ; NaN ];
Tc_8  = [ NaN ; NaN ; NaN ];
omc_error_8 = [ NaN ; NaN ; NaN ];
Tc_error_8  = [ NaN ; NaN ; NaN ];

%-- Image #9:
omc_9 = [ NaN ; NaN ; NaN ];
Tc_9  = [ NaN ; NaN ; NaN ];
omc_error_9 = [ NaN ; NaN ; NaN ];
Tc_error_9  = [ NaN ; NaN ; NaN ];

%-- Image #10:
omc_10 = [ -2.223436e+00 ; -2.156402e+00 ; 3.250998e-01 ];
Tc_10  = [ -1.743566e+02 ; -1.291247e+02 ; 5.526230e+02 ];
omc_error_10 = [ 2.312299e-03 ; 1.846462e-03 ; 4.616127e-03 ];
Tc_error_10  = [ 1.563964e+00 ; 1.623843e+00 ; 1.934869e+00 ];

%-- Image #11:
omc_11 = [ -2.190453e+00 ; -2.132918e+00 ; 3.674529e-01 ];
Tc_11  = [ -8.862708e+01 ; -1.306178e+02 ; 5.610295e+02 ];
omc_error_11 = [ 2.220532e-03 ; 2.032813e-03 ; 4.536937e-03 ];
Tc_error_11  = [ 1.592384e+00 ; 1.624711e+00 ; 1.874476e+00 ];

%-- Image #12:
omc_12 = [ 2.101426e+00 ; 1.959555e+00 ; 1.440264e-01 ];
Tc_12  = [ -2.095653e+02 ; -1.281965e+02 ; 5.392075e+02 ];
omc_error_12 = [ 2.386755e-03 ; 2.213562e-03 ; 4.598873e-03 ];
Tc_error_12  = [ 1.581379e+00 ; 1.619803e+00 ; 2.029507e+00 ];

%-- Image #13:
omc_13 = [ NaN ; NaN ; NaN ];
Tc_13  = [ NaN ; NaN ; NaN ];
omc_error_13 = [ NaN ; NaN ; NaN ];
Tc_error_13  = [ NaN ; NaN ; NaN ];

%-- Image #14:
omc_14 = [ NaN ; NaN ; NaN ];
Tc_14  = [ NaN ; NaN ; NaN ];
omc_error_14 = [ NaN ; NaN ; NaN ];
Tc_error_14  = [ NaN ; NaN ; NaN ];

%-- Image #15:
omc_15 = [ NaN ; NaN ; NaN ];
Tc_15  = [ NaN ; NaN ; NaN ];
omc_error_15 = [ NaN ; NaN ; NaN ];
Tc_error_15  = [ NaN ; NaN ; NaN ];

%-- Image #16:
omc_16 = [ NaN ; NaN ; NaN ];
Tc_16  = [ NaN ; NaN ; NaN ];
omc_error_16 = [ NaN ; NaN ; NaN ];
Tc_error_16  = [ NaN ; NaN ; NaN ];

%-- Image #17:
omc_17 = [ NaN ; NaN ; NaN ];
Tc_17  = [ NaN ; NaN ; NaN ];
omc_error_17 = [ NaN ; NaN ; NaN ];
Tc_error_17  = [ NaN ; NaN ; NaN ];

%-- Image #18:
omc_18 = [ NaN ; NaN ; NaN ];
Tc_18  = [ NaN ; NaN ; NaN ];
omc_error_18 = [ NaN ; NaN ; NaN ];
Tc_error_18  = [ NaN ; NaN ; NaN ];

%-- Image #19:
omc_19 = [ -1.750278e+00 ; -1.726710e+00 ; 5.052064e-01 ];
Tc_19  = [ -2.240750e+01 ; -1.472527e+02 ; 6.502281e+02 ];
omc_error_19 = [ 2.298097e-03 ; 2.135927e-03 ; 3.765271e-03 ];
Tc_error_19  = [ 1.857569e+00 ; 1.879049e+00 ; 1.990744e+00 ];

%-- Image #20:
omc_20 = [ 2.164661e+00 ; 2.190481e+00 ; 2.987948e-01 ];
Tc_20  = [ -1.837483e+02 ; -1.332422e+02 ; 4.639074e+02 ];
omc_error_20 = [ 2.262974e-03 ; 1.986171e-03 ; 4.593724e-03 ];
Tc_error_20  = [ 1.367437e+00 ; 1.420397e+00 ; 1.785868e+00 ];

%-- Image #21:
omc_21 = [ NaN ; NaN ; NaN ];
Tc_21  = [ NaN ; NaN ; NaN ];
omc_error_21 = [ NaN ; NaN ; NaN ];
Tc_error_21  = [ NaN ; NaN ; NaN ];

