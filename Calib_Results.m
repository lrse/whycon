% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1404.283214750724028 ; 1401.895740682764426 ];

%-- Principal point:
cc = [ 647.468403519270169 ; 344.829554848256748 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.156573946692681 ; -0.531019903933199 ; 0.000211545045502 ; -0.000438267857863 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 2.137950629319062 ; 2.195948632710874 ];

%-- Principal point uncertainty:
cc_error = [ 2.277076574762534 ; 1.939344722363009 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.006093335233319 ; 0.032551648991615 ; 0.000511807230573 ; 0.000692463798313 ; 0.000000000000000 ];

%-- Image size:
nx = 1280;
ny = 720;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 15;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.192545e+00 ; 2.206203e+00 ; -5.862499e-02 ];
Tc_1  = [ -3.104628e+02 ; -1.759748e+02 ; 1.021453e+03 ];
omc_error_1 = [ 1.365067e-03 ; 1.509816e-03 ; 3.141919e-03 ];
Tc_error_1  = [ 1.661465e+00 ; 1.422362e+00 ; 1.698263e+00 ];

%-- Image #2:
omc_2 = [ 2.194380e+00 ; 2.201903e+00 ; -4.854766e-02 ];
Tc_2  = [ -3.169134e+02 ; -1.752501e+02 ; 1.018739e+03 ];
omc_error_2 = [ 1.371201e-03 ; 1.517011e-03 ; 3.151200e-03 ];
Tc_error_2  = [ 1.657566e+00 ; 1.420387e+00 ; 1.701302e+00 ];

%-- Image #3:
omc_3 = [ NaN ; NaN ; NaN ];
Tc_3  = [ NaN ; NaN ; NaN ];
omc_error_3 = [ NaN ; NaN ; NaN ];
Tc_error_3  = [ NaN ; NaN ; NaN ];

%-- Image #4:
omc_4 = [ NaN ; NaN ; NaN ];
Tc_4  = [ NaN ; NaN ; NaN ];
omc_error_4 = [ NaN ; NaN ; NaN ];
Tc_error_4  = [ NaN ; NaN ; NaN ];

%-- Image #5:
omc_5 = [ NaN ; NaN ; NaN ];
Tc_5  = [ NaN ; NaN ; NaN ];
omc_error_5 = [ NaN ; NaN ; NaN ];
Tc_error_5  = [ NaN ; NaN ; NaN ];

%-- Image #6:
omc_6 = [ -1.769196e+00 ; -1.768196e+00 ; -6.277700e-01 ];
Tc_6  = [ -2.497841e+02 ; -1.096981e+02 ; 6.411859e+02 ];
omc_error_6 = [ 9.705776e-04 ; 1.391111e-03 ; 1.969102e-03 ];
Tc_error_6  = [ 1.045519e+00 ; 9.052319e-01 ; 1.093940e+00 ];

%-- Image #7:
omc_7 = [ -1.786803e+00 ; -1.745902e+00 ; -6.310336e-01 ];
Tc_7  = [ -2.476025e+02 ; -1.066490e+02 ; 6.430391e+02 ];
omc_error_7 = [ 9.750660e-04 ; 1.386167e-03 ; 1.969712e-03 ];
Tc_error_7  = [ 1.048998e+00 ; 9.065544e-01 ; 1.092632e+00 ];

%-- Image #8:
omc_8 = [ -2.145360e+00 ; -2.118366e+00 ; -5.767827e-01 ];
Tc_8  = [ -2.844390e+02 ; -1.622727e+02 ; 8.510944e+02 ];
omc_error_8 = [ 1.058641e-03 ; 1.347692e-03 ; 2.640254e-03 ];
Tc_error_8  = [ 1.401295e+00 ; 1.206780e+00 ; 1.439370e+00 ];

%-- Image #9:
omc_9 = [ -2.175494e+00 ; -2.114776e+00 ; -6.132284e-01 ];
Tc_9  = [ -3.208103e+02 ; -1.553255e+02 ; 8.455832e+02 ];
omc_error_9 = [ 1.093670e-03 ; 1.330071e-03 ; 2.713703e-03 ];
Tc_error_9  = [ 1.393495e+00 ; 1.205907e+00 ; 1.450793e+00 ];

%-- Image #10:
omc_10 = [ -2.177371e+00 ; -2.109008e+00 ; -6.088165e-01 ];
Tc_10  = [ -3.162459e+02 ; -1.520348e+02 ; 8.473076e+02 ];
omc_error_10 = [ 1.088539e-03 ; 1.331667e-03 ; 2.703893e-03 ];
Tc_error_10  = [ 1.395349e+00 ; 1.206832e+00 ; 1.450025e+00 ];

%-- Image #11:
omc_11 = [ NaN ; NaN ; NaN ];
Tc_11  = [ NaN ; NaN ; NaN ];
omc_error_11 = [ NaN ; NaN ; NaN ];
Tc_error_11  = [ NaN ; NaN ; NaN ];

%-- Image #12:
omc_12 = [ 2.056525e+00 ; 2.064390e+00 ; -4.131855e-01 ];
Tc_12  = [ -3.081739e+02 ; -1.707733e+02 ; 9.836085e+02 ];
omc_error_12 = [ 9.483772e-04 ; 1.362828e-03 ; 2.442888e-03 ];
Tc_error_12  = [ 1.591856e+00 ; 1.357206e+00 ; 1.446337e+00 ];

%-- Image #13:
omc_13 = [ -2.024379e+00 ; -2.086241e+00 ; 2.503551e-01 ];
Tc_13  = [ -4.232713e+02 ; -1.726512e+02 ; 1.071289e+03 ];
omc_error_13 = [ 1.452180e-03 ; 1.227366e-03 ; 2.488446e-03 ];
Tc_error_13  = [ 1.746664e+00 ; 1.519468e+00 ; 1.709866e+00 ];

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

