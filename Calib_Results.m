% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 7258.014241897765714 ; 7251.477086052713275 ];

%-- Principal point:
cc = [ 2621.059500897867110 ; 1730.728303398705748 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.201684405767599 ; 0.468305709098890 ; -0.000780140138275 ; 0.001508925067000 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 13.249333989403308 ; 12.679196586538120 ];

%-- Principal point uncertainty:
cc_error = [ 20.384751942068721 ; 15.723251871923811 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.016014783969313 ; 0.222250897578324 ; 0.000511538884957 ; 0.000634962163440 ; 0.000000000000000 ];

%-- Image size:
nx = 5184;
ny = 3456;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 4;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.181091e+00 ; 2.203831e+00 ; 1.739658e-01 ];
Tc_1  = [ -2.060552e+02 ; -1.245638e+02 ; 1.202500e+03 ];
omc_error_1 = [ 2.208949e-03 ; 2.061451e-03 ; 4.319836e-03 ];
Tc_error_1  = [ 3.389634e+00 ; 2.631482e+00 ; 2.186156e+00 ];

%-- Image #2:
omc_2 = [ 1.846508e+00 ; 1.731495e+00 ; 8.288899e-01 ];
Tc_2  = [ -1.168515e+02 ; -1.212052e+02 ; 9.046048e+02 ];
omc_error_2 = [ 2.448803e-03 ; 1.806657e-03 ; 3.215081e-03 ];
Tc_error_2  = [ 2.549000e+00 ; 1.967436e+00 ; 1.679915e+00 ];

%-- Image #3:
omc_3 = [ -1.831217e+00 ; -1.861685e+00 ; -7.066622e-01 ];
Tc_3  = [ -2.098202e+02 ; -4.273198e+01 ; 8.666456e+02 ];
omc_error_3 = [ 1.418445e-03 ; 2.357785e-03 ; 3.471642e-03 ];
Tc_error_3  = [ 2.434329e+00 ; 1.902736e+00 ; 1.673173e+00 ];

%-- Image #4:
omc_4 = [ -2.059450e+00 ; -2.054166e+00 ; 4.019440e-01 ];
Tc_4  = [ -2.519197e+02 ; -1.172768e+02 ; 1.132801e+03 ];
omc_error_4 = [ 2.305740e-03 ; 1.800699e-03 ; 3.632169e-03 ];
Tc_error_4  = [ 3.186813e+00 ; 2.481380e+00 ; 1.849618e+00 ];

