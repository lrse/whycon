% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1418.66598 ; 1420.44187 ];

%-- Principal point:
cc = [ 648.24093 ; 351.03666 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.19024 ; -0.61074 ; 0.00114 ; 0.00560 ; 0.00000 ];

%-- Focal length uncertainty:
fc_error = [ 1.632489265101235 ; 1.709755407322466 ];

%-- Principal point uncertainty:
cc_error = [ 2.868172590720035 ; 2.865971362053316 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.007810787671211 ; 0.052520704821959 ; 0.000901481384240 ; 0.000874617416433 ; 0.000000000000000 ];

%-- Image size:
nx = 1280;
ny = 720;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 18;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.167035e+00 ; 2.171586e+00 ; -1.054959e-01 ];
Tc_1  = [ -1.405269e+02 ; -1.340630e+02 ; 7.874489e+02 ];
omc_error_1 = [ 1.811637e-03 ; 1.819519e-03 ; 3.827391e-03 ];
Tc_error_1  = [ 1.609526e+00 ; 1.602909e+00 ; 1.108363e+00 ];

%-- Image #2:
omc_2 = [ -1.268161e+00 ; -2.682928e+00 ; 3.428047e-01 ];
Tc_2  = [ 1.363951e+02 ; -2.212046e+02 ; 1.326983e+03 ];
omc_error_2 = [ 1.851139e-03 ; 3.356085e-03 ; 5.528594e-03 ];
Tc_error_2  = [ 2.717330e+00 ; 2.707165e+00 ; 1.819726e+00 ];

%-- Image #3:
omc_3 = [ -2.004020e+00 ; -2.070130e+00 ; 3.505146e-01 ];
Tc_3  = [ 5.074159e+01 ; -1.745672e+02 ; 1.317195e+03 ];
omc_error_3 = [ 2.614859e-03 ; 2.695730e-03 ; 5.225348e-03 ];
Tc_error_3  = [ 2.695679e+00 ; 2.679122e+00 ; 1.752527e+00 ];

%-- Image #4:
omc_4 = [ 2.057274e+00 ; 2.131954e+00 ; -2.460841e-01 ];
Tc_4  = [ -1.021673e+02 ; -1.262825e+02 ; 8.435686e+02 ];
omc_error_4 = [ 1.739245e-03 ; 1.828028e-03 ; 3.709444e-03 ];
Tc_error_4  = [ 1.718669e+00 ; 1.706982e+00 ; 1.105328e+00 ];

%-- Image #5:
omc_5 = [ 1.776772e+00 ; 1.941482e+00 ; 6.626306e-01 ];
Tc_5  = [ -2.559717e+02 ; -1.154161e+02 ; 8.118332e+02 ];
omc_error_5 = [ 1.842088e-03 ; 1.524608e-03 ; 2.896990e-03 ];
Tc_error_5  = [ 1.691304e+00 ; 1.702742e+00 ; 1.354493e+00 ];

%-- Image #6:
omc_6 = [ 1.553576e+00 ; 1.580829e+00 ; 9.065854e-01 ];
Tc_6  = [ -1.842510e+02 ; -1.233075e+02 ; 8.002119e+02 ];
omc_error_6 = [ 1.984424e-03 ; 1.549706e-03 ; 2.410012e-03 ];
Tc_error_6  = [ 1.653790e+00 ; 1.651022e+00 ; 1.294192e+00 ];

%-- Image #7:
omc_7 = [ 1.392865e+00 ; 1.691135e+00 ; -1.280076e+00 ];
Tc_7  = [ -7.613895e+01 ; 2.378197e+01 ; 9.939344e+02 ];
omc_error_7 = [ 1.533003e-03 ; 2.178936e-03 ; 2.313519e-03 ];
Tc_error_7  = [ 2.016094e+00 ; 2.025548e+00 ; 1.005156e+00 ];

%-- Image #8:
omc_8 = [ 1.686516e+00 ; 7.802110e-01 ; -5.044458e-01 ];
Tc_8  = [ -1.942147e+02 ; 3.109336e+00 ; 8.348324e+02 ];
omc_error_8 = [ 1.971338e-03 ; 1.646825e-03 ; 1.945493e-03 ];
Tc_error_8  = [ 1.697282e+00 ; 1.701048e+00 ; 1.065915e+00 ];

%-- Image #9:
omc_9 = [ -1.579211e+00 ; -1.524154e+00 ; 1.200819e+00 ];
Tc_9  = [ 6.044681e+00 ; -7.766395e+01 ; 1.124795e+03 ];
omc_error_9 = [ 2.141041e-03 ; 1.542378e-03 ; 2.389852e-03 ];
Tc_error_9  = [ 2.295601e+00 ; 2.288238e+00 ; 1.175859e+00 ];

%-- Image #10:
omc_10 = [ -1.603266e+00 ; -1.561116e+00 ; 1.096637e+00 ];
Tc_10  = [ 8.479156e+01 ; -8.832968e+01 ; 1.120556e+03 ];
omc_error_10 = [ 1.992803e-03 ; 1.470071e-03 ; 2.388187e-03 ];
Tc_error_10  = [ 2.281097e+00 ; 2.278441e+00 ; 1.203689e+00 ];

%-- Image #11:
omc_11 = [ -1.173562e+00 ; -1.903341e+00 ; -1.056661e-02 ];
Tc_11  = [ -1.928437e+02 ; -2.123412e+02 ; 1.083226e+03 ];
omc_error_11 = [ 1.574989e-03 ; 1.929065e-03 ; 2.404709e-03 ];
Tc_error_11  = [ 2.218930e+00 ; 2.221841e+00 ; 1.545164e+00 ];

%-- Image #12:
omc_12 = [ -1.579472e+00 ; -2.550377e+00 ; -2.333112e-01 ];
Tc_12  = [ -1.656641e+02 ; -2.110647e+02 ; 1.018387e+03 ];
omc_error_12 = [ 1.654573e-03 ; 2.607985e-03 ; 4.593845e-03 ];
Tc_error_12  = [ 2.090795e+00 ; 2.105573e+00 ; 1.561377e+00 ];

%-- Image #13:
omc_13 = [ -1.368710e+00 ; -2.202397e+00 ; -6.460375e-01 ];
Tc_13  = [ -1.888728e+02 ; -1.284887e+02 ; 8.410500e+02 ];
omc_error_13 = [ 1.252289e-03 ; 2.121727e-03 ; 2.845922e-03 ];
Tc_error_13  = [ 1.714336e+00 ; 1.740724e+00 ; 1.304573e+00 ];

%-- Image #14:
omc_14 = [ -5.007741e-01 ; -2.549932e+00 ; 1.296396e+00 ];
Tc_14  = [ 1.116269e+02 ; -7.990837e+01 ; 1.152986e+03 ];
omc_error_14 = [ 1.649875e-03 ; 2.127428e-03 ; 2.909916e-03 ];
Tc_error_14  = [ 2.339767e+00 ; 2.353164e+00 ; 1.304947e+00 ];

%-- Image #15:
omc_15 = [ 1.722546e-01 ; -2.411963e+00 ; 1.102029e+00 ];
Tc_15  = [ 1.635372e+02 ; -1.004651e+02 ; 1.100928e+03 ];
omc_error_15 = [ 1.317063e-03 ; 2.266217e-03 ; 2.666254e-03 ];
Tc_error_15  = [ 2.240034e+00 ; 2.245391e+00 ; 1.316018e+00 ];

%-- Image #16:
omc_16 = [ 6.301208e-02 ; -3.039249e+00 ; -7.345011e-01 ];
Tc_16  = [ 1.379823e+02 ; -1.132048e+02 ; 7.638624e+02 ];
omc_error_16 = [ 9.397165e-04 ; 2.242951e-03 ; 3.380129e-03 ];
Tc_error_16  = [ 1.559333e+00 ; 1.568359e+00 ; 1.079564e+00 ];

%-- Image #17:
omc_17 = [ 2.086119e-01 ; -2.941235e+00 ; -8.947117e-01 ];
Tc_17  = [ 6.401197e-01 ; -8.063971e+01 ; 7.410109e+02 ];
omc_error_17 = [ 1.236390e-03 ; 2.337999e-03 ; 3.283497e-03 ];
Tc_error_17  = [ 1.505022e+00 ; 1.499758e+00 ; 1.111514e+00 ];

%-- Image #18:
omc_18 = [ -1.937899e+00 ; -1.663099e+00 ; -6.611829e-01 ];
Tc_18  = [ -1.908261e+01 ; -6.801622e+01 ; 7.382678e+02 ];
omc_error_18 = [ 1.378511e-03 ; 2.045139e-03 ; 2.839189e-03 ];
Tc_error_18  = [ 1.505209e+00 ; 1.494044e+00 ; 1.094012e+00 ];

